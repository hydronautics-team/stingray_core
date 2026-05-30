#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>
#include <iomanip>
#include <chrono>
#include <vector>
#include <cmath>
#include <signal.h>
#include <deque>
#include <fstream>
#include <algorithm>

using namespace std;
using namespace cv;
using namespace chrono;

volatile bool running = true;
void sigintHandler(int) { running = false; }

// Медианный фильтр (лучше скользящего среднего для углов)
class MedianFilter {
private:
    deque<double> buffer;
    size_t maxSize;
public:
    MedianFilter(size_t size) : maxSize(size) {}
    
    double filter(double value) {
        buffer.push_back(value);
        if (buffer.size() > maxSize) buffer.pop_front();
        
        vector<double> sorted(buffer.begin(), buffer.end());
        sort(sorted.begin(), sorted.end());
        return sorted[sorted.size() / 2];
    }
    
    void reset() { buffer.clear(); }
};

class Vec3MedianFilter {
private:
    MedianFilter x, y, z;
public:
    Vec3MedianFilter(size_t size) : x(size), y(size), z(size) {}
    Vec3d filter(const Vec3d& v) {
        return Vec3d(x.filter(v[0]), y.filter(v[1]), z.filter(v[2]));
    }
    void reset() { x.reset(); y.reset(); z.reset(); }
};

Vec3d rvecToEuler(const Vec3d& rvec) {
    Mat R;
    Rodrigues(rvec, R);
    R = R.t();
    double roll, pitch, yaw;
    yaw   = atan2(R.at<double>(1, 0), R.at<double>(0, 0));
    pitch = asin(-R.at<double>(2, 0));
    roll  = atan2(R.at<double>(2, 1), R.at<double>(2, 2));

    yaw   = yaw * 180.0 / CV_PI;
    pitch = -pitch * 180.0 / CV_PI;
    roll  = roll * 180.0 / CV_PI;

    if (yaw < 0) yaw += 360.0;
    roll += 180.0;
    if (roll > 180.0) roll -= 360.0;

    return Vec3d(yaw, roll, pitch);
}

Vec3d compensateTilt(const Vec3d& tvec, const Vec3d& euler) {
    double pitch = euler[1] * CV_PI / 180.0;  
    double roll  = euler[2] * CV_PI / 180.0;  
    double x = tvec[0], y = tvec[1], z = tvec[2];

    if (abs(euler[1]) <= 15.0) x = x * cos(pitch) + z * sin(pitch);
    if (abs(euler[2]) <= 15.0) y = y * cos(roll) + z * sin(roll);
    
    return Vec3d(y, x, z);
}

bool loadCalibration(const string& filename, Mat& cameraMatrix, Mat& distCoeffs, int& width, int& height) {
    ifstream f(filename);
    if (!f.is_open()) return false;
    
    string key;
    double fx = 300, fy = 300, cx = 320, cy = 240;
    double k1 = 0, k2 = 0, p1 = 0, p2 = 0, k3 = 0;
    
    while (f >> key) {
        if (key.find("\"width\"") != string::npos) { f >> key; width = stoi(key); }
        if (key.find("\"height\"") != string::npos) { f >> key; height = stoi(key); }
        if (key.find("\"fx\"") != string::npos) { f >> key; fx = stod(key); }
        if (key.find("\"fy\"") != string::npos) { f >> key; fy = stod(key); }
        if (key.find("\"cx\"") != string::npos) { f >> key; cx = stod(key); }
        if (key.find("\"cy\"") != string::npos) { f >> key; cy = stod(key); }
        if (key.find("\"k1\"") != string::npos) { f >> key; k1 = stod(key); }
        if (key.find("\"k2\"") != string::npos) { f >> key; k2 = stod(key); }
        if (key.find("\"p1\"") != string::npos) { f >> key; p1 = stod(key); }
        if (key.find("\"p2\"") != string::npos) { f >> key; p2 = stod(key); }
        if (key.find("\"k3\"") != string::npos) { f >> key; k3 = stod(key); }
    }
    f.close();
    
    cameraMatrix = (Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
    distCoeffs = (Mat_<double>(5, 1) << k1, k2, p1, p2, k3);
    return true;
}

// Проверка на выброс: если значение прыгнуло >10° за 1 кадр — игнорируем
bool isAngleValid(double current, double previous) {
    if (previous == -999) return true;  // первый кадр
    return abs(current - previous) < 10.0;
}

int main() {
    signal(SIGINT, sigintHandler);
    
    Mat cameraMatrix, distCoeffs;
    int width = 640, height = 480;
    
    if (!loadCalibration("camera_calibration.json", cameraMatrix, distCoeffs, width, height)) {
        cameraMatrix = (Mat_<double>(3, 3) << 300, 0, 320, 0, 300, 240, 0, 0, 1);
        distCoeffs = (Mat_<double>(5, 1) << 0, 0, 0, 0, 0);
    }
    
    float markerSize = 0.10f;
    Vec3MedianFilter angleFilter(7);  // Медианный фильтр по 7 кадрам
    Vec3MedianFilter posFilter(7);    // Медианный фильтр по 7 кадрам
    
    cout << "\033[2J\033[1;1H";
    cout << "============================================================" << endl;
    cout << "ArUco MARKER DETECTION" << endl;
    cout << "============================================================" << endl;
    
    VideoCapture cap(0);
    cap.set(CAP_PROP_FOURCC, VideoWriter::fourcc('M', 'J', 'P', 'G'));
    cap.set(CAP_PROP_FRAME_WIDTH, width);
    cap.set(CAP_PROP_FRAME_HEIGHT, height);
    
    if (!cap.isOpened()) { cerr << "Camera error" << endl; return -1; }
    
    Ptr<aruco::Dictionary> dict = aruco::getPredefinedDictionary(aruco::DICT_4X4_50);
    Ptr<aruco::DetectorParameters> params = aruco::DetectorParameters::create();
    params->cornerRefinementMethod = aruco::CORNER_REFINE_SUBPIX;
    params->cornerRefinementWinSize = 3;
    
    cout << "Camera: " << width << "x" << height << " | Marker: " << markerSize*100 << "cm" << endl;
    cout << "Filter: Median(7) + outlier rejection" << endl;
    cout << "------------------------------------------------------------" << endl;
    
    Mat frame, gray, processed;
    vector<int> ids, lastIds;
    vector<vector<Point2f>> corners, lastCorners;
    vector<Vec3d> rvecs, tvecs, lastRvecs, lastTvecs;
    
    int frameCount = 0, fpsFrames = 0, framesWithoutMarker = 0;
    double fps = 0;
    auto fpsStart = steady_clock::now(), lastPrint = steady_clock::now();
    const int maxFramesWithout = 10;
    
    // Предыдущие значения для проверки выбросов
    double prevYaw = -999, prevRoll = -999, prevPitch = -999;
    
    while (running) {
        cap >> frame;
        if (frame.empty()) break;
        
        if (frameCount % 3 == 0) {
            cvtColor(frame, gray, COLOR_BGR2GRAY);
            GaussianBlur(gray, processed, Size(3, 3), 0);
            adaptiveThreshold(processed, processed, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, 9, 3);
            
            aruco::detectMarkers(processed, dict, corners, ids, params);
            if (ids.empty()) aruco::detectMarkers(gray, dict, corners, ids, params);
            
            if (!ids.empty()) {
                aruco::estimatePoseSingleMarkers(corners, markerSize, cameraMatrix, distCoeffs, rvecs, tvecs);
                lastIds = ids; lastCorners = corners; lastRvecs = rvecs; lastTvecs = tvecs;
                framesWithoutMarker = 0;
            } else framesWithoutMarker++;
        } else {
            if (framesWithoutMarker < maxFramesWithout) {
                ids = lastIds; corners = lastCorners; rvecs = lastRvecs; tvecs = lastTvecs;
            } else ids.clear();
        }
        
        fpsFrames++;
        auto now = steady_clock::now();
        if (duration<double>(now - fpsStart).count() >= 1.0) {
            fps = fpsFrames / duration<double>(now - fpsStart).count();
            fpsFrames = 0; fpsStart = now;
        }
        
        if (duration<double>(now - lastPrint).count() >= 0.5) {
            cout << "\r[FPS:" << fixed << setprecision(0) << fps << "] ";
            if (!ids.empty()) {
                cout << ids.size() << " marker(s): ";
                for (size_t i = 0; i < ids.size(); i++) {
                    Vec3d rawAngles = rvecToEuler(rvecs[i]);
                    
                    // Проверка на выбросы
                    double yaw = rawAngles[0], roll = rawAngles[1], pitch = rawAngles[2];
                    if (!isAngleValid(yaw, prevYaw)) yaw = prevYaw;
                    if (!isAngleValid(roll, prevRoll)) roll = prevRoll;
                    if (!isAngleValid(pitch, prevPitch)) pitch = prevPitch;
                    
                    prevYaw = yaw; prevRoll = roll; prevPitch = pitch;
                    
                    Vec3d filteredAngles = angleFilter.filter(Vec3d(yaw, roll, pitch));
                    Vec3d compensatedPos = compensateTilt(tvecs[i], filteredAngles);
                    Vec3d filteredPos = posFilter.filter(compensatedPos);
                    
                    cout << "ID:" << ids[i] << "|x:" << fixed << setprecision(2) << filteredPos[0]
                         << " y:" << filteredPos[1] << " z:" << filteredPos[2] << "m"
                         << " yaw:" << setprecision(0) << filteredAngles[0] << "\u00B0"
                         << " roll:" << filteredAngles[1] << "\u00B0"
                         << " pitch:" << filteredAngles[2] << "\u00B0";
                    if (i < ids.size() - 1) cout << " | ";
                }
                cout << "   " << flush;
            } else {
                posFilter.reset(); angleFilter.reset();
                prevYaw = prevRoll = prevPitch = -999;
                cout << "Searching...   " << flush;
            }
            lastPrint = now;
        }
        frameCount++;
    }
    
    cap.release();
    cout << "\n\nSTOPPED" << endl;
    return 0;
}