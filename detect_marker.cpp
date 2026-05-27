#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>
#include <iomanip>
#include <chrono>
#include <vector>
#include <cmath>
#include <signal.h>
#include <deque>

using namespace std;
using namespace cv;
using namespace chrono;

volatile bool running = true;

void sigintHandler(int) {
    running = false;
}

// Простой фильтр скользящего среднего
class MovingAverage {
private:
    deque<double> buffer;
    size_t maxSize;
public:
    MovingAverage(size_t size) : maxSize(size) {}
    
    double filter(double value) {
        buffer.push_back(value);
        if (buffer.size() > maxSize) buffer.pop_front();
        
        double sum = 0;
        for (double v : buffer) sum += v;
        return sum / buffer.size();
    }
    
    void reset() { buffer.clear(); }
};

// Фильтр для Vec3d (X, Y, Z)
class Vec3Filter {
private:
    MovingAverage x, y, z;
public:
    Vec3Filter(size_t size) : x(size), y(size), z(size) {}
    
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

// Компенсация наклонов (крен, дифферент) для X и Y
Vec3d compensateTilt(const Vec3d& tvec, const Vec3d& euler) {
    double pitch = euler[1] * CV_PI / 180.0;  
    double roll  = euler[2] * CV_PI / 180.0;  
    
    double x = tvec[0];
    double y = tvec[1];
    double z = tvec[2];
    
    if (abs(euler[1]) <= 15.0) {
        x = x * cos(pitch) + z * sin(pitch);
    }
    if (abs(euler[2]) <= 15.0) {
        y = y * cos(roll) + z * sin(roll);
    }
    
    return Vec3d(y, x, z);
}

int main() {
    signal(SIGINT, sigintHandler);
    
    Mat cameraMatrix = (Mat_<double>(3, 3) <<
        839.067428091236, 0, 953.5271775639978,
        0, 869.6335345962565, 544.2180085851491,
        0, 0, 1);
    
    Mat distCoeffs = (Mat_<double>(5, 1) <<
        0.271577650144495, 0.3342575649939697, 0, 0, 0);
    
    float markerSize = 0.10f;
    
    // Фильтры (размер окна = 5 для плавности без большой задержки)
    Vec3Filter posFilter(5);   // X, Y, Z
    Vec3Filter angleFilter(3); // Yaw, Roll, Pitch
    
    cout << "\033[2J\033[1;1H";
    cout << "============================================================" << endl;
    cout << "ArUco MARKER DETECTION (C++)" << endl;
    cout << "============================================================" << endl;
    cout << "Opening camera..." << endl;
    
    VideoCapture cap(0, CAP_V4L2);
    cap.set(CAP_PROP_FOURCC, VideoWriter::fourcc('M', 'J', 'P', 'G'));
    cap.set(CAP_PROP_FRAME_WIDTH, 1920);
    cap.set(CAP_PROP_FRAME_HEIGHT, 1080);
    
    if (!cap.isOpened()) {
        cerr << "Error: Cannot open camera" << endl;
        return -1;
    }
    
    Ptr<aruco::Dictionary> dict = aruco::getPredefinedDictionary(aruco::DICT_4X4_100);
    
    Ptr<aruco::DetectorParameters> params = aruco::DetectorParameters::create();
    params->adaptiveThreshWinSizeMin = 3;
    params->adaptiveThreshWinSizeMax = 23;
    params->adaptiveThreshWinSizeStep = 10;
    params->minMarkerPerimeterRate = 0.03;
    params->maxMarkerPerimeterRate = 0.8;
    params->polygonalApproxAccuracyRate = 0.03;
    params->cornerRefinementMethod = aruco::CORNER_REFINE_SUBPIX;
    params->cornerRefinementWinSize = 5;
    params->errorCorrectionRate = 0.6;
    
    cout << "Camera: MJPG 1920x1080" << endl;
    cout << "Marker size: " << markerSize * 100 << " cm" << endl;
    cout << "Filters: pos(5), angle(3), tilt compensation ON" << endl;
    cout << "Press Ctrl+C to exit" << endl;
    cout << "------------------------------------------------------------" << endl;
    
    Mat frame, gray;
    vector<int> ids, lastIds;
    vector<vector<Point2f>> corners, lastCorners;
    vector<Vec3d> rvecs, tvecs, lastRvecs, lastTvecs;
    
    int frameCount = 0;
    double fps = 0;
    auto fpsStart = steady_clock::now();
    int fpsFrames = 0;
    auto lastPrint = steady_clock::now();
    
    while (running) {
        cap >> frame;
        if (frame.empty()) break;
        
        if (frameCount % 3 == 0) {
            cvtColor(frame, gray, COLOR_BGR2GRAY);
            aruco::detectMarkers(gray, dict, corners, ids, params);
            
            if (!ids.empty()) {
                aruco::estimatePoseSingleMarkers(corners, markerSize, cameraMatrix, distCoeffs, rvecs, tvecs);
            }
            
            lastIds = ids;
            lastCorners = corners;
            lastRvecs = rvecs;
            lastTvecs = tvecs;
        } else {
            ids = lastIds;
            corners = lastCorners;
            rvecs = lastRvecs;
            tvecs = lastTvecs;
        }
        
        fpsFrames++;
        auto now = steady_clock::now();
        double elapsed = duration<double>(now - fpsStart).count();
        
        if (elapsed >= 1.0) {
            fps = fpsFrames / elapsed;
            fpsFrames = 0;
            fpsStart = now;
        }
        
        if (duration<double>(now - lastPrint).count() >= 0.5) {
            cout << "\r[FPS:" << fixed << setprecision(0) << fps << "] ";
            
            if (!ids.empty()) {
                cout << ids.size() << " marker(s): ";
                for (size_t i = 0; i < ids.size(); i++) {
                    Vec3d rawAngles = rvecToEuler(rvecs[i]);
                    
                    // Компенсация наклонов
                    Vec3d compensatedPos = compensateTilt(tvecs[i], rawAngles);
                    
                    // Фильтрация
                    Vec3d filteredPos = posFilter.filter(compensatedPos);
                    Vec3d filteredAngles = angleFilter.filter(rawAngles);
                    
                    cout << "ID:" << ids[i]
                         << "|x:" << fixed << setprecision(2) << filteredPos[0]
                         << " y:" << filteredPos[1]
                         << " z:" << filteredPos[2] << "m"
                         << " yaw:" << setprecision(0) << filteredAngles[0] << "\u00B0"
                         << " roll:" << filteredAngles[1] << "\u00B0"
                         << " pitch:" << filteredAngles[2] << "\u00B0";
                    if (i < ids.size() - 1) cout << " | ";
                }
                cout << "   " << flush;
            } else {
                posFilter.reset();
                angleFilter.reset();
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