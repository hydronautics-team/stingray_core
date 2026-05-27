#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>
#include <iomanip>
#include <chrono>
#include <vector>
#include <cmath>
#include <signal.h>

using namespace std;
using namespace cv;
using namespace chrono;

volatile bool running = true;

void sigintHandler(int) {
    running = false;
}

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

int main() {
    signal(SIGINT, sigintHandler);
    
    Mat cameraMatrix = (Mat_<double>(3, 3) <<
        839.067428091236, 0, 953.5271775639978,
        0, 869.6335345962565, 544.2180085851491,
        0, 0, 1);
    
    Mat distCoeffs = (Mat_<double>(5, 1) <<
        0.271577650144495, 0.3342575649939697, 0, 0, 0);
    
    float markerSize = 0.10f;
    
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
    
    // Настройка параметров детектора
    Ptr<aruco::DetectorParameters> params = aruco::DetectorParameters::create();
    params->adaptiveThreshWinSizeMin = 3;
    params->adaptiveThreshWinSizeMax = 23;
    params->adaptiveThreshWinSizeStep = 10;
    params->minMarkerPerimeterRate = 0.03;  // Минимальный размер маркера (3% от кадра)
    params->maxMarkerPerimeterRate = 0.8;   // Максимальный размер (80% от кадра)
    params->polygonalApproxAccuracyRate = 0.03;
    params->minCornerDistanceRate = 0.01;
    params->minDistanceToBorder = 3;
    params->cornerRefinementMethod = aruco::CORNER_REFINE_SUBPIX;
    params->cornerRefinementWinSize = 5;
    params->cornerRefinementMaxIterations = 30;
    params->cornerRefinementMinAccuracy = 0.1;
    params->markerBorderBits = 1;
    params->perspectiveRemovePixelPerCell = 4;
    params->perspectiveRemoveIgnoredMarginPerCell = 0.13;
    params->maxErroneousBitsInBorderRate = 0.35;
    params->minOtsuStdDev = 5.0;
    params->errorCorrectionRate = 0.6;
    
    cout << "Camera: MJPG 1920x1080" << endl;
    cout << "Marker size: " << markerSize * 100 << " cm" << endl;
    cout << "Dictionary: DICT_4X4_100" << endl;
    cout << "Detection: every 3rd frame" << endl;
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
                    Vec3d angles = rvecToEuler(rvecs[i]);
                    cout << "ID:" << ids[i]
                         << "|x:" << fixed << setprecision(2) << tvecs[i][0]
                         << " y:" << tvecs[i][1]
                         << " z:" << tvecs[i][2] << "m"
                         << " yaw:" << setprecision(0) << angles[0] << "\u00B0";
                    if (i < ids.size() - 1) cout << " | ";
                }
                cout << "   " << flush;
            } else {
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