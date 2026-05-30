// ===================================================================
// Калибровка камеры:
// 1. Снимаем видео 640x480, каждый 5-й кадр ищем Charuco доску (9x13 клеток)
// 2. detectMarkers() находит ArUco маркеры на доске
// 3. interpolateCornersCharuco() вычисляет углы клеток по маркерам
// 4. Сохраняем кадры где найдено ≥15 углов (нужно ≥40 кадров)
// 5. calibrateCameraCharuco() по всем кадрам вычисляет:
//    - cameraMatrix: фокус (fx,fy) и центр (cx,cy) в пикселях
//    - distCoeffs:  искажения линзы (k1,k2,p1,p2,k3)
//    - RMS:         точность (меньше 2 = хорошо)
// Зная параметры камеры, можем точно измерять положение маркера
// ===================================================================
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <iostream>
#include <fstream>
#include <vector>
#include <chrono>
#include <signal.h>

using namespace std;
using namespace cv;
using namespace chrono;

volatile bool running = true;
void sigintHandler(int) { running = false; }

int main() {
    signal(SIGINT, sigintHandler);
    
    // Параметры ChAruco доски 

    int squaresX = 9, squaresY = 13;
    float squareSize = 0.028f, markerSize = 0.019f;  // метры
    int width = 640, height = 480;
    int calibrationTime = 30, minFrames = 40, frameStep = 5;
    
    cout << "\033[2J\033[1;1H";
    cout << "CHARUCO CALIBRATION | " << width << "x" << height 
         << " | " << calibrationTime << "s | every " << frameStep << "th frame" << endl;
    cout << "------------------------------------------------------------" << endl;
    
    VideoCapture cap(0);
    cap.set(CAP_PROP_FOURCC, VideoWriter::fourcc('M', 'J', 'P', 'G'));
    cap.set(CAP_PROP_FRAME_WIDTH, width);
    cap.set(CAP_PROP_FRAME_HEIGHT, height);
    if (!cap.isOpened()) { cerr << "Camera error" << endl; return -1; }
    
    Ptr<aruco::Dictionary> dict = aruco::getPredefinedDictionary(aruco::DICT_4X4_250);
    Ptr<aruco::CharucoBoard> board = aruco::CharucoBoard::create(squaresX, squaresY, squareSize, markerSize, dict);
    Ptr<aruco::DetectorParameters> params = aruco::DetectorParameters::create();
    params->cornerRefinementMethod = aruco::CORNER_REFINE_SUBPIX;
    
    vector<vector<Point2f>> allCorners;
    vector<vector<int>> allIds;
    Mat frame, gray;
    int frameCount = 0, savedCount = 0;
    auto startTime = steady_clock::now();
    
    cout << "Show board to camera..." << endl;
    
    while (running) {
        cap >> frame;
        if (frame.empty()) break;
        
        if (frameCount % frameStep == 0) {
            cvtColor(frame, gray, COLOR_BGR2GRAY);
            
            vector<int> ids;
            vector<vector<Point2f>> corners, rejected;
            aruco::detectMarkers(gray, dict, corners, ids, params, rejected);
            
            if (ids.size() > 5) {
                vector<Point2f> charCorners;
                vector<int> charIds;
                aruco::interpolateCornersCharuco(corners, ids, gray, board, charCorners, charIds);
                
                if (charIds.size() >= 15) {
                    allCorners.push_back(charCorners);
                    allIds.push_back(charIds);
                    savedCount++;
                }
            }
        }
        frameCount++;
        
        double elapsed = duration<double>(steady_clock::now() - startTime).count();
        cout << "\r[" << (int)elapsed << "s] Saved: " << savedCount << "/" << minFrames << flush;
        
        if (elapsed >= calibrationTime && savedCount >= minFrames) break;
        if (elapsed >= 60) break;
    }
    cap.release();
    
    cout << "\n\nFrames: " << savedCount << endl;
    if (savedCount < 15) { cerr << "Not enough frames" << endl; return -1; }
    
    Mat cameraMatrix, distCoeffs;
    vector<Mat> rvecs, tvecs;
    double rms = aruco::calibrateCameraCharuco(allCorners, allIds, board, Size(width, height),
        cameraMatrix, distCoeffs, rvecs, tvecs, CALIB_FIX_K3 | CALIB_ZERO_TANGENT_DIST);
    
    cout << "RMS: " << rms << endl;
    cout << "Camera matrix:\n" << cameraMatrix << endl;
    cout << "Distortion:\n" << distCoeffs << endl;
    
    ofstream f("camera_calibration.json");
    f << "{" << endl;
    f << "  \"width\": " << width << "," << endl;
    f << "  \"height\": " << height << "," << endl;
    f << "  \"fx\": " << cameraMatrix.at<double>(0,0) << "," << endl;
    f << "  \"fy\": " << cameraMatrix.at<double>(1,1) << "," << endl;
    f << "  \"cx\": " << cameraMatrix.at<double>(0,2) << "," << endl;
    f << "  \"cy\": " << cameraMatrix.at<double>(1,2) << "," << endl;
    f << "  \"k1\": " << distCoeffs.at<double>(0) << "," << endl;
    f << "  \"k2\": " << distCoeffs.at<double>(1) << "," << endl;
    f << "  \"p1\": " << distCoeffs.at<double>(2) << "," << endl;
    f << "  \"p2\": " << distCoeffs.at<double>(3) << "," << endl;
    f << "  \"k3\": " << distCoeffs.at<double>(4) << "," << endl;
    f << "  \"rms\": " << rms << endl;
    f << "}" << endl;
    f.close();
    cout << "Saved: camera_calibration.json" << endl;
    
    return 0;
}