#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <iostream>
#include <sstream>
#include <thread>
#include <mutex>
#include <vector>
#include <cmath>

#include "detect_marker/filters.hpp"
#include "detect_marker/utils.hpp"
#include "detect_marker/http_server.hpp"

using namespace std;
using namespace cv;

volatile bool running = true;
Mat globalFrame;
mutex frameMutex;
string globalJson = "{}";
mutex jsonMutex;

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = make_shared<rclcpp::Node>("detect_marker_node");
    auto pub_yaw = node->create_publisher<std_msgs::msg::Float64>("/control/vision/yaw", 10);
    auto pub_depth = node->create_publisher<std_msgs::msg::Float64>("/control/vision/depth", 10);

    Mat camMat, dist;
    int w = 640, h = 480;
    loadCalibration("camera_calibration.json", camMat, dist, w, h);
    float mSize = 0.10f;

    YawFilter angF_yaw(0.3);
    PosFilter posF_z(0.3);
    double lastYaw = 0, lastZ = 0;
    int lastId = -1;

    cout << "\033[2J\033[1;1H";
    cout << "detect_marker (ROS2)" << endl;
    cout << "============================================================" << endl;

    VideoCapture cap(0);
    cap.set(CAP_PROP_FOURCC, VideoWriter::fourcc('M', 'J', 'P', 'G'));
    cap.set(CAP_PROP_FRAME_WIDTH, w);
    cap.set(CAP_PROP_FRAME_HEIGHT, h);
    if (!cap.isOpened()) { cerr << "Camera error" << endl; return 1; }

    thread http(httpServer, 8080);

    auto dict = aruco::getPredefinedDictionary(aruco::DICT_4X4_50);
    auto params = aruco::DetectorParameters::create();
    params->cornerRefinementMethod = aruco::CORNER_REFINE_SUBPIX;
    params->cornerRefinementWinSize = 3;

    Mat frame, gray, proc;
    vector<int> ids;
    vector<vector<Point2f>> corners;
    vector<Vec3d> rvecs, tvecs;

    while (running && rclcpp::ok()) {
        cap >> frame;
        if (frame.empty()) break;
        Mat disp = frame.clone();

        cvtColor(frame, gray, COLOR_BGR2GRAY);
        GaussianBlur(gray, proc, Size(3, 3), 0);
        adaptiveThreshold(proc, proc, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, 9, 3);
        aruco::detectMarkers(proc, dict, corners, ids, params);
        if (ids.empty()) aruco::detectMarkers(gray, dict, corners, ids, params);

        if (!ids.empty()) {
            aruco::estimatePoseSingleMarkers(corners, mSize, camMat, dist, rvecs, tvecs);

            Mat R;
            Rodrigues(rvecs[0], R);
            R = R.t();
            double rawYaw = atan2(R.at<double>(1, 0), R.at<double>(0, 0)) * 180.0 / CV_PI;
            if (rawYaw < 0) rawYaw += 360.0;

            lastYaw = angF_yaw.filter(rawYaw);
            lastZ = posF_z.filter(tvecs[0][2]);
            lastId = ids[0];

            auto msg_yaw = std_msgs::msg::Float64();
            msg_yaw.data = lastYaw;
            pub_yaw->publish(msg_yaw);

            auto msg_depth = std_msgs::msg::Float64();
            msg_depth.data = lastZ;
            pub_depth->publish(msg_depth);

            aruco::drawDetectedMarkers(disp, corners, ids);
            aruco::drawAxis(disp, camMat, dist, rvecs[0], tvecs[0], 0.05);
        } else if (lastId >= 0) {
            auto msg_yaw = std_msgs::msg::Float64();
            msg_yaw.data = lastYaw;
            pub_yaw->publish(msg_yaw);
            auto msg_depth = std_msgs::msg::Float64();
            msg_depth.data = lastZ;
            pub_depth->publish(msg_depth);
        }

        {
            lock_guard<mutex> lk(frameMutex);
            globalFrame = disp.clone();
        }

        rclcpp::spin_some(node);
        cout << "\r[OK] yaw=" << lastYaw << "° depth=" << lastZ << "m   " << flush;
    }

    cap.release();
    http.join();
    rclcpp::shutdown();
    cout << "\nSTOPPED" << endl;
    return 0;
}