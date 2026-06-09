#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <iostream>
#include <sstream>
#include <thread>
#include <mutex>
#include <iomanip>
#include <chrono>
#include <vector>
#include <cmath>
#include <signal.h>
#include <fstream>

#include "detect_marker/http_server.hpp"

using namespace std;
using namespace cv;
using namespace chrono;

volatile bool running = true;
Mat globalFrame;
mutex frameMutex;
string globalJson = "{}";
mutex jsonMutex;

// ВСЕ ТЕ ЖЕ ФИЛЬТРЫ
class YawFilter {
    double alpha, value;
    bool first;
public:
    YawFilter(double a = 0.3) : alpha(a), value(0), first(true) {}
    double filter(double v) {
        if (first) { value = v; first = false; return v; }
        double diff = v - value;
        if (diff > 180) diff -= 360;
        if (diff < -180) diff += 360;
        value += alpha * diff;
        if (value < 0) value += 360;
        if (value >= 360) value -= 360;
        return value;
    }
    void reset() { first = true; }
};

class RollFilter {
    double alpha, value;
    bool first;
public:
    RollFilter(double a = 0.3) : alpha(a), value(0), first(true) {}
    double filter(double v) {
        if (first) { value = v; first = false; return v; }
        double diff = v - value;
        if (diff > 180) diff -= 360;
        if (diff < -180) diff += 360;
        value += alpha * diff;
        if (value > 180) value -= 360;
        if (value < -180) value += 360;
        return value;
    }
    void reset() { first = true; }
};

class PitchFilter {
    double alpha, value;
    bool first;
public:
    PitchFilter(double a = 0.3) : alpha(a), value(0), first(true) {}
    double filter(double v) {
        if (first) { value = v; first = false; return v; }
        value += alpha * (v - value);
        return value;
    }
    void reset() { first = true; }
};

class PosFilter {
    double alpha, value;
    bool first;
public:
    PosFilter(double a = 0.3) : alpha(a), value(0), first(true) {}
    double filter(double v) {
        if (first) { value = v; first = false; return v; }
        value += alpha * (v - value);
        return value;
    }
    void reset() { first = true; }
};

// Фильтр для скорости
class VelocityFilter {
    double alpha, value;
    bool first;
public:
    VelocityFilter(double a = 0.5) : alpha(a), value(0), first(true) {}
    double filter(double v) {
        if (first) { value = v; first = false; return v; }
        value = alpha * v + (1 - alpha) * value;
        return value;
    }
    void reset() { first = true; }
};

Vec3d rvecToEuler(const Vec3d& rvec) {
    Mat R; Rodrigues(rvec, R); R = R.t();
    double yaw = atan2(R.at<double>(1,0), R.at<double>(0,0)) * 180.0/CV_PI;
    double pitch = -asin(-R.at<double>(2,0)) * 180.0/CV_PI;
    double roll = atan2(R.at<double>(2,1), R.at<double>(2,2)) * 180.0/CV_PI;
    if (yaw < 0) yaw += 360.0;
    roll += 180.0; if (roll > 180.0) roll -= 360.0;
    return Vec3d(yaw, pitch, roll);
}

Vec3d compensateTilt(const Vec3d& tvec, const Vec3d& euler) {
    double p = euler[1] * CV_PI/180.0, r = euler[2] * CV_PI/180.0;
    double x = tvec[0], y = tvec[1], z = tvec[2];
    if (abs(euler[1]) <= 15.0) x = x*cos(p) + z*sin(p);
    if (abs(euler[2]) <= 15.0) y = y*cos(r) + z*sin(r);
    return Vec3d(y, -x, z);
}

bool loadCalibration(const string& fn, Mat& cam, Mat& dist, int& w, int& h) {
    ifstream f(fn); if (!f.is_open()) return false;
    string k; double fx=300,fy=300,cx=320,cy=240,k1=0,k2=0,p1=0,p2=0,k3=0;
    while (f >> k) {
        if (k.find("\"fx\"")!=string::npos){f>>k;fx=stod(k);}
        if (k.find("\"fy\"")!=string::npos){f>>k;fy=stod(k);}
        if (k.find("\"cx\"")!=string::npos){f>>k;cx=stod(k);}
        if (k.find("\"cy\"")!=string::npos){f>>k;cy=stod(k);}
        if (k.find("\"k1\"")!=string::npos){f>>k;k1=stod(k);}
        if (k.find("\"k2\"")!=string::npos){f>>k;k2=stod(k);}
        if (k.find("\"p1\"")!=string::npos){f>>k;p1=stod(k);}
        if (k.find("\"p2\"")!=string::npos){f>>k;p2=stod(k);}
        if (k.find("\"k3\"")!=string::npos){f>>k;k3=stod(k);}
    }
    cam = (Mat_<double>(3,3)<<fx,0,cx,0,fy,cy,0,0,1);
    dist = (Mat_<double>(5,1)<<k1,k2,p1,p2,k3);
    return true;
}

void signalHandler(int) {
    running = false;
}

int main(int argc, char** argv) {
    signal(SIGINT, signalHandler);
    
    rclcpp::init(argc, argv);
    auto node = make_shared<rclcpp::Node>("detect_marker_node");
    auto pub_yaw = node->create_publisher<std_msgs::msg::Float64>("/control/vision/yaw", 10);
    auto pub_depth = node->create_publisher<std_msgs::msg::Float64>("/control/vision/depth", 10);
    
    // Публикации для скоростей
    auto pub_yaw_rate = node->create_publisher<std_msgs::msg::Float64>("/control/vision/yaw_rate", 10);
    auto pub_z_velocity = node->create_publisher<std_msgs::msg::Float64>("/control/vision/z_velocity", 10);
    
    Mat camMat, dist;
    int w=640, h=480;
    loadCalibration("camera_calibration.json", camMat, dist, w, h);
    
    float mSize = 0.10f;
    
    YawFilter   angF_yaw(0.3);
    RollFilter  angF_roll(0.3);
    PitchFilter angF_pitch(0.3);
    PosFilter posF_x(0.3), posF_y(0.3), posF_z(0.3);
    
    // Фильтры для скорости
    VelocityFilter velF_yaw(0.3);
    VelocityFilter velF_z(0.3);
    
    double lastX=0, lastY=0, lastZ=0, lastYaw=0, lastRoll=0, lastPitch=0;
    double lastYawRate = 0, lastZVelocity = 0;
    int lastId = -1;
    
    // Для вычисления скорости
    auto lastTime = steady_clock::now();
    double lastYawForVel = 0;
    double lastZForVel = 0;
    bool firstVel = true;
    
    cout << "\033[2J\033[1;1H";
    cout << "ArUco + ROS2 + HTTP + VELOCITY" << endl;
    cout << "============================================================" << endl;
    
    VideoCapture cap(0);
    cap.set(CAP_PROP_FOURCC, VideoWriter::fourcc('M','J','P','G'));
    cap.set(CAP_PROP_FRAME_WIDTH, w); cap.set(CAP_PROP_FRAME_HEIGHT, h);
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
    
    while (running) {
        cap >> frame;
        if (frame.empty()) break;
        Mat disp = frame.clone();
        
        auto now = steady_clock::now();
        double dt = duration<double>(now - lastTime).count();
        if (dt > 0.1) dt = 0.033;
        
        cvtColor(frame, gray, COLOR_BGR2GRAY);
        GaussianBlur(gray, proc, Size(3,3), 0);
        adaptiveThreshold(proc, proc, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, 9, 3);
        aruco::detectMarkers(proc, dict, corners, ids, params);
        if (ids.empty()) aruco::detectMarkers(gray, dict, corners, ids, params);
        
        stringstream js;
        js << "{\"markers\":[";
        
        if (!ids.empty()) {
            aruco::estimatePoseSingleMarkers(corners, mSize, camMat, dist, rvecs, tvecs);
            
            Vec3d ang = rvecToEuler(rvecs[0]);
            double fy = angF_yaw.filter(ang[0]);
            double fr = angF_roll.filter(ang[1]);
            double fp = angF_pitch.filter(ang[2]);
            
            Vec3d cPos = compensateTilt(tvecs[0], Vec3d(fy, fr, fp));
            double fx = posF_x.filter(cPos[0]);
            double fyy = posF_y.filter(cPos[1]);
            double fz = posF_z.filter(cPos[2]);
            
            // ВЫЧИСЛЯЕМ СКОРОСТИ (только для ROS и терминала)
            if (!firstVel && dt > 0) {
                double yawDiff = fy - lastYawForVel;
                if (yawDiff > 180) yawDiff -= 360;
                if (yawDiff < -180) yawDiff += 360;
                double rawYawRate = yawDiff / dt;
                lastYawRate = velF_yaw.filter(rawYawRate);
                
                double rawZVel = (fz - lastZForVel) / dt;
                lastZVelocity = velF_z.filter(rawZVel);
            }
            
            lastX = fx; lastY = fyy; lastZ = fz;
            lastYaw = fy; lastRoll = fr; lastPitch = fp;
            lastId = ids[0];
            lastYawForVel = fy;
            lastZForVel = fz;
            firstVel = false;
            
            // JSON БЕЗ СКОРОСТЕЙ
            js << "{\"id\":" << ids[0]
               << ",\"x\":" << fixed << setprecision(3) << fx
               << ",\"y\":" << fyy
               << ",\"z\":" << fz
               << ",\"yaw\":" << setprecision(1) << fy
               << ",\"roll\":" << fr
               << ",\"pitch\":" << fp << "}";
            
            aruco::drawDetectedMarkers(disp, corners, ids);
            aruco::drawAxis(disp, camMat, dist, rvecs[0], tvecs[0], 0.05);
            
        } else if (lastId >= 0) {
            js << "{\"id\":" << lastId
               << ",\"x\":" << fixed << setprecision(3) << lastX
               << ",\"y\":" << lastY
               << ",\"z\":" << lastZ
               << ",\"yaw\":" << setprecision(1) << lastYaw
               << ",\"roll\":" << lastRoll
               << ",\"pitch\":" << lastPitch << "}";
        }
        js << "]}";
        
        // HTTP
        {
            lock_guard<mutex> lk(jsonMutex);
            globalJson = js.str();
        }
        {
            lock_guard<mutex> lk(frameMutex);
            globalFrame = disp.clone();
        }
        
        // ROS публикации
        auto msg_yaw = std_msgs::msg::Float64();
        msg_yaw.data = lastYaw;
        pub_yaw->publish(msg_yaw);
        
        auto msg_depth = std_msgs::msg::Float64();
        msg_depth.data = lastZ;
        pub_depth->publish(msg_depth);
        
        // Публикуем скорости в ROS
        auto msg_yaw_rate = std_msgs::msg::Float64();
        msg_yaw_rate.data = lastYawRate;
        pub_yaw_rate->publish(msg_yaw_rate);
        
        auto msg_z_velocity = std_msgs::msg::Float64();
        msg_z_velocity.data = lastZVelocity;
        pub_z_velocity->publish(msg_z_velocity);
        
        lastTime = now;
        
        // ВЫВОД В ТЕРМИНАЛ (со скоростями)
        static auto tPrint = steady_clock::now();
        auto printNow = steady_clock::now();
        if (duration<double>(printNow - tPrint).count() >= 0.5) {
            cout << "\r";
            if (!ids.empty() || lastId >= 0) {
                cout << "ID:" << lastId 
                     << " | X:" << fixed << setprecision(3) << lastX 
                     << " Y:" << lastY << " Z:" << lastZ
                     << " | Yaw:" << setprecision(1) << lastYaw << "°"
                     << " | Yaw_rate:" << setprecision(2) << lastYawRate << "°/s"
                     << " | Z_vel:" << lastZVelocity << " m/s   " << flush;
            } else {
                cout << "Searching...   " << flush;
            }
            tPrint = printNow;
        }
    }
    
    cap.release();
    http.join();
    rclcpp::shutdown();
    cout << "\n\nSTOPPED" << endl;
    return 0;
}