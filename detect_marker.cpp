#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
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
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>

using namespace std;
using namespace cv;
using namespace chrono;

volatile bool running = true;
void sigintHandler(int) { running = false; }

Mat globalFrame;
mutex frameMutex;
string globalJson = "{}";
mutex jsonMutex;

class YawFilter {
    double alpha, value;
    bool first;
public:
    YawFilter(double a = 0.5) : alpha(a), value(0), first(true) {}
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
};

class RollFilter {
    double alpha, value;
    bool first;
public:
    RollFilter(double a = 0.5) : alpha(a), value(0), first(true) {}
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
};

class PitchFilter {
    double alpha, value;
    bool first;
public:
    PitchFilter(double a = 0.5) : alpha(a), value(0), first(true) {}
    double filter(double v) {
        if (first) { value = v; first = false; return v; }
        value += alpha * (v - value);
        return value;
    }
};

class PosFilter {
    double alpha, value;
    bool first;
public:
    PosFilter(double a = 0.5) : alpha(a), value(0), first(true) {}
    double filter(double v) {
        if (first) { value = v; first = false; return v; }
        value += alpha * (v - value);
        return value;
    }
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

void httpServer(int port) {
    int fd = socket(AF_INET, SOCK_STREAM, 0), opt = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    sockaddr_in addr{}; addr.sin_family=AF_INET; addr.sin_addr.s_addr=INADDR_ANY; addr.sin_port=htons(port);
    bind(fd,(sockaddr*)&addr,sizeof(addr)); listen(fd,3);
    cout << "HTTP: http://0.0.0.0:" << port << "/stream | /data | /video" << endl;
    
    while (running) {
        int cl = accept(fd,nullptr,nullptr); if(cl<0) continue;
        char buf[2048]{}; read(cl,buf,2047); string req(buf);
        
        if (req.find("GET /stream") != string::npos) {
            string hdr = "HTTP/1.1 200 OK\r\nContent-Type: multipart/x-mixed-replace; boundary=frame\r\nAccess-Control-Allow-Origin: *\r\n\r\n";
            send(cl,hdr.c_str(),hdr.size(),0);
            while (running) {
                Mat f; { lock_guard<mutex> lk(frameMutex); if(!globalFrame.empty()) f=globalFrame.clone(); }
                if(!f.empty()) {
                    vector<uchar> jpg; imencode(".jpg",f,jpg,{IMWRITE_JPEG_QUALITY,50});
                    string b = "--frame\r\nContent-Type: image/jpeg\r\n\r\n";
                    send(cl,b.c_str(),b.size(),0); send(cl,(char*)jpg.data(),jpg.size(),0); send(cl,"\r\n",2,0);
                }
                this_thread::sleep_for(milliseconds(33));
            }
        } else if (req.find("GET /data") != string::npos) {
            string j; { lock_guard<mutex> lk(jsonMutex); j=globalJson; }
            string rsp = "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nAccess-Control-Allow-Origin: *\r\nContent-Length: "+to_string(j.size())+"\r\n\r\n"+j;
            send(cl,rsp.c_str(),rsp.size(),0);
        } else if (req.find("GET /video") != string::npos) {
            Mat f; { lock_guard<mutex> lk(frameMutex); if(!globalFrame.empty()) f=globalFrame.clone(); }
            if(!f.empty()) {
                vector<uchar> jpg; imencode(".jpg",f,jpg,{IMWRITE_JPEG_QUALITY,50});
                string hdr = "HTTP/1.1 200 OK\r\nContent-Type: image/jpeg\r\nContent-Length: "+to_string(jpg.size())+"\r\n\r\n";
                send(cl,hdr.c_str(),hdr.size(),0); send(cl,(char*)jpg.data(),jpg.size(),0);
            }
        }
        close(cl);
    }
    close(fd);
}

int main() {
    signal(SIGINT, sigintHandler);
    
    ofstream logFile("marker_log.json");
    logFile << "[" << endl;
    bool firstEntry = true;
    ostringstream logBuffer;
    auto lastFlush = steady_clock::now();
    
    Mat camMat = (Mat_<double>(3,3) << 393.621, 0, 315.072, 0, 403.996, 241.373, 0, 0, 1);
    Mat dist = (Mat_<double>(5,1) << 0.413208, -0.00684427, 0, 0, 0);
    float mSize = 0.10f;
    
    YawFilter   angF_yaw(0.5);
    RollFilter  angF_roll(0.5);
    PitchFilter angF_pitch(0.5);
    PosFilter posF_x(0.5), posF_y(0.5), posF_z(0.5);
    
    double lastX=0, lastY=0, lastZ=0, lastYaw=0, lastRoll=0, lastPitch=0;
    int lastId = -1;
    
    cout << "\033[2J\033[1;1H";
    cout << "ArUco + HTTP + LOG" << endl;
    cout << "============================================================" << endl;
    
    VideoCapture cap(0);
    cap.set(CAP_PROP_FOURCC, VideoWriter::fourcc('M','J','P','G'));
    cap.set(CAP_PROP_FRAME_WIDTH, 640); cap.set(CAP_PROP_FRAME_HEIGHT, 480);
    if (!cap.isOpened()) { cerr << "Camera error" << endl; return 1; }
    
    thread http(httpServer, 8080);
    
    auto dict = aruco::getPredefinedDictionary(aruco::DICT_4X4_50);
    auto params = aruco::DetectorParameters::create();
    params->cornerRefinementMethod = aruco::CORNER_REFINE_SUBPIX;
    params->cornerRefinementWinSize = 3;
    
    Mat frame, gray, proc;
    vector<int> ids, lastIds;
    vector<vector<Point2f>> corners, lastCorners;
    vector<Vec3d> rvecs, tvecs, lastRvecs, lastTvecs;
    
    int fc=0, noMarker=0;
    double fps=0;
    auto t0=steady_clock::now(), tPrint=steady_clock::now();
    int fpsCnt=0;
    auto firstFrameTime = steady_clock::time_point();
    bool firstFrame = true;
    
    while (running) {
        cap >> frame; if (frame.empty()) break;
        Mat disp = frame.clone();
        
        auto frameTime = steady_clock::now();
        if (firstFrame) { firstFrameTime = frameTime; firstFrame = false; }
        double t = duration<double>(frameTime - firstFrameTime).count();
        
        cvtColor(frame, gray, COLOR_BGR2GRAY);
        GaussianBlur(gray, proc, Size(3,3), 0);
        adaptiveThreshold(proc, proc, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, 7, 2);
        
        aruco::detectMarkers(proc, dict, corners, ids, params);
        if (ids.empty()) aruco::detectMarkers(gray, dict, corners, ids, params);
        
        if (!ids.empty()) {
            aruco::estimatePoseSingleMarkers(corners, mSize, camMat, dist, rvecs, tvecs);
            lastIds=ids; lastCorners=corners; lastRvecs=rvecs; lastTvecs=tvecs; noMarker=0;
        } else noMarker++;
        
        if (noMarker < 10) { ids=lastIds; corners=lastCorners; rvecs=lastRvecs; tvecs=lastTvecs; }
        else ids.clear();
        
        stringstream js, term;
        js << "{\"markers\":[";
        
        if (!ids.empty()) {
            for (size_t i=0; i<ids.size(); i++) {
                Vec3d ang = rvecToEuler(rvecs[i]);
                double fy = angF_yaw.filter(ang[0]);
                double fr = angF_roll.filter(ang[1]);
                double fp = angF_pitch.filter(ang[2]);
                
                Vec3d cPos = compensateTilt(tvecs[i], Vec3d(fy, fr, fp));
                double fx = posF_x.filter(cPos[0]);
                double fyy = posF_y.filter(cPos[1]);
                double fz = posF_z.filter(cPos[2]);
                
                lastX=fx; lastY=fyy; lastZ=fz;
                lastYaw=fy; lastRoll=fr; lastPitch=fp;
                lastId = ids[i];
                
                if (i>0) { js << ","; term << " | "; }
                js << "{\"id\":"<<ids[i]<<",\"x\":"<<fx<<",\"y\":"<<fyy<<",\"z\":"<<fz
                   <<",\"yaw\":"<<fy<<",\"roll\":"<<fr<<",\"pitch\":"<<fp<<"}";
                term << "ID:"<<ids[i]<<"|x:"<<fixed<<setprecision(2)<<fx<<" y:"<<fyy<<" z:"<<fz
                     <<"m yaw:"<<setprecision(0)<<fy<<"° roll:"<<fr<<"° pitch:"<<fp<<"°";
            }
            
            // Запись в буфер
            if (!firstEntry) logBuffer << "," << endl;
            firstEntry = false;
            logBuffer << "  {\"t\":" << fixed << setprecision(3) << t
                      << ",\"frame\":" << fc
                      << ",\"id\":" << lastId
                      << ",\"x\":" << lastX
                      << ",\"y\":" << lastY
                      << ",\"z\":" << lastZ
                      << ",\"yaw\":" << lastYaw
                      << ",\"roll\":" << lastRoll
                      << ",\"pitch\":" << lastPitch
                      << ",\"fps\":" << fps << "}";
            
            aruco::drawDetectedMarkers(disp, corners, ids);
            aruco::drawAxis(disp, camMat, dist, rvecs[0], tvecs[0], 0.05);
            
        } else if (lastId >= 0) {
            js << "{\"id\":"<<lastId<<",\"x\":"<<lastX<<",\"y\":"<<lastY<<",\"z\":"<<lastZ
               <<",\"yaw\":"<<lastYaw<<",\"roll\":"<<lastRoll<<",\"pitch\":"<<lastPitch<<"}";
            term << "ID:"<<lastId<<"|x:"<<fixed<<setprecision(2)<<lastX<<" y:"<<lastY<<" z:"<<lastZ
                 <<"m yaw:"<<setprecision(0)<<lastYaw<<"° roll:"<<lastRoll<<"° pitch:"<<lastPitch<<"° (hold)";
            
            if (!firstEntry) logBuffer << "," << endl;
            firstEntry = false;
            logBuffer << "  {\"t\":" << fixed << setprecision(3) << t
                      << ",\"frame\":" << fc
                      << ",\"id\":" << lastId
                      << ",\"x\":" << lastX
                      << ",\"y\":" << lastY
                      << ",\"z\":" << lastZ
                      << ",\"yaw\":" << lastYaw
                      << ",\"roll\":" << lastRoll
                      << ",\"pitch\":" << lastPitch
                      << ",\"fps\":" << fps << "}";
        }
        js << "]}";
        
        { lock_guard<mutex> lk(jsonMutex); globalJson = js.str(); }
        { lock_guard<mutex> lk(frameMutex); globalFrame = disp.clone(); }
        
        // Сброс буфера в файл раз в секунду
        if (duration<double>(steady_clock::now() - lastFlush).count() >= 1.0) {
            logFile << logBuffer.str();
            logFile.flush();
            logBuffer.str("");
            lastFlush = steady_clock::now();
        }
        
        fpsCnt++;
        auto now = steady_clock::now();
        if (duration<double>(now-t0).count() >= 1.0) {
            fps = fpsCnt / duration<double>(now-t0).count();
            fpsCnt=0; t0=now;
        }
        
        if (duration<double>(now-tPrint).count() >= 0.5) {
            cout << "\r[FPS:" << fixed << setprecision(0) << fps << "] ";
            if (!ids.empty() || lastId >= 0) {
                cout << "1 marker(s): " << term.str() << "   " << flush;
            } else {
                cout << "Searching...   " << flush;
            }
            tPrint = now;
        }
        fc++;
    }
    
    // Дописать остатки буфера
    logFile << logBuffer.str();
    logFile << endl << "]" << endl;
    logFile.close();
    cout << "\nLog saved: marker_log.json" << endl;
    
    cap.release(); http.join();
    cout << "\n\nSTOPPED" << endl;
    return 0;
}