#pragma once
#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <thread>
#include <mutex>
#include <vector>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>

extern volatile bool running;
extern cv::Mat globalFrame;
extern std::mutex frameMutex;
extern std::string globalJson;
extern std::mutex jsonMutex;

inline void httpServer(int port) {
    int fd = socket(AF_INET, SOCK_STREAM, 0), opt = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(port);
    bind(fd, (sockaddr*)&addr, sizeof(addr));
    listen(fd, 3);
    std::cout << "HTTP: http://0.0.0.0:" << port << "/stream | /data | /video" << std::endl;

    while (running) {
        int cl = accept(fd, nullptr, nullptr);
        if (cl < 0) continue;
        char buf[2048]{};
        read(cl, buf, 2047);
        std::string req(buf);

        if (req.find("GET /stream") != std::string::npos) {
            std::string hdr = "HTTP/1.1 200 OK\r\nContent-Type: multipart/x-mixed-replace; boundary=frame\r\nAccess-Control-Allow-Origin: *\r\n\r\n";
            send(cl, hdr.c_str(), hdr.size(), 0);
            while (running) {
                cv::Mat f;
                {
                    std::lock_guard<std::mutex> lk(frameMutex);
                    if (!globalFrame.empty()) f = globalFrame.clone();
                }
                if (!f.empty()) {
                    std::vector<uchar> jpg;
                    cv::imencode(".jpg", f, jpg, {cv::IMWRITE_JPEG_QUALITY, 50});
                    std::string b = "--frame\r\nContent-Type: image/jpeg\r\n\r\n";
                    send(cl, b.c_str(), b.size(), 0);
                    send(cl, (char*)jpg.data(), jpg.size(), 0);
                    send(cl, "\r\n", 2, 0);
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(33));
            }
        } else if (req.find("GET /data") != std::string::npos) {
            std::string j;
            {
                std::lock_guard<std::mutex> lk(jsonMutex);
                j = globalJson;
            }
            std::string rsp = "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nAccess-Control-Allow-Origin: *\r\nContent-Length: " + std::to_string(j.size()) + "\r\n\r\n" + j;
            send(cl, rsp.c_str(), rsp.size(), 0);
        } else if (req.find("GET /video") != std::string::npos) {
            cv::Mat f;
            {
                std::lock_guard<std::mutex> lk(frameMutex);
                if (!globalFrame.empty()) f = globalFrame.clone();
            }
            if (!f.empty()) {
                std::vector<uchar> jpg;
                cv::imencode(".jpg", f, jpg, {cv::IMWRITE_JPEG_QUALITY, 50});
                std::string hdr = "HTTP/1.1 200 OK\r\nContent-Type: image/jpeg\r\nAccess-Control-Allow-Origin: *\r\nContent-Length: " + std::to_string(jpg.size()) + "\r\n\r\n";
                send(cl, hdr.c_str(), hdr.size(), 0);
                send(cl, (char*)jpg.data(), jpg.size(), 0);
            }
        }
        close(cl);
    }
    close(fd);
}