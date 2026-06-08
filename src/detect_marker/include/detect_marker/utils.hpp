#pragma once
#include <opencv2/opencv.hpp>
#include <string>
#include <fstream>

inline bool loadCalibration(const std::string& fn, cv::Mat& cam, cv::Mat& dist, int& w, int& h) {
    std::ifstream f(fn);
    if (!f.is_open()) return false;
    std::string k;
    double fx = 300, fy = 300, cx = 320, cy = 240;
    double k1 = 0, k2 = 0, p1 = 0, p2 = 0, k3 = 0;
    while (f >> k) {
        if (k.find("\"fx\"") != std::string::npos) { f >> k; fx = std::stod(k); }
        if (k.find("\"fy\"") != std::string::npos) { f >> k; fy = std::stod(k); }
        if (k.find("\"cx\"") != std::string::npos) { f >> k; cx = std::stod(k); }
        if (k.find("\"cy\"") != std::string::npos) { f >> k; cy = std::stod(k); }
        if (k.find("\"k1\"") != std::string::npos) { f >> k; k1 = std::stod(k); }
        if (k.find("\"k2\"") != std::string::npos) { f >> k; k2 = std::stod(k); }
        if (k.find("\"p1\"") != std::string::npos) { f >> k; p1 = std::stod(k); }
        if (k.find("\"p2\"") != std::string::npos) { f >> k; p2 = std::stod(k); }
        if (k.find("\"k3\"") != std::string::npos) { f >> k; k3 = std::stod(k); }
    }
    f.close();
    cam = (cv::Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
    dist = (cv::Mat_<double>(5, 1) << k1, k2, p1, p2, k3);
    return true;
}