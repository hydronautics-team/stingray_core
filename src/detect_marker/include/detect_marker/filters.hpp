#pragma once

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
};