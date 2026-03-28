#include "hydrolib_pid.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <iostream>
#include <numbers>

using namespace hydrolib;

TEST(TestPID, HarmonicTest)
{
    constexpr unsigned freq_hz = 100;
    constexpr unsigned test_time_s = 5;

    constexpr unsigned control_amplitude = 1000;
    constexpr unsigned control_freq_hz = 1;

    constexpr unsigned p = 1;
    constexpr unsigned i = 100;
    constexpr unsigned divide_shift = 12;

    constexpr double freq_rad = control_freq_hz * 2 * std::numbers::pi;

    controlling::PID<freq_hz> pid;
    pid.SetP(p << divide_shift);
    pid.SetI(i << divide_shift);
    pid.SetDivideShift(divide_shift);

    for (unsigned t = 0; t < freq_hz * test_time_s; t++)
    {
        double sin_value =
            std::sin(static_cast<double>(t) * freq_rad / freq_hz);
        double cos_value =
            std::cos(static_cast<double>(t) * freq_rad / freq_hz);
        double control = control_amplitude * sin_value;
        double target_output = p * control_amplitude * sin_value -
                               i * control_amplitude * cos_value / freq_rad +
                               i * control_amplitude / freq_rad;
        int real_output = pid.Process(control);

        // std::cout << target_output << " : " << real_output << "\n";

        EXPECT_LT(std::abs(target_output - real_output),
                  control_amplitude * 0.05);
    }
}

TEST(TestPID, ControlTest)
{
    constexpr unsigned freq_hz = 100;
    constexpr unsigned test_time_s = 5;

    constexpr unsigned control = 1000;
    constexpr double tau = 1;

    constexpr unsigned p = 100;
    constexpr unsigned i = 200;
    constexpr unsigned divide_shift = 10;

    controlling::PID<freq_hz> pid;
    pid.SetP(p << divide_shift);
    pid.SetI(i << divide_shift);
    pid.SetDivideShift(divide_shift);

    double controlled_value = 0;
    double last_control = 0;

    for (unsigned t = 0; t < freq_hz * test_time_s; t++)
    {
        double current_control = pid.Process(control - controlled_value);
        controlled_value = ((last_control + current_control) / freq_hz -
                            (1 - 2 * tau) * controlled_value) /
                           (2 * tau);

        last_control = current_control;

        // std::cout << controlled_value << " : 0\n";
    }

    EXPECT_LT(std::abs(controlled_value - control), control * 0.01);
}
