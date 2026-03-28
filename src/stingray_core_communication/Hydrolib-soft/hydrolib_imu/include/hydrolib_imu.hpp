#pragma once

#include <concepts>

namespace hydrolib::sensors
{

struct IMUData
{
    int yaw_mdeg;
    int pitch_mdeg;
    int roll_mdeg;

    int yaw_rate_mdeg_per_s;
    int pitch_rate_mdeg_per_s;
    int roll_rate_mdeg_per_s;
};

template <typename T>
concept IMUConcept = requires(T imu) {
    { imu.GetIMUData() } -> std::same_as<IMUData>;
};

} // namespace hydrolib::sensors