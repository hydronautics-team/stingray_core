#pragma once

#include <concepts>

namespace hydrolib::sensors
{

struct PressureSensorData
{
    int depth_mm;
    int depth_rate_mm_per_s;
};

template <typename T>
concept PressureSensorConcept = requires(T sensor) {
    { sensor.GetPressureData() } -> std::same_as<PressureSensorData>;
};

} // namespace hydrolib::sensors