#pragma once

#include "hydrolib_fixed_point.hpp"
#include "hydrolib_quaternions.hpp"
#include "hydrolib_rotations.hpp"
#include "hydrolib_vector3d.hpp"

namespace hydrolib::sensors
{

template <math::ArithmeticConcept Number, double PERIOD_S>
class IMUProcessor
{
public:
    constexpr IMUProcessor();

public:
    math::Quaternion<Number> Process(math::Vector3D<Number> accel_g,
                                     math::Vector3D<Number> gyro_deg_per_s);
    void Calibrate(math::Vector3D<Number> g_z,
                   math::Vector3D<Number> g_opposite_z,
                   math::Vector3D<Number> g_x);

private:
    math::Quaternion<Number> orientation_;
    math::Vector3D<Number> accel_absolute_error_;
    math::Quaternion<Number> orientation_compensation_;
};

//=========================================================

template <math::ArithmeticConcept Number, double PERIOD_S>
constexpr IMUProcessor<Number, PERIOD_S>::IMUProcessor()
    : orientation_(0, 0, 0, 1),
      accel_absolute_error_(0, 0, 0),
      orientation_compensation_(0, 0, 0, 1)
{
}

template <math::ArithmeticConcept Number, double PERIOD_S>
math::Quaternion<Number>
IMUProcessor<Number, PERIOD_S>::Process(math::Vector3D<Number> accel_g,
                                        math::Vector3D<Number> gyro_rad_per_s)
{
    accel_g = accel_g - accel_absolute_error_;
    accel_g = math::Rotate(accel_g, orientation_compensation_);
    Number w_rad_per_s = gyro_rad_per_s.Length();
    Number fi_rad = w_rad_per_s * Number(PERIOD_S);
    gyro_rad_per_s.Normalize();
    math::Quaternion<Number> gyro_rotation = math::Quaternion<Number>(
        gyro_rad_per_s * sin(fi_rad / 2), cos(fi_rad / 2));
    math::Quaternion<Number> model_orientation = orientation_ * gyro_rotation;
    math::Vector3D<Number> g_model =
        math::Rotate({0, 0, -1}, model_orientation);
    math::Quaternion<Number> model_orientation_xy =
        math::GetRotation({0, 0, -1}, g_model);
    math::Quaternion<Number> model_orientation_z =
        model_orientation * (!model_orientation_xy);
    math::Quaternion<Number> sensor_orientation =
        math::GetRotation({0, 0, -1}, accel_g);
    math::Quaternion<Number> completed_sensor_orientation =
        model_orientation_z * sensor_orientation;
    completed_sensor_orientation.Normalize();
    // math::Quaternion<Number> mean_orientation =
    //     math::GetMean(model_orientation, completed_sensor_orientation);
    orientation_ = completed_sensor_orientation;
    return orientation_;
}

template <math::ArithmeticConcept Number, double PERIOD_S>
void IMUProcessor<Number, PERIOD_S>::Calibrate(
    math::Vector3D<Number> g_z, math::Vector3D<Number> g_opposite_z,
    math::Vector3D<Number> g_x)
{
    accel_absolute_error_ = (g_z + g_opposite_z) / 2;
    math::Vector3D<Number> g_z_compensated = g_z - accel_absolute_error_;
    math::Vector3D<Number> g_x_compensated = g_x - accel_absolute_error_;
    math::Quaternion<Number> first_rotation =
        math::GetRotation(g_z_compensated, {0, 0, 1});
    math::Vector3D<Number> g_x_rotated =
        math::Rotate(g_x_compensated, first_rotation);
    math::Quaternion<Number> second_rotation =
        math::GetRotation(g_x_rotated, {1, 0, 0});
    orientation_compensation_ = second_rotation * first_rotation;
}
} // namespace hydrolib::sensors
