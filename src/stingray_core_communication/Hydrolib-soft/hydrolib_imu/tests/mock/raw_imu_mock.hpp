#pragma once

#include "hydrolib_fixed_point.hpp"
#include "hydrolib_quaternions.hpp"
#include "hydrolib_rotations.hpp"
#include "hydrolib_vector3d.hpp"

namespace hydrolib::sensors
{

template <math::ArithmeticConcept Number>
class RawIMUMock
{
public:
    RawIMUMock();

public:
    void SetTarget(math::Vector3D<Number> axis, Number angle_rad, int n);

    bool Step();

    math::Vector3D<Number> GetAcceleration() const;
    math::Vector3D<Number> GetGyro() const;
    math::Quaternion<Number> GetOrientation() const;

private:
    int counter_;
    math::Quaternion<Number> orientation_;
    math::Quaternion<Number> delta_orientation_;
    math::Vector3D<Number> w_;
};

template <math::ArithmeticConcept Number>
inline RawIMUMock<Number>::RawIMUMock()
    : counter_(0), orientation_(0, 0, 0, 1), delta_orientation_(0, 0, 0, 1)
{
}

template <math::ArithmeticConcept Number>
inline void RawIMUMock<Number>::SetTarget(math::Vector3D<Number> axis,
                                          Number angle_rad, int n)
{
    auto delta_angle_rad = angle_rad / n;
    delta_orientation_ = {axis * sin(delta_angle_rad / 2),
                          cos(delta_angle_rad / 2)};
    w_ = axis * delta_angle_rad;
    counter_ = n;
}

template <math::ArithmeticConcept Number>
inline bool RawIMUMock<Number>::Step()
{
    if (counter_ == 0)
    {
        w_ = {0, 0, 0};
        return false;
    }
    counter_--;
    orientation_ = orientation_ * delta_orientation_;
    return true;
}

template <math::ArithmeticConcept Number>
inline math::Vector3D<Number> RawIMUMock<Number>::GetAcceleration() const
{
    math::Vector3D<Number> g(0, 0, -1);
    auto result = math::Rotate(g, orientation_);
    result.Normalize();
    return result;
}

template <math::ArithmeticConcept Number>
inline math::Vector3D<Number> RawIMUMock<Number>::GetGyro() const
{
    return w_;
}

template <math::ArithmeticConcept Number>
inline math::Quaternion<Number> RawIMUMock<Number>::GetOrientation() const
{
    return orientation_;
}

} // namespace hydrolib::sensors