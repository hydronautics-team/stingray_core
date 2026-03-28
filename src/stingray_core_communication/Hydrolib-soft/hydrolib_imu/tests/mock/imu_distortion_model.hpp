#pragma once

#include "hydrolib_fixed_point.hpp"
#include "hydrolib_quaternions.hpp"
#include "hydrolib_rotations.hpp"
#include "hydrolib_vector3d.hpp"

namespace hydrolib::sensors
{

template <math::ArithmeticConcept Number>
class IMUDistorter
{
public:
    IMUDistorter(math::Quaternion<Number> rotation,
                 math::Vector3D<Number> error);

public:
    math::Vector3D<Number> DistortAccel(math::Vector3D<Number> accel);

private:
    math::Quaternion<Number> rotation_;
    math::Vector3D<Number> error_;
};

template <math::ArithmeticConcept Number>
IMUDistorter<Number>::IMUDistorter(math::Quaternion<Number> rotation,
                                   math::Vector3D<Number> error)
    : rotation_(rotation), error_(error)
{
}

template <math::ArithmeticConcept Number>
math::Vector3D<Number>
IMUDistorter<Number>::DistortAccel(math::Vector3D<Number> accel)
{
    return math::Rotate(accel, rotation_) + error_;
}
} // namespace hydrolib::sensors