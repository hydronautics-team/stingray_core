#pragma once

#include "hydrolib_fixed_point.hpp"
#include "hydrolib_quaternions.hpp"
#include "hydrolib_vector3d.hpp"

namespace hydrolib::math
{
template <ArithmeticConcept Number>
Quaternion<Number> GetRotation(Vector3D<Number> from, Vector3D<Number> to);

template <ArithmeticConcept Number>
Vector3D<Number> Rotate(Vector3D<Number> source, Quaternion<Number> rotation);

template <ArithmeticConcept Number>
Quaternion<Number> ExtractZRotation(Quaternion<Number> &quaternion);

template <ArithmeticConcept Number>
Quaternion<Number> GetMean(const Quaternion<Number> &q1,
                           const Quaternion<Number> &q2);

/////////////////////////////////////////////////////////////////////////

template <ArithmeticConcept Number>
inline Quaternion<Number> GetRotation(Vector3D<Number> from,
                                      Vector3D<Number> to)
{
    from.Normalize();
    to.Normalize();
    Number w = from.Dot(to) + 1;
    if (w != 0)
    {
        Quaternion<Number> result = Quaternion<Number>(from.Cross(to), w);
        result.Normalize();
        return result;
    }
    else
    {
        if (from.y == 0 && from.x == 0)
        {
            return {1, 0, 0, 0};
        }
        Quaternion<Number> result = Quaternion<Number>(from.y, -from.x, 0, 0);
        result.Normalize();
        return result;
    }
}

template <ArithmeticConcept Number>
inline Vector3D<Number> Rotate(Vector3D<Number> source,
                               Quaternion<Number> rotation)
{
    auto result = rotation * Quaternion<Number>(source) * (!rotation);
    return {.x = result.x, .y = result.y, .z = result.z};
}

template <ArithmeticConcept Number>
inline Quaternion<Number> ExtractZRotation(Quaternion<Number> &quaternion)
{
    Number new_w =
        sqrt(quaternion.w * quaternion.w + quaternion.z * quaternion.z);
    Number sin_yaw = quaternion.z / new_w;
    Number cos_yaw = quaternion.w / new_w;
    Number new_x = quaternion.x * cos_yaw + quaternion.y * sin_yaw;
    Number new_y = quaternion.y * cos_yaw - quaternion.x * sin_yaw;
    quaternion.x = new_x;
    quaternion.y = new_y;
    quaternion.z = 0;
    quaternion.w = new_w;
    return Quaternion<Number>(0, 0, sin_yaw, cos_yaw);
}

template <ArithmeticConcept Number>
inline Quaternion<Number> GetMean(const Quaternion<Number> &q1,
                                  const Quaternion<Number> &q2)
{
    if (q1.Dot(q2) < 0)
    {
        auto result = (q1 - q2) / 2;
        result.Normalize();
        return result;
    }
    else
    {
        auto result = (q1 + q2) / 2;
        result.Normalize();
        return result;
    }
}
} // namespace hydrolib::math
