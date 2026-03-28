#pragma once

#include "hydrolib_vector3d.hpp"

namespace hydrolib::math
{
template <ArithmeticConcept Number>
class Quaternion
{
public:
    constexpr Quaternion(Number x, Number y, Number z, Number w);
    constexpr Quaternion(Vector3D<Number> &vector);
    constexpr Quaternion(Vector3D<Number> &&vector, Number w);

public:
    Quaternion &operator+=(const Quaternion &other);
    Quaternion operator-() const;
    Quaternion operator+(const Quaternion &other) const;
    Quaternion operator-(const Quaternion &other) const;
    Quaternion operator*(const Quaternion &other) const;
    Quaternion operator!() const;

    Quaternion operator*(Number k) const;
    Quaternion operator/(Number k) const;

    Number Dot(const Quaternion &other) const;

    Number GetNorm() const;
    void Normalize();

public:
    Number x;
    Number y;
    Number z;
    Number w;
};

template <ArithmeticConcept Number>
constexpr Quaternion<Number>::Quaternion(Number x, Number y, Number z, Number w)
    : x(x), y(y), z(z), w(w)
{
}

template <ArithmeticConcept Number>
constexpr Quaternion<Number>::Quaternion(Vector3D<Number> &vector)
    : x(vector.x), y(vector.y), z(vector.z), w(0)
{
}

template <ArithmeticConcept Number>
constexpr Quaternion<Number>::Quaternion(Vector3D<Number> &&vector, Number w)
    : x(vector.x), y(vector.y), z(vector.z), w(w)
{
}

template <ArithmeticConcept Number>
inline Quaternion<Number> &
Quaternion<Number>::operator+=(const Quaternion<Number> &other)
{
    x += other.x;
    y += other.y;
    z += other.z;
    w += other.w;
    return *this;
}

template <ArithmeticConcept Number>
inline Quaternion<Number> Quaternion<Number>::operator-() const
{
    return Quaternion{-x, -y, -z, -w};
}

template <ArithmeticConcept Number>
inline Quaternion<Number>
Quaternion<Number>::operator+(const Quaternion<Number> &other) const
{
    Quaternion result = *this;
    result += other;
    return result;
}

template <ArithmeticConcept Number>
inline Quaternion<Number>
Quaternion<Number>::operator-(const Quaternion<Number> &other) const
{
    Quaternion result = *this;
    result += -other;
    return result;
}

template <ArithmeticConcept Number>
inline Quaternion<Number>
Quaternion<Number>::operator*(const Quaternion<Number> &other) const
{
    return Quaternion(x * other.w + w * other.x + y * other.z - z * other.y,
                      z * other.x + y * other.w + w * other.y - x * other.z,
                      w * other.z + x * other.y - y * other.x + z * other.w,
                      w * other.w - y * other.y - z * other.z - x * other.x);
}

template <ArithmeticConcept Number>
inline Quaternion<Number> Quaternion<Number>::operator!() const
{
    return Quaternion(-x, -y, -z, w);
}

template <ArithmeticConcept Number>
inline Number Quaternion<Number>::GetNorm() const
{
    return sqrt(x * x + y * y + z * z + w * w);
}

template <ArithmeticConcept Number>
inline Quaternion<Number> Quaternion<Number>::operator*(Number k) const
{
    return Quaternion(x * k, y * k, z * k, w * k);
}

template <ArithmeticConcept Number>
inline Quaternion<Number> Quaternion<Number>::operator/(Number k) const
{
    return Quaternion(x / k, y / k, z / k, w / k);
}

template <ArithmeticConcept Number>
inline Number Quaternion<Number>::Dot(const Quaternion<Number> &other) const
{
    return x * other.x + y * other.y + z * other.z + w * other.w;
}

template <ArithmeticConcept Number>
inline void Quaternion<Number>::Normalize()
{
    Number norm = GetNorm();
    if (norm != 0)
    {
        x /= norm;
        y /= norm;
        z /= norm;
        w /= norm;
    }
}

} // namespace hydrolib::math
