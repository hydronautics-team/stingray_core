#pragma once

#include "hydrolib_fixed_point.hpp"

namespace hydrolib::math
{
template <ArithmeticConcept Number>
struct Vector3D
{
public:
    Number Dot(Vector3D &other) const;
    Vector3D Cross(Vector3D &other) const;
    Number Length() const;
    void Normalize();

    Vector3D &operator+=(Vector3D other);
    Vector3D operator+(Vector3D other) const;
    Vector3D operator-(Vector3D other) const;
    Vector3D &operator-=(Vector3D other);
    Vector3D operator-() const;

    Vector3D operator*(const Number &scalar) const;
    Vector3D &operator*=(const Number &scalar);
    Vector3D operator/(const Number &scalar) const;
    Vector3D &operator/=(const Number &scalar);

public:
    Number x;
    Number y;
    Number z;
};

template <ArithmeticConcept Number>
inline Number Vector3D<Number>::Dot(Vector3D<Number> &other) const
{
    return (x * other.x) + (y * other.y) + (z * other.z);
}

template <ArithmeticConcept Number>
inline Vector3D<Number> Vector3D<Number>::Cross(Vector3D<Number> &other) const
{
    return Vector3D<Number>(y * other.z - z * other.y,
                            z * other.x - x * other.z,
                            x * other.y - y * other.x);
}

template <ArithmeticConcept Number>
inline Number Vector3D<Number>::Length() const
{
    return sqrt(x * x + y * y + z * z);
}

template <ArithmeticConcept Number>
inline void Vector3D<Number>::Normalize()
{
    Number len = Length();
    if (len == 0)
    {
        x = 0;
        y = 0;
        z = 0;
    }
    else
    {
        x /= len;
        y /= len;
        z /= len;
    }
}

template <ArithmeticConcept Number>
inline Vector3D<Number> &Vector3D<Number>::operator+=(Vector3D<Number> other)
{
    x += other.x;
    y += other.y;
    z += other.z;
    return *this;
}

template <ArithmeticConcept Number>
inline Vector3D<Number>
Vector3D<Number>::operator+(Vector3D<Number> other) const
{
    auto result = *this;
    result += other;
    return result;
}

template <ArithmeticConcept Number>
inline Vector3D<Number> Vector3D<Number>::operator-() const
{
    return Vector3D<Number>(-x, -y, -z);
}

template <ArithmeticConcept Number>
inline Vector3D<Number> &Vector3D<Number>::operator-=(Vector3D<Number> other)
{
    *this += -other;
    return *this;
}

template <ArithmeticConcept Number>
inline Vector3D<Number>
Vector3D<Number>::operator-(Vector3D<Number> other) const
{
    auto result = *this;
    result -= other;
    return result;
}

template <ArithmeticConcept Number>
inline Vector3D<Number> Vector3D<Number>::operator*(const Number &scalar) const
{
    return Vector3D<Number>(x * scalar, y * scalar, z * scalar);
}

template <ArithmeticConcept Number>
inline Vector3D<Number> &Vector3D<Number>::operator*=(const Number &scalar)
{
    x *= scalar;
    y *= scalar;
    z *= scalar;
    return *this;
}

template <ArithmeticConcept Number>
inline Vector3D<Number> Vector3D<Number>::operator/(const Number &scalar) const
{
    return Vector3D<Number>(x / scalar, y / scalar, z / scalar);
}

template <ArithmeticConcept Number>
inline Vector3D<Number> &Vector3D<Number>::operator/=(const Number &scalar)
{
    x /= scalar;
    y /= scalar;
    z /= scalar;
    return *this;
}
} // namespace hydrolib::math