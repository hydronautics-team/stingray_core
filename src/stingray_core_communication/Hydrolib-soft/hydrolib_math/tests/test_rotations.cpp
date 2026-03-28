#include "hydrolib_quaternions.hpp"
#include "hydrolib_rotations.hpp"
#include "hydrolib_vector3d.hpp"

#include <gtest/gtest.h>

using namespace hydrolib::math;

TEST(TestHydrolibMathRotations, Rotation)
{
    Vector3D<double> a{1, 1, 1};
    Vector3D<double> b{1, 2, 3};

    b.Normalize();
    b *= a.Length();

    auto q = GetRotation(a, b);
    auto result = Rotate(a, q);
    EXPECT_NEAR(b.x, result.x, 0.0001);
    EXPECT_NEAR(b.y, result.y, 0.0001);
    EXPECT_NEAR(b.z, result.z, 0.0001);
}

TEST(TestHydrolibMathRotations, RotationReversed)
{
    Vector3D<double> a{1, 1, 1};
    Quaternion<double> q{1, 2, 3, 4};

    q.Normalize();

    auto b = Rotate(a, q);
    auto result = GetRotation(a, b);
    std::cout << static_cast<double>(q.x) << " " << static_cast<double>(q.y)
              << " " << static_cast<double>(q.z) << " "
              << static_cast<double>(q.w) << std::endl;
    std::cout << static_cast<double>(result.x) << " "
              << static_cast<double>(result.y) << " "
              << static_cast<double>(result.z) << " "
              << static_cast<double>(result.w) << std::endl;
    auto c = Rotate(a, result);
    EXPECT_NEAR(c.x, b.x, 0.0001);
    EXPECT_NEAR(c.y, b.y, 0.0001);
    EXPECT_NEAR(c.z, b.z, 0.0001);
}

TEST(TestHydrolibMathRotations, RotationCornerCase)
{
    Vector3D<double> a{1, 0, 0};
    Vector3D<double> b{-1, 0, 0};

    b.Normalize();
    b *= a.Length();

    auto q = GetRotation(a, b);
    auto result = Rotate(a, q);
    EXPECT_NEAR(b.x, result.x, 0.0001);
    EXPECT_NEAR(b.y, result.y, 0.0001);
    EXPECT_NEAR(b.z, result.z, 0.0001);
}

TEST(TestHydrolibMathRotations, AxisRotation)
{
    Vector3D<double> x{1, 0, 0};
    Vector3D<double> x_{0, 0, 1};
    Vector3D<double> y{0, 1, 0};
    Vector3D<double> y_{0, 1, 0};
    Vector3D<double> z{0, 0, 1};
    Vector3D<double> z_{-1, 0, 0};

    auto q = GetRotation(x, x_);
    auto result_y = Rotate(y, q);
    auto result_z = Rotate(z, q);
    EXPECT_DOUBLE_EQ(y_.x, result_y.x);
    EXPECT_DOUBLE_EQ(y_.y, result_y.y);
    EXPECT_DOUBLE_EQ(y_.z, result_y.z);
    EXPECT_DOUBLE_EQ(z_.x, result_z.x);
    EXPECT_DOUBLE_EQ(z_.y, result_z.y);
    EXPECT_DOUBLE_EQ(z_.z, result_z.z);
}

TEST(TestHydrolibMath, QuaternionExtractZRotation)
{
    Quaternion<double> q_z(0.0, 0.0, 1.0, 1.0);
    q_z.Normalize();
    Quaternion<double> q_xy(1.0, 1.0, 0.0, 1.0);
    q_xy.Normalize();
    Quaternion<double> q = q_z * q_xy;

    Quaternion<double> result = ExtractZRotation(q);
    EXPECT_DOUBLE_EQ(result.x, q_z.x);
    EXPECT_DOUBLE_EQ(result.y, q_z.y);
    EXPECT_DOUBLE_EQ(result.z, q_z.z);
    EXPECT_DOUBLE_EQ(result.w, q_z.w);
    EXPECT_DOUBLE_EQ(q.x, q_xy.x);
    EXPECT_DOUBLE_EQ(q.y, q_xy.y);
    EXPECT_DOUBLE_EQ(q.z, q_xy.z);
    EXPECT_DOUBLE_EQ(q.w, q_xy.w);
}

TEST(TestHydrolibMath, QuaternionExtractZRotationReversed)
{
    Quaternion<double> q(1.0, 1.0, 1.0, 1.0);
    q.Normalize();

    Quaternion<double> q_xy = q;

    Quaternion<double> q_z = ExtractZRotation(q_xy);
    auto result = q_z * q_xy;
    EXPECT_DOUBLE_EQ(q.x, result.x);
    EXPECT_DOUBLE_EQ(q.y, result.y);
    EXPECT_DOUBLE_EQ(q.z, result.z);
    EXPECT_DOUBLE_EQ(q.w, result.w);
}
