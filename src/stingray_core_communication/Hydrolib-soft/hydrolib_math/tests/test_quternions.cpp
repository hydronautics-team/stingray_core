#include "hydrolib_quaternions.hpp"
#include "hydrolib_vector3d.hpp"

#include <gtest/gtest.h>

using namespace hydrolib::math;

TEST(TestHydrolibMath, QuaternionConstructorWithComponents)
{
    Quaternion<double> q(1.0, 2.0, 3.0, 4.0);

    EXPECT_EQ(q.x, 1.0);
    EXPECT_EQ(q.y, 2.0);
    EXPECT_EQ(q.z, 3.0);
    EXPECT_EQ(q.w, 4.0);
}

TEST(TestHydrolibMath, QuaternionConstructorWithVector3D)
{
    Vector3D<double> v{10.0, 20.0, 30.0};
    Quaternion<double> q(v);

    EXPECT_EQ(q.x, 10.0);
    EXPECT_EQ(q.y, 20.0);
    EXPECT_EQ(q.z, 30.0);
    EXPECT_EQ(q.w, 0.0);
}

TEST(TestHydrolibMath, QuaternionAddition)
{
    Quaternion<double> q1(double(1), double(2), double(3), double(4));
    Quaternion<double> q2(double(5), double(6), double(7), double(8));
    Quaternion<double> result = q1 + q2;

    EXPECT_EQ(result.x, double(6));
    EXPECT_EQ(result.y, double(8));
    EXPECT_EQ(result.z, double(10));
    EXPECT_EQ(result.w, double(12));
}

TEST(TestHydrolibMath, QuaternionAdditionAssignment)
{
    Quaternion<double> q1(double(1), double(2), double(3), double(4));
    Quaternion<double> q2(double(5), double(6), double(7), double(8));
    q1 += q2;

    EXPECT_EQ(q1.x, double(6));
    EXPECT_EQ(q1.y, double(8));
    EXPECT_EQ(q1.z, double(10));
    EXPECT_EQ(q1.w, double(12));
}

TEST(TestHydrolibMath, QuaternionSubtraction)
{
    Quaternion<double> q1(double(10), double(20), double(30), double(40));
    Quaternion<double> q2(double(5), double(6), double(7), double(8));
    Quaternion<double> result = q1 - q2;

    EXPECT_EQ(result.x, double(5));
    EXPECT_EQ(result.y, double(14));
    EXPECT_EQ(result.z, double(23));
    EXPECT_EQ(result.w, double(32));
}

TEST(TestHydrolibMath, QuaternionUnaryMinus)
{
    Quaternion<double> q(double(1), double(-2), double(3), double(-4));
    Quaternion<double> result = -q;

    EXPECT_EQ(result.x, double(-1));
    EXPECT_EQ(result.y, double(2));
    EXPECT_EQ(result.z, double(-3));
    EXPECT_EQ(result.w, double(4));
}

TEST(TestHydrolibMath, QuaternionMultiplication)
{
    Quaternion<double> q1(double(1), double(0), double(0), double(0));
    Quaternion<double> q2(double(0), double(1), double(0), double(0));
    Quaternion<double> result = q1 * q2;

    EXPECT_EQ(result.x, double(0));
    EXPECT_EQ(result.y, double(0));
    EXPECT_EQ(result.z, double(1));
    EXPECT_EQ(result.w, double(0));
}

TEST(TestHydrolibMath, QuaternionMultiplicationWithIdentity)
{
    Quaternion<double> q(double(1), double(2), double(3), double(0));
    Quaternion<double> identity(double(0), double(0), double(0), double(1));
    Quaternion<double> result = q * identity;

    EXPECT_EQ(result.x, double(1));
    EXPECT_EQ(result.y, double(2));
    EXPECT_EQ(result.z, double(3));
    EXPECT_EQ(result.w, double(0));
}

TEST(TestHydrolibMath, QuaternionConjugate)
{
    Quaternion<double> q(double(1), double(2), double(3), double(4));
    Quaternion<double> conjugate = !q;

    EXPECT_EQ(conjugate.x, double(-1));
    EXPECT_EQ(conjugate.y, double(-2));
    EXPECT_EQ(conjugate.z, double(-3));
    EXPECT_EQ(conjugate.w, double(4));
}

TEST(TestHydrolibMath, QuaternionScalarMultiplication)
{
    Quaternion<double> q(double(2), double(4), double(6), double(8));
    Quaternion<double> result = q * double(3);

    EXPECT_EQ(result.x, double(6));
    EXPECT_EQ(result.y, double(12));
    EXPECT_EQ(result.z, double(18));
    EXPECT_EQ(result.w, double(24));
}

TEST(TestHydrolibMath, QuaternionScalarDivision)
{
    Quaternion<double> q(double(6), double(12), double(18), double(24));
    Quaternion<double> result = q / double(3);

    EXPECT_EQ(result.x, double(2));
    EXPECT_EQ(result.y, double(4));
    EXPECT_EQ(result.z, double(6));
    EXPECT_EQ(result.w, double(8));
}

TEST(TestHydrolibMath, QuaternionNorm)
{
    Quaternion<double> q(double(3), double(4), double(0), double(0));
    double norm = q.GetNorm();

    EXPECT_EQ(norm, double(5));
}

TEST(TestHydrolibMath, QuaternionNormZero)
{
    Quaternion<double> q(double(0), double(0), double(0), double(0));
    double norm = q.GetNorm();

    EXPECT_EQ(norm, double(0));
}

TEST(TestHydrolibMath, QuaternionNormalize)
{
    Quaternion<double> q(double(3), double(4), double(0), double(0));
    q.Normalize();

    auto norm = q.GetNorm();
    EXPECT_EQ(norm, double(1));
}

TEST(TestHydrolibMath, QuaternionChainOperations)
{
    Quaternion<double> q1(double(1), double(2), double(3), double(4));
    Quaternion<double> q2(double(2), double(3), double(4), double(5));
    Quaternion<double> q3(double(1), double(1), double(1), double(1));

    Quaternion<double> result = (q1 + q2) - q3;

    EXPECT_EQ(result.x, double(2));
    EXPECT_EQ(result.y, double(4));
    EXPECT_EQ(result.z, double(6));
    EXPECT_EQ(result.w, double(8));
}

TEST(TestHydrolibMath, QuaternionNegativeComponents)
{
    Quaternion<double> q(double(-1), double(-2), double(-3), double(-4));

    EXPECT_EQ(q.x, double(-1));
    EXPECT_EQ(q.y, double(-2));
    EXPECT_EQ(q.z, double(-3));
    EXPECT_EQ(q.w, double(-4));

    Quaternion<double> positive = -q;
    EXPECT_EQ(positive.x, double(1));
    EXPECT_EQ(positive.y, double(2));
    EXPECT_EQ(positive.z, double(3));
    EXPECT_EQ(positive.w, double(4));
}

TEST(TestHydrolibMath, QuaternionScalarOperationsWithZero)
{
    Quaternion<double> q(double(5), double(10), double(15), double(20));
    Quaternion<double> result_mult = q * double(0);

    EXPECT_EQ(result_mult.x, double(0));
    EXPECT_EQ(result_mult.y, double(0));
    EXPECT_EQ(result_mult.z, double(0));
    EXPECT_EQ(result_mult.w, double(0));
}

TEST(TestHydrolibMath, QuaternionScalarOperationsWithOne)
{
    Quaternion<double> q(double(5), double(10), double(15), double(20));
    Quaternion<double> result_mult = q * double(1);
    Quaternion<double> result_div = q / double(1);

    EXPECT_EQ(result_mult.x, double(5));
    EXPECT_EQ(result_mult.y, double(10));
    EXPECT_EQ(result_mult.z, double(15));
    EXPECT_EQ(result_mult.w, double(20));

    EXPECT_EQ(result_div.x, double(5));
    EXPECT_EQ(result_div.y, double(10));
    EXPECT_EQ(result_div.z, double(15));
    EXPECT_EQ(result_div.w, double(20));
}

TEST(TestHydrolibMath, QuaternionComplexMultiplication)
{
    Quaternion<double> q1(double(2), double(4), double(1), double(3));
    Quaternion<double> q2(double(3), double(5), double(2), double(1));
    Quaternion<double> result = q1 * q2;

    EXPECT_EQ(result.w, double(-25));
    EXPECT_EQ(result.x, double(14));
    EXPECT_EQ(result.y, double(18));
    EXPECT_EQ(result.z, double(5));
}