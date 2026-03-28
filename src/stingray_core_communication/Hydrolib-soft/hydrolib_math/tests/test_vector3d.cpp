#include "hydrolib_vector3d.hpp"

#include <gtest/gtest.h>

using namespace hydrolib::math;

// Basic Vector3D construction and initialization tests
TEST(TestHydrolibMathVector3D, DoubleConstruction)
{
    Vector3D<double> vec{1.0, 2.0, 3.0};

    EXPECT_DOUBLE_EQ(vec.x, 1.0);
    EXPECT_DOUBLE_EQ(vec.y, 2.0);
    EXPECT_DOUBLE_EQ(vec.z, 3.0);
}

// Dot product tests
TEST(TestHydrolibMathVector3D, DotProductOrthogonalVectors)
{
    Vector3D<double> vec1{1.0, 0.0, 0.0};
    Vector3D<double> vec2{0.0, 1.0, 0.0};

    double result = vec1.Dot(vec2);

    EXPECT_DOUBLE_EQ(result, 0.0);
}

TEST(TestHydrolibMathVector3D, DotProductParallelVectors)
{
    Vector3D<double> vec1{3.0, 4.0, 5.0};
    Vector3D<double> vec2{3.0, 4.0, 5.0};

    double result = vec1.Dot(vec2);

    // 3*3 + 4*4 + 5*5 = 9 + 16 + 25 = 50
    EXPECT_DOUBLE_EQ(result, 50.0);
}

TEST(TestHydrolibMathVector3D, DotProductGeneralCase)
{
    Vector3D<double> vec1{1.0, 2.0, 3.0};
    Vector3D<double> vec2{4.0, 5.0, 6.0};

    double result = vec1.Dot(vec2);

    // 1*4 + 2*5 + 3*6 = 4 + 10 + 18 = 32
    EXPECT_DOUBLE_EQ(result, 32.0);
}

TEST(TestHydrolibMathVector3D, DotProductWithNegativeValues)
{
    Vector3D<double> vec1{1.0, -2.0, 3.0};
    Vector3D<double> vec2{-1.0, 2.0, 1.0};

    double result = vec1.Dot(vec2);

    // 1*(-1) + (-2)*2 + 3*1 = -1 - 4 + 3 = -2
    EXPECT_DOUBLE_EQ(result, -2.0);
}

// Cross product tests
TEST(TestHydrolibMathVector3D, CrossProductUnitVectors)
{
    Vector3D<double> vec1{1.0, 0.0, 0.0}; // i
    Vector3D<double> vec2{0.0, 1.0, 0.0}; // j

    Vector3D<double> result = vec1.Cross(vec2);

    // i × j = k
    EXPECT_DOUBLE_EQ(result.x, 0.0);
    EXPECT_DOUBLE_EQ(result.y, 0.0);
    EXPECT_DOUBLE_EQ(result.z, 1.0);
}

TEST(TestHydrolibMathVector3D, CrossProductGeneralCase)
{
    Vector3D<double> vec1{1.0, 2.0, 3.0};
    Vector3D<double> vec2{4.0, 5.0, 6.0};

    Vector3D<double> result = vec1.Cross(vec2);

    // (2*6 - 3*5, 3*4 - 1*6, 1*5 - 2*4) = (12 - 15, 12 - 6, 5 - 8) = (-3, 6,
    // -3)
    EXPECT_DOUBLE_EQ(result.x, -3.0);
    EXPECT_DOUBLE_EQ(result.y, 6.0);
    EXPECT_DOUBLE_EQ(result.z, -3.0);
}

TEST(TestHydrolibMathVector3D, CrossProductParallelVectors)
{
    Vector3D<double> vec1{2.0, 4.0, 6.0};
    Vector3D<double> vec2{1.0, 2.0, 3.0};

    Vector3D<double> result = vec1.Cross(vec2);

    // Cross product of parallel vectors should be zero vector
    EXPECT_DOUBLE_EQ(result.x, 0.0);
    EXPECT_DOUBLE_EQ(result.y, 0.0);
    EXPECT_DOUBLE_EQ(result.z, 0.0);
}

TEST(TestHydrolibMathVector3D, LengthCalculation)
{
    Vector3D<double> vec{3.0, 4.0, 0.0};

    double length = vec.Length();

    EXPECT_DOUBLE_EQ(length, 5.0);
}

TEST(TestHydrolibMathVector3D, Length3DVector)
{
    Vector3D<double> vec{1.0, 2.0, 2.0};

    double length = vec.Length();

    EXPECT_DOUBLE_EQ(length, 3.0);
}

TEST(TestHydrolibMathVector3D, ScalarMultiplicationRight)
{
    Vector3D<double> vec{1.0, 2.0, 3.0};
    double scalar(2);

    Vector3D<double> result = vec * scalar;

    EXPECT_DOUBLE_EQ(result.x, 2.0);
    EXPECT_DOUBLE_EQ(result.y, 4.0);
    EXPECT_DOUBLE_EQ(result.z, 6.0);
}

TEST(TestHydrolibMathVector3D, ScalarMultiplicationAssignment)
{
    Vector3D<double> vec{2.0, 4.0, 6.0};
    double scalar(0.5);

    vec *= scalar;

    EXPECT_DOUBLE_EQ(vec.x, 1.0);
    EXPECT_DOUBLE_EQ(vec.y, 2.0);
    EXPECT_DOUBLE_EQ(vec.z, 3.0);
}

TEST(TestHydrolibMathVector3D, ScalarMultiplicationZero)
{
    Vector3D<double> vec{5.0, 10.0, 15.0};
    double zero(0);

    Vector3D<double> result = vec * zero;

    EXPECT_DOUBLE_EQ(result.x, 0.0);
    EXPECT_DOUBLE_EQ(result.y, 0.0);
    EXPECT_DOUBLE_EQ(result.z, 0.0);
}

TEST(TestHydrolibMathVector3D, ScalarMultiplicationNegative)
{
    Vector3D<double> vec{1.0, -2.0, 3.0};
    double scalar(-2);

    Vector3D<double> result = vec * scalar;

    EXPECT_DOUBLE_EQ(result.x, -2.0);
    EXPECT_DOUBLE_EQ(result.y, 4.0);
    EXPECT_DOUBLE_EQ(result.z, -6.0);
}

TEST(TestHydrolibMathVector3D, ScalarDivision)
{
    Vector3D<double> vec{6.0, 8.0, 10.0};
    double divisor(2);

    Vector3D<double> result = vec / divisor;

    EXPECT_DOUBLE_EQ(result.x, 3.0);
    EXPECT_DOUBLE_EQ(result.y, 4.0);
    EXPECT_DOUBLE_EQ(result.z, 5.0);
}

TEST(TestHydrolibMathVector3D, ScalarDivisionAssignment)
{
    Vector3D<double> vec{9.0, 12.0, 15.0};
    double divisor(3);

    vec /= divisor;

    EXPECT_DOUBLE_EQ(vec.x, 3.0);
    EXPECT_DOUBLE_EQ(vec.y, 4.0);
    EXPECT_DOUBLE_EQ(vec.z, 5.0);
}

TEST(TestHydrolibMathVector3D, ScalarDivisionDecimal)
{
    Vector3D<double> vec{1.0, 2.0, 3.0};
    double divisor(0.5);

    Vector3D<double> result = vec / divisor;

    EXPECT_DOUBLE_EQ(result.x, 2.0);
    EXPECT_DOUBLE_EQ(result.y, 4.0);
    EXPECT_DOUBLE_EQ(result.z, 6.0);
}

TEST(TestHydrolibMathVector3D, ScalarOperationsChaining)
{
    Vector3D<double> vec{1.0, 2.0, 3.0};
    double scalar1(2);
    double scalar2(3);

    Vector3D<double> result = (vec * scalar1) * scalar2;

    EXPECT_DOUBLE_EQ(result.x, 6.0);
    EXPECT_DOUBLE_EQ(result.y, 12.0);
    EXPECT_DOUBLE_EQ(result.z, 18.0);
}

TEST(TestHydrolibMathVector3D, ScalarMultiplicationDivisionInverse)
{
    Vector3D<double> original{2.5, 5.0, 7.5};
    double scalar(4);

    Vector3D<double> multiplied = original * scalar;
    Vector3D<double> result = multiplied / scalar;

    EXPECT_DOUBLE_EQ(result.x, original.x);
    EXPECT_DOUBLE_EQ(result.y, original.y);
    EXPECT_DOUBLE_EQ(result.z, original.z);
}

TEST(TestHydrolibMathVector3D, ZeroVectorDotProduct)
{
    Vector3D<double> zero_vec{0.0, 0.0, 0.0};
    Vector3D<double> any_vec{5.0, 10.0, 15.0};

    double result = zero_vec.Dot(any_vec);

    EXPECT_DOUBLE_EQ(result, 0.0);
}

TEST(TestHydrolibMathVector3D, ZeroVectorCrossProduct)
{
    Vector3D<double> zero_vec{0.0, 0.0, 0.0};
    Vector3D<double> any_vec{5.0, 10.0, 15.0};

    Vector3D<double> result = zero_vec.Cross(any_vec);

    EXPECT_DOUBLE_EQ(result.x, 0.0);
    EXPECT_DOUBLE_EQ(result.y, 0.0);
    EXPECT_DOUBLE_EQ(result.z, 0.0);
}

TEST(TestHydrolibMathVector3D, SelfCrossProduct)
{
    Vector3D<double> vec{1.0, 2.0, 3.0};

    Vector3D<double> result = vec.Cross(vec);

    // Any vector crossed with itself should be zero vector
    EXPECT_DOUBLE_EQ(result.x, 0.0);
    EXPECT_DOUBLE_EQ(result.y, 0.0);
    EXPECT_DOUBLE_EQ(result.z, 0.0);
}

TEST(TestHydrolibMathVector3D, SelfDotProduct)
{
    Vector3D<double> vec{3.0, 4.0, 0.0};

    double result = vec.Dot(vec);

    // 3^2 + 4^2 + 0^2 = 9 + 16 + 0 = 25
    EXPECT_DOUBLE_EQ(result, 25.0);
}