#include "hydrolib_fixed_point.hpp"

#include <gtest/gtest.h>

#include <cmath>

using namespace hydrolib::math;

TEST(TestHydrolibMath, FixedPointBaseConstructor)
{
    FixedPointBase fp(5.0);

    EXPECT_DOUBLE_EQ(static_cast<double>(fp), 5.0);
}

TEST(TestHydrolibMath, FixedPointBaseConstructorWithDivider)
{
    FixedPointBase fp(5, 2);

    EXPECT_DOUBLE_EQ(static_cast<double>(fp), 2.5);
}

TEST(TestHydrolibMath, FixedPointBaseConstructorWithLiteral)
{
    auto fp = 3.5_fp;

    EXPECT_DOUBLE_EQ(static_cast<double>(fp), 3.5);
}

TEST(TestHydrolibMath, FixedPointBaseAddition)
{
    FixedPointBase a(10);
    FixedPointBase b(5);
    FixedPointBase result = a + b;

    EXPECT_DOUBLE_EQ(static_cast<double>(result), 15.0);
}

TEST(TestHydrolibMath, FixedPointBaseAdditionAssignment)
{
    FixedPointBase a(10);
    FixedPointBase b(5);
    a += b;

    EXPECT_DOUBLE_EQ(static_cast<double>(a), 15.0);
}

TEST(TestHydrolibMath, FixedPointBaseSubtraction)
{
    FixedPointBase a(10);
    FixedPointBase b(3);
    FixedPointBase result = a - b;

    EXPECT_DOUBLE_EQ(static_cast<double>(result), 7.0);
}

TEST(TestHydrolibMath, FixedPointBaseSubtractionWithNegative)
{
    FixedPointBase a(5);
    FixedPointBase b(-3);
    FixedPointBase result = a - b;

    EXPECT_DOUBLE_EQ(static_cast<double>(result), 8.0);
}

TEST(TestHydrolibMath, FixedPointBaseSubtractionAssignment)
{
    FixedPointBase a(10);
    FixedPointBase b(3);
    a -= b;

    EXPECT_DOUBLE_EQ(static_cast<double>(a), 7.0);
}

TEST(TestHydrolibMath, FixedPointBaseUnaryMinus)
{
    FixedPointBase a(10);
    FixedPointBase b(-5);

    EXPECT_DOUBLE_EQ(static_cast<double>(-a), -10.0);
    EXPECT_DOUBLE_EQ(static_cast<double>(-b), 5.0);
}

TEST(TestHydrolibMath, FixedPointBaseMultiplication)
{
    FixedPointBase a(4);
    FixedPointBase b(3);
    FixedPointBase result = a * b;

    EXPECT_DOUBLE_EQ(static_cast<double>(result), 12.0);
}

TEST(TestHydrolibMath, FixedPointBaseMultiplicationAssignment)
{
    FixedPointBase a(6);
    FixedPointBase b(2);
    a *= b;

    EXPECT_DOUBLE_EQ(static_cast<double>(a), 12.0);
}

TEST(TestHydrolibMath, FixedPointBaseDivision)
{
    FixedPointBase a(15);
    FixedPointBase b(3);
    FixedPointBase result = a / b;

    EXPECT_DOUBLE_EQ(static_cast<double>(result), 5.0);
}

TEST(TestHydrolibMath, FixedPointBaseDivisionAssignment)
{
    FixedPointBase a(20);
    FixedPointBase b(4);
    a /= b;

    EXPECT_DOUBLE_EQ(static_cast<double>(a), 5.0);
}

TEST(TestHydrolibMath, FixedPointBaseChainOperations)
{
    FixedPointBase a(10);
    FixedPointBase b(5);
    FixedPointBase c(2);

    FixedPointBase result = (a + b) * c;
    EXPECT_DOUBLE_EQ(static_cast<double>(result), 30.0);

    FixedPointBase result2 = a * b - c;
    EXPECT_DOUBLE_EQ(static_cast<double>(result2), 48.0);
}

TEST(TestHydrolibMath, FixedPointBaseLargeNumbers)
{
    constexpr double a = 31.999;
    constexpr double b = 31.999;
    FixedPointBase large1(a);
    FixedPointBase large2(b);

    EXPECT_NEAR(static_cast<double>(large1 + large2),
                static_cast<double>(large1) + static_cast<double>(large2),
                1.0 / (1 << FixedPointBase::GetFractionBits()));
    EXPECT_NEAR(static_cast<double>(large1 - large2),
                static_cast<double>(large1) - static_cast<double>(large2),
                1.0 / (1 << FixedPointBase::GetFractionBits()));
    EXPECT_NEAR(static_cast<double>(large1 * large2),
                static_cast<double>(large1) * static_cast<double>(large2),
                1.0 / (1 << FixedPointBase::GetFractionBits()));
    EXPECT_NEAR(static_cast<double>(large1 / large2),
                static_cast<double>(large1) / static_cast<double>(large2),
                1.0 / (1 << FixedPointBase::GetFractionBits()));
}

TEST(TestHydrolibMath, FixedPointBaseSmallDecimals)
{
    constexpr double a = 0.001;
    constexpr double b = 0.001;
    FixedPointBase small1(a);
    FixedPointBase small2(b);

    EXPECT_NEAR(static_cast<double>(small1 + small2),
                static_cast<double>(small1) + static_cast<double>(small2),
                1.0 / (1 << FixedPointBase::GetFractionBits()));
    EXPECT_NEAR(static_cast<double>(small1 - small2),
                static_cast<double>(small1) - static_cast<double>(small2),
                1.0 / (1 << FixedPointBase::GetFractionBits()));
    EXPECT_NEAR(static_cast<double>(small1 * small2),
                static_cast<double>(small1) * static_cast<double>(small2),
                1.0 / (1 << FixedPointBase::GetFractionBits()));
    EXPECT_NEAR(static_cast<double>(small1 / small2),
                static_cast<double>(small1) / static_cast<double>(small2),
                1.0 / (1 << FixedPointBase::GetFractionBits()));
}

TEST(TestHydrolibMath, FixedPointBaseSqrt)
{
    FixedPointBase a(16);
    FixedPointBase result = sqrt(a);

    EXPECT_DOUBLE_EQ(static_cast<double>(result), 4.0);

    FixedPointBase b(9);
    FixedPointBase result2 = sqrt(b);

    EXPECT_DOUBLE_EQ(static_cast<double>(result2), 3.0);

    auto c = 6.25_fp;
    FixedPointBase result3 = sqrt(c);

    EXPECT_DOUBLE_EQ(static_cast<double>(result3), 2.5);

    auto d = 0.25_fp;
    FixedPointBase result4 = sqrt(d);

    EXPECT_DOUBLE_EQ(static_cast<double>(result4), 0.5);
}

TEST(TestHydrolibMath, FixedPointBaseSin)
{
    constexpr double rads = 3.14159265358979323846 / 4;
    FixedPointBase a(rads);
    FixedPointBase result = sin(a);

    EXPECT_NEAR(static_cast<double>(result), sin(rads), 0.002);
}

TEST(TestHydrolibMath, FixedPointBaseCos)
{
    constexpr double rads = 3.14159265358979323846 / 4;
    FixedPointBase a(rads);
    FixedPointBase result = cos(a);

    EXPECT_NEAR(static_cast<double>(result), cos(rads), 0.002);
}

TEST(TestHydrolibMath, FixedPointBaseGetFractionBits)
{
    EXPECT_EQ(FixedPointBase::GetFractionBits(), 16);
}

TEST(TestHydrolibMath, FixedPointBaseGetAbsIntPart)
{
    FixedPointBase fp1(5.75);
    EXPECT_EQ(fp1.GetAbsIntPart(), 5);

    FixedPointBase fp2(10.25);
    EXPECT_EQ(fp2.GetAbsIntPart(), 10);

    FixedPointBase fp3(0.99);
    EXPECT_EQ(fp3.GetAbsIntPart(), 0);

    FixedPointBase fp4(-5.75);
    EXPECT_EQ(fp4.GetAbsIntPart(), 5);

    FixedPointBase fp5(-0.25);
    EXPECT_EQ(fp5.GetAbsIntPart(), 0);

    FixedPointBase fp6(42);
    EXPECT_EQ(fp6.GetAbsIntPart(), 42);

    FixedPointBase fp7(-15);
    EXPECT_EQ(fp7.GetAbsIntPart(), 15);

    FixedPointBase fp8(0);
    EXPECT_EQ(fp8.GetAbsIntPart(), 0);
}

TEST(TestHydrolibMath, FixedPointBaseGetAbsFractionPart)
{
    FixedPointBase fp1(5.25);
    EXPECT_EQ(fp1.GetAbsFractionPart(),
              0.25 * (1 << FixedPointBase::GetFractionBits()));

    FixedPointBase fp2(3.5);
    EXPECT_EQ(fp2.GetAbsFractionPart(),
              0.5 * (1 << FixedPointBase::GetFractionBits()));

    FixedPointBase fp3(7.75);
    EXPECT_EQ(fp3.GetAbsFractionPart(),
              0.75 * (1 << FixedPointBase::GetFractionBits()));

    FixedPointBase fp4(10);
    EXPECT_EQ(fp4.GetAbsFractionPart(), 0);

    FixedPointBase fp5(0);
    EXPECT_EQ(fp5.GetAbsFractionPart(), 0);
}

TEST(TestHydrolibMath, FixedPointBaseGetAbsFractionPartNegative)
{
    FixedPointBase fp6(-2.25);
    EXPECT_EQ(fp6.GetAbsFractionPart(),
              0.25 * (1 << FixedPointBase::GetFractionBits()));
}

TEST(TestHydrolibMath, FixedPointBaseComparisonOperators)
{
    FixedPointBase a(5);
    FixedPointBase b(3);
    FixedPointBase c(5);

    // FixedPoint to FixedPoint comparisons
    EXPECT_TRUE(a > b);
    EXPECT_TRUE(b < a);
    EXPECT_TRUE(a >= c);
    EXPECT_TRUE(c >= a);
    EXPECT_TRUE(b <= a);
    EXPECT_FALSE(a < b);
    EXPECT_TRUE(a != b);
    EXPECT_FALSE(a != c);

    // FixedPoint to int comparisons
    EXPECT_TRUE(a == 5);
    EXPECT_TRUE(a != 3);
    EXPECT_TRUE(a > 3);
    EXPECT_TRUE(a >= 5);
    EXPECT_TRUE(b < 5);
    EXPECT_TRUE(b <= 3);
    EXPECT_FALSE(a < 3);
    EXPECT_FALSE(b > 5);
}

TEST(TestHydrolibMath, FixedPointBaseDecimalComparisons)
{
    auto a = 5.5_fp;
    auto b = 3.2_fp;
    auto c = 5.5_fp;

    EXPECT_TRUE(a > b);
    EXPECT_TRUE(b < a);
    EXPECT_TRUE(a >= c);
    EXPECT_TRUE(a <= c);
    EXPECT_TRUE(a == c);
    EXPECT_TRUE(a != b);

    EXPECT_TRUE(a > 5);
    EXPECT_TRUE(a >= 5);
    EXPECT_FALSE(a == 5);
    EXPECT_TRUE(b < 4);
    EXPECT_TRUE(b <= 4);
}
