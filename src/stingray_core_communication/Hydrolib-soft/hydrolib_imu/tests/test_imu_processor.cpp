#include "hydrolib_fixed_point.hpp"
#include "hydrolib_imu_processor.hpp"

#include "hydrolib_quaternions.hpp"
#include "hydrolib_rotations.hpp"
#include "mock/imu_distortion_model.hpp"
#include "mock/raw_imu_mock.hpp"

#include <gtest/gtest.h>

using namespace hydrolib::sensors;
using namespace hydrolib::math;

TEST(TestIMUProcessor, Process)
{
    RawIMUMock<FixedPointBase> imu_mock;
    Vector3D<FixedPointBase> axis{2, 3, 1};
    axis.Normalize();
    FixedPointBase angle_rad = FixedPointBase(50, 180) * pi;
    Quaternion<FixedPointBase> target{axis * sin(angle_rad / 2),
                                      cos(angle_rad / 2)};

    imu_mock.SetTarget(axis, angle_rad, 15);

    IMUProcessor<FixedPointBase, 1.0> imu_processor;

    Quaternion<FixedPointBase> result{0, 0, 0, 1};

    auto q = imu_mock.GetOrientation();

    while (imu_mock.Step())
    {
        q = imu_mock.GetOrientation();
        std::cout << static_cast<double>(q.x) << " " << static_cast<double>(q.y)
                  << " " << static_cast<double>(q.z) << " "
                  << static_cast<double>(q.w) << std::endl;
        result = imu_processor.Process(imu_mock.GetAcceleration(),
                                       imu_mock.GetGyro());
        std::cout << static_cast<double>(result.x) << " "
                  << static_cast<double>(result.y) << " "
                  << static_cast<double>(result.z) << " "
                  << static_cast<double>(result.w) << std::endl;
        std::cout << "===========" << std::endl;
    }

    q = imu_mock.GetOrientation();
    std::cout << static_cast<double>(target.x) << " "
              << static_cast<double>(target.y) << " "
              << static_cast<double>(target.z) << " "
              << static_cast<double>(target.w) << std::endl;

    EXPECT_NEAR(static_cast<double>(result.w), static_cast<double>(target.w),
                0.01);
    EXPECT_NEAR(static_cast<double>(result.x), static_cast<double>(target.x),
                0.01);
    EXPECT_NEAR(static_cast<double>(result.y), static_cast<double>(target.y),
                0.01);
    EXPECT_NEAR(static_cast<double>(result.z), static_cast<double>(target.z),
                0.01);
}

TEST(TestIMUProcessor, ProcessCorner)
{
    RawIMUMock<FixedPointBase> imu_mock;
    Vector3D<FixedPointBase> axis{1, 0, 0};
    axis.Normalize();
    FixedPointBase angle_rad = pi;
    Quaternion<FixedPointBase> target{axis * sin(angle_rad / 2),
                                      cos(angle_rad / 2)};

    imu_mock.SetTarget(axis, angle_rad, 15);

    IMUProcessor<FixedPointBase, 1.0> imu_processor;

    Quaternion<FixedPointBase> result{0, 0, 0, 1};

    auto q = imu_mock.GetOrientation();

    while (imu_mock.Step())
    {
        q = imu_mock.GetOrientation();
        std::cout << static_cast<double>(q.x) << " " << static_cast<double>(q.y)
                  << " " << static_cast<double>(q.z) << " "
                  << static_cast<double>(q.w) << std::endl;
        result = imu_processor.Process(imu_mock.GetAcceleration(),
                                       imu_mock.GetGyro());
        std::cout << static_cast<double>(result.x) << " "
                  << static_cast<double>(result.y) << " "
                  << static_cast<double>(result.z) << " "
                  << static_cast<double>(result.w) << std::endl;
        std::cout << "===========" << std::endl;

        EXPECT_NEAR(static_cast<double>(result.w), static_cast<double>(q.w),
                    0.01);
        EXPECT_NEAR(static_cast<double>(result.x), static_cast<double>(q.x),
                    0.01);
        EXPECT_NEAR(static_cast<double>(result.y), static_cast<double>(q.y),
                    0.01);
        EXPECT_NEAR(static_cast<double>(result.z), static_cast<double>(q.z),
                    0.01);
    }

    axis = {0, 0, 1};
    axis.Normalize();
    angle_rad = pi;
    target = {axis * sin(angle_rad / 2), cos(angle_rad / 2)};

    imu_mock.SetTarget(axis, angle_rad, 15);

    while (imu_mock.Step())
    {
        q = imu_mock.GetOrientation();
        std::cout << static_cast<double>(q.x) << " " << static_cast<double>(q.y)
                  << " " << static_cast<double>(q.z) << " "
                  << static_cast<double>(q.w) << std::endl;
        result = imu_processor.Process(imu_mock.GetAcceleration(),
                                       imu_mock.GetGyro());
        std::cout << static_cast<double>(result.x) << " "
                  << static_cast<double>(result.y) << " "
                  << static_cast<double>(result.z) << " "
                  << static_cast<double>(result.w) << std::endl;
        std::cout << "===========" << std::endl;

        EXPECT_NEAR(static_cast<double>(result.w), static_cast<double>(q.w),
                    0.01);
        EXPECT_NEAR(static_cast<double>(result.x), static_cast<double>(q.x),
                    0.01);
        EXPECT_NEAR(static_cast<double>(result.y), static_cast<double>(q.y),
                    0.01);
        EXPECT_NEAR(static_cast<double>(result.z), static_cast<double>(q.z),
                    0.01);
    }
}

TEST(TestIMUProcessor, DistortionTest)
{
    Quaternion<double> rotation = {1, 2, 3, 4};
    rotation.Normalize();
    IMUDistorter<double> distorter(rotation, {0.2, 0.3, 0.4});

    RawIMUMock<double> imu_mock;
    Vector3D<double> axis{2, 3, 1};
    axis.Normalize();
    double angle_rad = 50.0 / 180 * 3.14;
    imu_mock.SetTarget(axis, angle_rad, 1);
    while (imu_mock.Step())
    {
    }

    IMUProcessor<double, 1.0> imu_processor;
    auto g_z = distorter.DistortAccel({0, 0, 1});
    auto g_opposite_z = distorter.DistortAccel({0, 0, -1});
    auto g_x = distorter.DistortAccel({1, 0, 0});
    imu_processor.Calibrate(g_z, g_opposite_z, g_x);
    auto g = imu_mock.GetAcceleration();
    auto g_distorted = distorter.DistortAccel(g);
    auto result = imu_processor.Process(g_distorted, {0, 0, 0});
    auto result_g = Rotate({0, 0, -1}, result);
    EXPECT_NEAR(static_cast<double>(result_g.x), static_cast<double>(g.x),
                0.01);
    EXPECT_NEAR(static_cast<double>(result_g.y), static_cast<double>(g.y),
                0.01);
    EXPECT_NEAR(static_cast<double>(result_g.z), static_cast<double>(g.z),
                0.01);
}
