#include <gtest/gtest.h>

#include "hydrolib_thrust_generator.hpp"

constexpr int THRUST_COUNT = 6;
constexpr int DEGREES_OF_FREEDOM_COUNT = 6;

TEST(TestHydrolibThrustGenerator, GenerationThrust)
{
    int thrust_matrix[DEGREES_OF_FREEDOM_COUNT][THRUST_COUNT] = {
        {+1, -1, -1, +1, -1, +1}, {+1, +1, +0, +0, +1, +1},
        {+0, +0, -1, +1, +0, +0}, {+0, +0, +1, +1, +0, +0},
        {-1, +1, +0, +0, -1, +1}, {+1, +1, +0, +0, -1, -1}};

    hydrolib::controlling::ThrustGenerator<THRUST_COUNT> generator(
        thrust_matrix[0], thrust_matrix[1], thrust_matrix[2], thrust_matrix[3],
        thrust_matrix[4], thrust_matrix[5], 5);
    int thrust_expectency[THRUST_COUNT] = {2, 2, -1, 3, -2, 2};
    // hydrolib::controlling::Control control = {0, 0, 0, 0, 0, 0};
    // for (int i = 0; i < THRUST_COUNT; i++)
    // {
    //     control.x_torque += thrust_matrix[0][i] * thrust_expectency[i];
    //     control.y_torque += thrust_matrix[1][i] * thrust_expectency[i];
    //     control.z_torque += thrust_matrix[2][i] * thrust_expectency[i];
    //     control.x_force += thrust_matrix[3][i] * thrust_expectency[i];
    //     control.y_force += thrust_matrix[4][i] * thrust_expectency[i];
    //     control.z_force += thrust_matrix[5][i] * thrust_expectency[i];
    // }
    hydrolib::controlling::Control control = {1, 1, 1, 1, 1, 1};
    int thrust_result[THRUST_COUNT];
    generator.ProcessWithFeedback(control, thrust_result);
    for (int i = 0; i < THRUST_COUNT; i++)
    {
        EXPECT_EQ(thrust_result[i], thrust_expectency[i]);
    }
}
