#pragma once

namespace hydrolib::controlling
{

struct Control
{
    int x_force;
    int y_force;
    int z_force;

    int x_torque;
    int y_torque;
    int z_torque;
};

template <unsigned THRUSTERS_COUNT, bool ENABLE_SUM_CLAMP = false>
class ThrustGenerator
{

public:
    constexpr ThrustGenerator(int *x_rotation_gain, int *y_rotation_gain,
                              int *z_rotation_gain, int *x_linear_gain,
                              int *y_linear_gain, int *z_linear_gain,
                              int single_clamp, int sum_clamp = 0);

public:
    void ProcessWithFeedback(Control &control, int *dest);

private:
    int x_rotation_gain_[THRUSTERS_COUNT];
    int y_rotation_gain_[THRUSTERS_COUNT];
    int z_rotation_gain_[THRUSTERS_COUNT];

    int x_linear_gain_[THRUSTERS_COUNT];
    int y_linear_gain_[THRUSTERS_COUNT];
    int z_linear_gain_[THRUSTERS_COUNT];

    int single_clamp_;

    int sum_clamp_;
};

template <unsigned THRUSTERS_COUNT, bool ENABLE_SUM_CLAMP>
constexpr ThrustGenerator<THRUSTERS_COUNT, ENABLE_SUM_CLAMP>::ThrustGenerator(
    int *x_rotation_gain, int *y_rotation_gain, int *z_rotation_gain,
    int *x_linear_gain, int *y_linear_gain, int *z_linear_gain,
    int single_clamp, int sum_clamp)
    : single_clamp_(single_clamp), sum_clamp_(sum_clamp)
{
    for (unsigned i = 0; i < THRUSTERS_COUNT; i++)
    {
        x_rotation_gain_[i] = x_rotation_gain[i];
        x_linear_gain_[i] = x_linear_gain[i];

        y_rotation_gain_[i] = y_rotation_gain[i];
        y_linear_gain_[i] = y_linear_gain[i];

        z_rotation_gain_[i] = z_rotation_gain[i];
        z_linear_gain_[i] = z_linear_gain[i];
    }
}

template <unsigned THRUSTERS_COUNT, bool ENABLE_SUM_CLAMP>
void ThrustGenerator<THRUSTERS_COUNT, ENABLE_SUM_CLAMP>::ProcessWithFeedback(
    Control &control, int *dest)
{
    int max = 0;
    int sum = 0;
    for (unsigned i = 0; i < THRUSTERS_COUNT; i++)
    {
        dest[i] = x_rotation_gain_[i] * control.x_torque +
                  x_linear_gain_[i] * control.x_force +
                  y_rotation_gain_[i] * control.y_torque +
                  y_linear_gain_[i] * control.y_force +
                  z_rotation_gain_[i] * control.z_torque +
                  z_linear_gain_[i] * control.z_force;
        int dest_abs = dest[i] >= 0 ? dest[i] : -dest[i];

        if (dest_abs > max)
        {
            max = dest_abs;
        }

        if constexpr (ENABLE_SUM_CLAMP)
        {
            sum += dest_abs;
        }
    }

    unsigned enumerator = 1;
    unsigned denumerator = 1;

    if (max > single_clamp_)
    {
        enumerator = single_clamp_;
        denumerator = max;
    }

    if constexpr (ENABLE_SUM_CLAMP)
    {
        if (sum > sum_clamp_)
        {
            if (sum_clamp_ * denumerator < sum * enumerator)
            {
                enumerator = sum_clamp_;
                denumerator = sum;
            }
        }
    }

    if (enumerator == denumerator)
    {
        for (unsigned i = 0; i < THRUSTERS_COUNT; i++)
        {
            dest[i] = dest[i] * enumerator / denumerator;
        }
        control.x_torque = control.x_torque * enumerator / denumerator;
        control.y_torque = control.y_torque * enumerator / denumerator;
        control.z_torque = control.z_torque * enumerator / denumerator;
        control.x_force = control.x_force * enumerator / denumerator;
        control.y_force = control.y_force * enumerator / denumerator;
        control.z_force = control.z_force * enumerator / denumerator;
    }
}
} // namespace hydrolib::controlling
