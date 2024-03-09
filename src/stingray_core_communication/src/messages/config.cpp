#include "messages/config.h"

// pult -> cm4 -> stm
RequestConfigMessage::RequestConfigMessage() : AbstractMessage() {
    flags = 0;
    stab_flags = 0;

    current_contour = 0;

    march = 0;
    lag = 0;
    depth = 0;
    roll = 0;
    pitch = 0;
    yaw = 0;

    dt = 0;
    k_joy = 0;
    k_tuning = 0;

    pid_kp = 0;
    pid_ki = 0;
    pid_kd = 0;
    pid_max_i = 0;
    pid_min_i = 0;
    pid_max = 0;
    pid_min = 0;

    posFilter_t = 0;
    posFilter_k = 0;
    speedFilter_y = 0;
    speedFilter_k = 0;

    out_max = 0;
    out_min = 0;

    checksum = 0;

    thrusters_on = 0;
    reset_imu = 0;
    reset_depth = 0;
    rgb_light_on = 0;
    lower_light_on = 0;

    stab_march = 0;
    stab_lag = 0;
    stab_depth = 0;
    stab_roll = 0;
    stab_pitch = 0;
    stab_yaw = 0;

    current_march = 0;
    current_lag = 0;
    current_depth = 0;
    current_roll = 0;
    current_pitch = 0;
    current_yaw = 0;
}

// stm -> cm4 -> pult
ResponseConfigMessage::ResponseConfigMessage() : AbstractMessage() {
    depth = 0;
    roll = 0;
    pitch = 0;
    yaw = 0;

    input = 0;
    pos_filtered = 0;
    speed_filtered = 0;

    joy_gained = 0;
    target_integrator = 0;

    pid_pre_error = 0;
    pid_error = 0;
    pid_integral = 0;
    pid_Pout = 0;
    pid_Iout = 0;
    pid_Dout = 0;
    pid_output = 0;

    tuning_summator = 0;
    speed_error = 0;
    out_pre_saturation = 0;
    out = 0;

    current_logic_electronics = 0;
    for (int i = 0; i < 8; i++) {
        current_vma[i] = 0;
    }
    for (int i = 0; i < 4; i++) {
        voltage_battery_cell[i] = 0;
    }
    voltage_battery = 0;

    checksum = 0;
}

// pull message from byte-vector (pult to raspberry_cm4)
bool RequestConfigMessage::parse(std::vector<uint8_t>& input) {
    popFromVector(input, checksum, true);
    uint16_t checksum_calc = getChecksum16b(input);
    if (checksum_calc != checksum) {
        return false;
    }

    popFromVector(input, out_min);
    popFromVector(input, out_max);

    popFromVector(input, speedFilter_k);
    popFromVector(input, speedFilter_y);
    popFromVector(input, posFilter_k);
    popFromVector(input, posFilter_t);

    popFromVector(input, pid_min);
    popFromVector(input, pid_max);
    popFromVector(input, pid_min_i);
    popFromVector(input, pid_max_i);
    popFromVector(input, pid_kd);
    popFromVector(input, pid_ki);
    popFromVector(input, pid_kp);

    popFromVector(input, k_tuning);
    popFromVector(input, k_joy);
    popFromVector(input, dt);

    popFromVector(input, yaw);
    popFromVector(input, pitch);
    popFromVector(input, roll);
    popFromVector(input, depth);
    popFromVector(input, lag);
    popFromVector(input, march);

    popFromVector(input, current_contour);

    popFromVector(input, stab_flags);
    popFromVector(input, flags);

    thrusters_on = pickBit(flags, 0);
    reset_imu = pickBit(flags, 1);
    reset_depth = pickBit(flags, 2);
    rgb_light_on = pickBit(flags, 3);
    lower_light_on = pickBit(flags, 4);

    stab_march = pickBit(stab_flags, 0);
    stab_lag = pickBit(stab_flags, 1);
    stab_depth = pickBit(stab_flags, 2);
    stab_roll = pickBit(stab_flags, 3);
    stab_pitch = pickBit(stab_flags, 4);
    stab_yaw = pickBit(stab_flags, 5);

    current_march = pickBit(current_contour, 0);
    current_lag = pickBit(current_contour, 1);
    current_depth = pickBit(current_contour, 2);
    current_roll = pickBit(current_contour, 3);
    current_pitch = pickBit(current_contour, 4);
    current_yaw = pickBit(current_contour, 5);

    return true;
}

// form byte-vector (raspberry_cm4 to pult)
void ResponseConfigMessage::pack(std::vector<uint8_t>& container) {
    pushToVector(container, depth);
    pushToVector(container, roll);
    pushToVector(container, pitch);
    pushToVector(container, yaw);

    pushToVector(container, input);
    pushToVector(container, pos_filtered);
    pushToVector(container, speed_filtered);

    pushToVector(container, joy_gained);
    pushToVector(container, target_integrator);

    pushToVector(container, pid_pre_error);
    pushToVector(container, pid_error);
    pushToVector(container, pid_integral);
    pushToVector(container, pid_Pout);
    pushToVector(container, pid_Iout);
    pushToVector(container, pid_Dout);
    pushToVector(container, pid_output);

    pushToVector(container, tuning_summator);
    pushToVector(container, speed_error);
    pushToVector(container, out_pre_saturation);
    pushToVector(container, out);

    pushToVector(container, current_logic_electronics);
    for (int i = 0; i < 8; i++) {
        pushToVector(container, current_vma[i]);
    }
    for (int i = 0; i < 4; i++) {
        pushToVector(container, voltage_battery_cell[i]);
    }
    pushToVector(container, voltage_battery);

    uint16_t checksum = getChecksum16b(container);
    pushToVector(container, checksum);  // do i need to revert bytes here?
}
