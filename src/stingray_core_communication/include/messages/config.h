#ifndef STINGRAY_MESSAGES_CONFIG_H
#define STINGRAY_MESSAGES_CONFIG_H

#include "messages/common.h"

// pult -> cm4 -> stm
struct RequestConfigMessage : public AbstractMessage {
    RequestConfigMessage();

    const static uint8_t length = 94; // 1(type) + 91(message) + 2(checksum) = 94 dyte

    const static uint8_t type = 0x55;

    uint8_t flags; // [0]thrusters_on, [1]reset_imu, [2]reset_depth, [3]rgb_light_on, [4]lower_light_on,
    uint8_t stab_flags; // stab [0]march, [1]lag, [2]depth, [3]roll, [4]pitch, [5]yaw, [6]thrusters_on, [6]reset_imu

    uint8_t current_contour; // current contour: [0]march, [1]lag, [2]depth, [3]roll, [4]pitch, [5]yaw

    float_t march;
    float_t lag;
    float_t depth;
    float_t roll;
    float_t pitch;
    float_t yaw;

    float_t dt;
    float_t k_joy;
    float_t k_tuning;

    float_t pid_kp;
    float_t pid_ki;
    float_t pid_kd;
    float_t pid_max_i;
    float_t pid_min_i;
    float_t pid_max;
    float_t pid_min;

    float_t posFilter_t;
    float_t posFilter_k;
    float_t speedFilter_y;
    float_t speedFilter_k;

    float_t out_max;
    float_t out_min;

    uint16_t checksum;

    bool thrusters_on;
    bool reset_imu;
    bool reset_depth;
    bool rgb_light_on;
    bool lower_light_on;

    bool stab_march;
    bool stab_lag;
    bool stab_depth;
    bool stab_roll;
    bool stab_pitch;
    bool stab_yaw;

    bool current_march;
    bool current_lag;
    bool current_depth;
    bool current_roll;
    bool current_pitch;
    bool current_yaw;

    bool parse(std::vector<uint8_t>& input) override; // pult to raspberry_cm4
};

// stm -> cm4 -> pult
struct ResponseConfigMessage : public AbstractMessage {
    ResponseConfigMessage();

    const static uint8_t length = 138; // 136(message) + 2(checksum) = 138 dyte

    float_t depth;
    float_t roll;
    float_t pitch;
    float_t yaw;

    float_t input;
    float_t pos_filtered;
    float_t speed_filtered;

    float_t joy_gained;
    float_t target_integrator;

    float_t pid_pre_error;
    float_t pid_error;
    float_t pid_integral;
    float_t pid_Pout;
    float_t pid_Iout;
    float_t pid_Dout;
    float_t pid_output;

    float_t tuning_summator;
    float_t speed_error;
    float_t out_pre_saturation;
    float_t out;

    float_t current_logic_electronics; // from jetson + raspberry dc-dc
    float_t current_vma[8];
    float_t voltage_battery_cell[4];
    float_t voltage_battery; // 56

    uint16_t checksum; // 136(message) + 2(checksum) = 138 dyte

    void pack(std::vector<uint8_t>& container) override; // raspberry_cm4 to pult
};

#endif  // STINGRAY_MESSAGES_CONFIG_H
