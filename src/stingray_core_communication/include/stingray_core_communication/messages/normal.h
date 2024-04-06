#ifndef STINGRAY_MESSAGES_NORMAL_H
#define STINGRAY_MESSAGES_NORMAL_H

#include "stingray_core_communication/messages/common.h"



// pult -> cm4 -> stm
struct RequestNormalMessage : public AbstractMessage {
    RequestNormalMessage();

    const static uint8_t length = 31; // 1(type) + 28(message) + 2(checksum) = 31 dyte
    /// Number of the devs
    static const uint8_t dev_amount = 2;

    // parsel start
    const static uint8_t type = 0xA5;

    uint8_t flags; // [0]reset_imu, [1]enable_thrusters
    uint8_t stab_flags; // [0]depth, [1]roll, [2]pitch, [3]yaw
    float surge; // NED coordinate system
    float sway;
    float depth;
    float roll;
    float pitch;
    float yaw;
    int8_t dev[dev_amount];

    uint16_t checksum;
    // parsel end

    bool stab_depth;
    bool stab_roll;
    bool stab_pitch;
    bool stab_yaw;
    bool reset_imu;
    bool enable_thrusters;

    void pack(std::vector<uint8_t> &container) override; // raspberry_cm4 to STM
    bool parse(std::vector<uint8_t> &input) override; // pult to raspberry_cm4
};

// stm -> cm4 -> pult
struct ResponseNormalMessage : public AbstractMessage {
    ResponseNormalMessage();

    const static uint8_t length = 28; // 26(message) + 2(checksum) = 30 dyte
    /// Number of the devs
    static const uint8_t dev_amount = 2;
    static const uint8_t flare_amount = 0;

    // parsel start
    float roll;
    float pitch;
    float yaw;
    float surge_accel;
    float sway_accel;
    float depth;
    int8_t dev[dev_amount];
    char flare_seq[flare_amount];


    uint16_t checksum;
    // parsel end

    void pack(std::vector<uint8_t> &container) override; // raspberry_cm4 to pult
    bool parse(std::vector<uint8_t> &input) override; // STM to raspberry_cm4
};

#endif  // STINGRAY_MESSAGES_NORMAL_H
