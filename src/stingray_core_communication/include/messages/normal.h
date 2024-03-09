#ifndef STINGRAY_MESSAGES_NORMAL_H
#define STINGRAY_MESSAGES_NORMAL_H

#include "messages/common.h"

// pult -> cm4 -> stm
struct RequestNormalMessage : public AbstractMessage {
    RequestNormalMessage();

    const static uint8_t length = 31; // 1(type) + 28(message) + 2(checksum) = 31 dyte

    // parsel start
    const static uint8_t type = 0xA5;

    uint8_t flags; // [0]reset_imu, [1]reset_pc, [2]enable_thrusters
    uint8_t stab_flags; // [0]depth, [1]roll, [2]pitch, [3]yaw
    float surge; // NED coordinate system
    float sway;
    float depth;
    float roll;
    float pitch;
    float yaw;
    int8_t dropper;
	int8_t grabber;

    uint16_t checksum;
    // parsel end

    bool stab_depth;
    bool stab_roll;
    bool stab_pitch;
    bool stab_yaw;
    bool reset_imu;
	bool reset_pc;
    bool enable_thrusters;

    void pack(std::vector<uint8_t>& container) override; // raspberry_cm4 to STM
    bool parse(std::vector<uint8_t>& input) override; // pult to raspberry_cm4
};

// stm -> cm4 -> pult
struct ResponseNormalMessage : public AbstractMessage {
    ResponseNormalMessage();

    const static uint8_t length = 32; // 30(message) + 2(checksum) = 30 dyte

    float roll;
    float pitch;
    float yaw;
    float roll_speed;
    float pitch_speed;
    float yaw_speed;
    float depth;
    int8_t dropper;
	int8_t grabber;

    uint16_t checksum;

    void pack(std::vector<uint8_t>& container) override; // raspberry_cm4 to pult
    bool parse(std::vector<uint8_t>& input) override; // STM to raspberry_cm4
};

#endif  // STINGRAY_MESSAGES_NORMAL_H
