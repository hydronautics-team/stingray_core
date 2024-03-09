#ifndef STINGRAY_MESSAGES_NORMAL_H
#define STINGRAY_MESSAGES_NORMAL_H

#include "stingray_core_communication/messages/common.h"

// pult -> cm4 -> stm
struct WeltMessage : public AbstractMessage {
    WeltMessage();

    const static uint8_t length = 33; // 31(message) + 2(checksum) = 31 dyte

    uint8_t reset_imu;
    uint8_t stab_depth;
    uint8_t stab_roll;
    uint8_t stab_pitch;
    uint8_t stab_yaw;
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

    void pack(std::vector<uint8_t> &container) override; // raspberry_cm4 to STM
    bool parse(std::vector<uint8_t> &input) override; // pult to raspberry_cm4
};

#endif  // STINGRAY_MESSAGES_NORMAL_H
