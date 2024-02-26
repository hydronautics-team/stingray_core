#ifndef STINGRAY_MESSAGES_DIRECT_H
#define STINGRAY_MESSAGES_DIRECT_H

#include "messages/common.h"

struct RequestDirectMessage : public AbstractMessage {
    RequestDirectMessage();

    const static uint8_t length = 23; // 1(type) + 20(message) + 2(checksum) = 23 dyte

    const static uint8_t type = 0xAA;

    uint8_t flags; // [0]thrusters_on, [1]reset_imu, [2]reset_depth, [3]rgb_light_on, [4]lower_light_on,

    uint8_t id; // 0..7
    uint8_t adress; // 0..7

    float_t target_forse; // newton

    uint8_t reverse;
    float_t k_forward;
    float_t k_backward;

    int16_t s_forward; // max PWM
    int16_t s_backward; // min PWM

    uint16_t checksum; // 1(type) + 20(message) + 2(checksum) = 23 dyte

    bool thrusters_on;
    bool reset_imu;
    bool reset_depth;
    bool rgb_light_on;
    bool lower_light_on;

    bool parse(std::vector<uint8_t>& input) override; // pult to raspberry_cm4
};

/** @brief Structure for storing and processing data from the STM32 direct request message protocol
 * Shore send requests and STM send responses
 */
struct ResponseDirectMessage : public AbstractMessage {
    ResponseDirectMessage();

    const static uint8_t length = 59; // 57(message) + 2(checksum) = 59 dyte

    uint8_t id; // 0..7

    float_t current_logic_electronics; // from jetson + raspberry dc-dc
    float_t current_vma[8];
    float_t voltage_battery_cell[4];
    float_t voltage_battery; // 56

    uint16_t checksum; // 57(message) + 2(checksum) = 59 dyte

    void pack(std::vector<uint8_t>& container) override; // raspberry_cm4 to pult
};
#endif  // STINGRAY_MESSAGES_DIRECT_H
