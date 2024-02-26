#include "messages/normal.h"

RequestNormalMessage::RequestNormalMessage() : AbstractMessage() {
    flags = 0;
    stab_flags = 0;
    surge = 0;
    sway = 0;
    depth = 0;
    roll = 0;
    pitch = 0;
    yaw = 0;

    checksum = 0;

    reset_imu = false;
    reset_pc = false;
    enable_thrusters = false;

    stab_depth = false;
    stab_roll = false;
    stab_pitch = false;
    stab_yaw = false;
}

// form byte-vector (raspberry_cm4 to STM) // TODO
void RequestNormalMessage::pack(std::vector<uint8_t> &container) {
    pushToVector(container, type);

    setBit(flags, 0, reset_imu);
    setBit(flags, 1, reset_pc);
    setBit(flags, 2, enable_thrusters);
    pushToVector(container, flags);

    setBit(stab_flags, 0, depth);
    setBit(stab_flags, 1, roll);
    setBit(stab_flags, 2, pitch);
    setBit(stab_flags, 3, yaw);
    pushToVector(container, stab_flags);

    pushToVector(container, surge);
    pushToVector(container, sway);
    pushToVector(container, depth);
    pushToVector(container, roll);
    pushToVector(container, pitch);
    pushToVector(container, yaw);
    pushToVector(container, dropper);
    pushToVector(container, grabber);

    uint16_t checksum = getChecksum16b(container);
    pushToVector(container, checksum);  // do i need to revert bytes here?
}

// pull message from byte-vector (pult to raspberry_cm4)
bool RequestNormalMessage::parse(std::vector<uint8_t> &input) {
    popFromVector(input, checksum, true);
    uint16_t checksum_calc = getChecksum16b(input);
    if (checksum_calc != checksum) {
        return false;
    }

    popFromVector(input, yaw);
    popFromVector(input, pitch);
    popFromVector(input, roll);
    popFromVector(input, depth);
    popFromVector(input, sway);
    popFromVector(input, surge);
    popFromVector(input, stab_flags);
    popFromVector(input, flags);

    reset_imu = pickBit(flags, 0);
    reset_pc = pickBit(flags, 1);
    enable_thrusters = pickBit(flags, 2);

    stab_depth = pickBit(stab_flags, 0);
    stab_roll = pickBit(stab_flags, 1);
    stab_pitch = pickBit(stab_flags, 2);
    stab_yaw = pickBit(stab_flags, 3);

    return true;
}

ResponseNormalMessage::ResponseNormalMessage() {
    roll = 0;
    pitch = 0;
    yaw = 0;
    roll_speed = 0;
    pitch_speed = 0;
    yaw_speed = 0;
    depth = 0;
    dropper = 0;
    grabber = 0;

    checksum = 0;
}

// form byte-vector (raspberry_cm4 to pult)
void ResponseNormalMessage::pack(std::vector<uint8_t> &container) {
    pushToVector(container, roll);
    pushToVector(container, pitch);
    pushToVector(container, yaw);
    pushToVector(container, roll_speed);
    pushToVector(container, pitch_speed);
    pushToVector(container, yaw_speed);
    pushToVector(container, depth);
    pushToVector(container, dropper);
    pushToVector(container, grabber);

    uint16_t checksum = getChecksum16b(container);
    pushToVector(container, checksum);  // do i need to revert bytes here?
};

// pull message from byte-vector (STM to raspberry_cm4)
bool ResponseNormalMessage::parse(std::vector<uint8_t> &input) {
    popFromVector(input, checksum, true);

    uint16_t checksum_calc = getChecksum16b(input);

    // if (checksum_calc != checksum) {
    //     return false;
    // }

    popFromVector(input, grabber);
    popFromVector(input, dropper);
    popFromVector(input, depth);
    popFromVector(input, yaw_speed);
    popFromVector(input, pitch_speed);
    popFromVector(input, roll_speed);
    popFromVector(input, yaw); 
    popFromVector(input, pitch); 
    popFromVector(input, roll); 

    return true;
}
