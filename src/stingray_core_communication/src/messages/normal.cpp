#include "stingray_core_communication/messages/normal.h"

RequestNormalMessage::RequestNormalMessage() : AbstractMessage() {
    flags = 0;
    stab_flags = 0;
    surge = 0;
    sway = 0;
    depth = 0;
    roll = 0;
    pitch = 0;
    yaw = 0;
    for (int i = 0; i < dev_amount; i++) {
        dev[i] = 0;
    }

    checksum = 0;

    reset_imu = false;
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
    setBit(flags, 1, enable_thrusters);
    pushToVector(container, flags);

    setBit(stab_flags, 0, stab_depth);
    setBit(stab_flags, 1, stab_roll);
    setBit(stab_flags, 2, stab_pitch);
    setBit(stab_flags, 3, stab_yaw);
    pushToVector(container, stab_flags);

    pushToVector(container, surge);
    pushToVector(container, sway);
    pushToVector(container, depth);
    pushToVector(container, roll);
    pushToVector(container, pitch);
    pushToVector(container, yaw);

    for (int i = 0; i < dev_amount; i++) {
        pushToVector(container, dev[i]);
    }

    uint16_t checksum = getChecksum16b(container);
    pushToVector(container, checksum);  // do i need to revert bytes here?
}

// pull message from byte-vector (pult to raspberry_cm4)
bool RequestNormalMessage::parse(std::vector<uint8_t> &input) {
    popFromVector(input, checksum, true);
    uint16_t checksum_calc = getChecksum16b(input);
    // if (checksum_calc != checksum) {
    //     return false;
    // }

    for (int i = 0; i < dev_amount; i++) {
        popFromVector(input, dev[dev_amount - i]);
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
    enable_thrusters = pickBit(flags, 1);

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
    depth = 0;
    for (int i = 0; i < dev_amount; i++) {
        dev[i] = 0;
    }

    checksum = 0;
}

// form byte-vector (raspberry_cm4 to pult)
void ResponseNormalMessage::pack(std::vector<uint8_t> &container) {
    pushToVector(container, roll);
    pushToVector(container, pitch);
    pushToVector(container, yaw);
    pushToVector(container, depth);
    for (int i = 0; i < dev_amount; i++) {
        pushToVector(container, dev[i]);
    }

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

    for (int i = 0; i < dev_amount; i++) {
        popFromVector(input, dev[dev_amount - i]);
    }
    popFromVector(input, depth);
    popFromVector(input, yaw);
    popFromVector(input, pitch);
    popFromVector(input, roll);

    return true;
}
