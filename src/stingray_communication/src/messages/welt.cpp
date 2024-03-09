#include "messages/welt.h"

WeltMessage::WeltMessage() : AbstractMessage() {
    reset_imu = 0;
    stab_depth = 0;
    stab_roll = 0;
    stab_pitch = 0;
    stab_yaw = 0;
    surge = 0.0;
    sway = 0.0;
    depth = 0.0;
    roll = 0.0;
    pitch = 0.0;
    yaw = 0.0;
    dropper = 0;
    grabber = 0;

    checksum = 0;
}

// form byte-vector (raspberry_cm4 to STM) // TODO
void WeltMessage::pack(std::vector<uint8_t> &container) {
    pushToVector(container, reset_imu);
    pushToVector(container, stab_depth);
    pushToVector(container, stab_roll);
    pushToVector(container, stab_pitch);
    pushToVector(container, stab_yaw);
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
bool WeltMessage::parse(std::vector<uint8_t> &input) {
    popFromVector(input, checksum, true);
    uint16_t checksum_calc = getChecksum16b(input);
    if (checksum_calc != checksum) {
        return false;
    }

    popFromVector(input, grabber);
    popFromVector(input, dropper);
    popFromVector(input, yaw);
    popFromVector(input, pitch);
    popFromVector(input, roll);
    popFromVector(input, depth);
    popFromVector(input, sway);
    popFromVector(input, surge);
    popFromVector(input, stab_yaw);
    popFromVector(input, stab_pitch);
    popFromVector(input, stab_roll);
    popFromVector(input, stab_depth);
    popFromVector(input, reset_imu);

    return true;
}