#include "messages/direct.h"

// pult -> cm4 -> stm
RequestDirectMessage::RequestDirectMessage() : AbstractMessage() {
    flags = 0;

    id = 0;
    adress = 0;

    target_forse = 0;

    reverse = 0;
    k_forward = 0;
    k_backward = 0;

    s_forward = 0;
    s_backward = 0;

    checksum = 0;

    thrusters_on = 0;
    reset_imu = 0;
    reset_depth = 0;
    rgb_light_on = 0;
    lower_light_on = 0;
}

// stm -> cm4 -> pult
ResponseDirectMessage::ResponseDirectMessage() : AbstractMessage() {
    id = 0;

    current_logic_electronics = 0;
    for (int i = 0; i < 4; i++) {
        current_vma[i] = 0;
    }
    for (int i = 0; i < 8; i++) {
        voltage_battery_cell[i] = 0;
    }
    voltage_battery = 0;

    checksum = 0;
}

// pult to raspberry_cm4
bool RequestDirectMessage::parse(std::vector<uint8_t>& input) {
    popFromVector(input, checksum, true);
    uint16_t checksum_calc = getChecksum16b(input);
    if (checksum_calc != checksum) {
        return false;
    }

    popFromVector(input, s_backward);
    popFromVector(input, s_forward);

    popFromVector(input, k_backward);
    popFromVector(input, k_forward);
    popFromVector(input, reverse);

    popFromVector(input, target_forse);

    popFromVector(input, adress);
    popFromVector(input, id);

    popFromVector(input, flags);

    thrusters_on = pickBit(flags, 0);
    reset_imu = pickBit(flags, 1);
    reset_depth = pickBit(flags, 2);
    rgb_light_on = pickBit(flags, 3);
    lower_light_on = pickBit(flags, 4);

    return true;
}

// form byte-vector (raspberry_cm4 to pult)
void ResponseDirectMessage::pack(std::vector<uint8_t>& container) {
    pushToVector(container, id);

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
