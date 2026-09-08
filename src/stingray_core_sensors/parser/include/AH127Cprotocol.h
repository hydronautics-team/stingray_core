#ifndef AH127CPROTOCOL_H
#define AH127CPROTOCOL_H

#include <stdint.h>
#include <string>
#include <vector>
#include <chrono>
#include <libserial/SerialPort.h>

#pragma pack(push,1)

struct Header_AH {
    uint8_t identif = 0x77;
    uint8_t lenght = 0x38;
    uint8_t adress = 0x00;
    uint8_t command = 0x59;
};

struct DataFromAH127C {
    Header_AH header;
    float yaw = 0.0;
    float pitch = 0.0;
    float roll = 0.0;
    float X_accel = 0.0;
    float Y_accel = 0.0;
    float Z_accel = 0.0;
    float X_rate = 0.0;
    float Y_rate = 0.0;
    float Z_rate = 0.0;
    float X_magn = 0.0;
    float Y_magn = 0.0;
    float Z_magn = 0.0;
    float first_qvat = 0.0;
    float second_qvat = 0.0;
    float third_qvat = 0.0;
    float four_qvat = 0.0;
    float crc = 0.0;
};

struct Header_AH_calibration_start {
    uint8_t identif = 0x77;
    uint8_t lenght = 0x05;
    uint8_t adress = 0x00;
    uint8_t command = 0x91;
    uint8_t datafield = 0x00;
    uint8_t checksum = 0x96;
};

struct Header_AH_calibration_end {
    uint8_t identif = 0x77;
    uint8_t lenght = 0x05;
    uint8_t adress = 0x00;
    uint8_t command = 0x92;
    uint8_t datafield = 0x00;
    uint8_t checksum = 0x97;
};

#pragma pack(pop)

class AH127Cprotocol {
public:
    AH127Cprotocol(std::string portName, int baudRate);
    ~AH127Cprotocol() { if (m_port.IsOpen()) m_port.Close(); }

    DataFromAH127C data;
    Header_AH_calibration_start calibr_start;
    Header_AH_calibration_end calibr_end;

    bool flag_calibration_start = false;
    bool flag_calibration_end = false;
    int flag_start_cal = 0;
    int flag_finish_cal = 0;

    bool correctChecksum(const std::vector<uint8_t> &ba);
    void readData();
    void timeoutSlot();
    void readyReadForTimer();

protected:
    uint8_t calculateCRC(const uint8_t data[], uint32_t length);
    void parseBuffer();

    std::vector<uint8_t> m_buffer;
    int baudRate = 115200;  // Изменено на 115200
    std::chrono::steady_clock::time_point last_receive_time;

public:
    LibSerial::SerialPort m_port;
};

// Прототипы функций преобразования BCD
float BCDToFloat(const uint8_t* buf);
float BCDToFloatAccel(const uint8_t* buf);
float BCDToFloatMagn(const uint8_t* buf);

#endif // AH127CPROTOCOL_H
