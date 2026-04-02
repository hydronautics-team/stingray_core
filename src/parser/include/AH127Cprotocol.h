#ifndef AH127CPROTOCOL_H
#define AH127CPROTOCOL_H

#include <stdint.h>
#include <string>
#include <vector>
#include <chrono>
#include <libserial/SerialPort.h>

//класс протокола
#pragma pack(push,1)
//заглушка для заголовка послыки
struct Header_AH {
    uint8_t identif = 0x77;
    uint8_t lenght = 0x38;
    uint8_t adress = 0x00;
    uint8_t command = 0x59;
};
//структура, которая приходит от датчика
struct DataFromAH127C {
    Header_AH header;

    float yaw;
    float pitch=0;
    float roll=0;
    float X_accel=0;
    float Y_accel=0;
    float Z_accel=0;
    float X_rate=0;
    float Y_rate=0;
    float Z_rate=0;
    float X_magn=0;
    float Y_magn=0;
    float Z_magn=0;
    float first_qvat=0;
    float second_qvat=0;
    float third_qvat=0;
    float four_qvat=0;
    float crc;
};

//заглушка для ответа от команды начала калибровки
struct Header_AH_calibration_start {
    uint8_t identif = 0x77;
    uint8_t lenght = 0x05;
    uint8_t adress = 0x00;
    uint8_t command = 0x91;
    uint8_t datafield = 0x00;
    uint8_t checksum = 0x96;
};

//заглушка для ответа от команды начала калибровки
struct Header_AH_calibration_end {
    uint8_t identif = 0x77;
    uint8_t lenght = 0x05;
    uint8_t adress = 0x00;
    uint8_t command = 0x92;
    uint8_t datafield = 0x00;
    uint8_t checksum = 0x97;
};
#pragma pack(pop)

class AH127Cprotocol 
{
public:
    AH127Cprotocol(std::string portName, int baudRate);
    DataFromAH127C data;//выходная структура
    Header_AH_calibration_start calibr_start;
    Header_AH_calibration_end calibr_end;
    bool flag_calibration_start = false;
    bool flag_calibration_end = false;
    int flag_start_cal = 0;
    int flag_finish_cal = 0;
    bool correctChecksum (const std::vector<uint8_t> &ba);
    void readData(); //слот, который будет вызываться в ответ на readyRead
    void timeoutSlot();
    void readyReadForTimer();

protected:
    uint8_t calculateCRC(const uint8_t data[], uint32_t length);
    void parseBuffer();
    std::vector<uint8_t> m_buffer;
    int baudRate = 115200; //бодрейт
    std::chrono::steady_clock::time_point last_receive_time;
public:
    LibSerial::SerialPort m_port;
};

#endif // AH127CPROTOCOL_H
