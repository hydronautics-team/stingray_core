#include "AH127Cprotocol.h"
#include <stdint.h>
#include <vector>
#include <algorithm>
#include <iostream>

#define LOG_INFO(msg) std::cout << "[INFO]: " << msg << std::endl
#define LOG_ERR(msg) std::cerr << "[ERROR]: " << msg << std::endl


AH127Cprotocol::AH127Cprotocol(std::string portName, int baudRate)

{
    m_port.Open(portName);
    m_port.SetBaudRate(LibSerial::BaudRate::BAUD_9600);


    char cmd_1[6]; //задание формата посылки и частоты выдачи данных, 2.15 и 2.17
    cmd_1[0] = 0x77;
    cmd_1[1] = 0x05;
    cmd_1[2] = 0x00;
    cmd_1[3] = 0x56;
    cmd_1[4] = 0x05;
    cmd_1[5] = 0x60;

    std::vector<uint8_t> v_cmd1(cmd_1, cmd_1 + 6);
    m_port.Write(v_cmd1);
    std::cout << "cmd_1 отправлена успешно (0x56 0x05)" << std::endl;

    char cmd_2[6];
    cmd_2[0] = 0x77;
    cmd_2[1] = 0x05;
    cmd_2[2] = 0x00;
    cmd_2[3] = 0x0C;
    cmd_2[4] = 0x05;
    cmd_2[5] = 0x16;
    std::vector<uint8_t> v_cmd2(cmd_2, cmd_2 + 6);
    m_port.Write(v_cmd2);
    std::cout << "cmd_2 отправлена успешно (0x0C 0x05)" << std::endl;
}

uint8_t AH127Cprotocol::calculateCRC(const uint8_t data[], uint32_t length) {
     unsigned int i;
     unsigned short crc = 0;
     for(i=0; i<length; i++){
         crc += data[i];
     }
    return crc & 0xFF;
}

bool AH127Cprotocol::correctChecksum(const std::vector<uint8_t> &ba) {
    if (ba.size() < 56) return false;
    if (calculateCRC((unsigned char*)ba.data(), 55) == ba[55]) {
        return true;
    }
    return false;
}

void AH127Cprotocol::readData() {
    if (m_port.IsDataAvailable()) {
        uint8_t byte;
        while(m_port.IsDataAvailable()){
            m_port.ReadByte(byte); 
            m_buffer.push_back(byte);
        }
        readyReadForTimer();
        parseBuffer();
    }
}

void AH127Cprotocol::readyReadForTimer() {
    last_receive_time = std::chrono::steady_clock::now();
}

void AH127Cprotocol::timeoutSlot(){
auto now = std::chrono::steady_clock::now();
auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_receive_time).count();
 double deltaTMax = 2000;
    if (elapsed>deltaTMax) {
        uint8_t cmd_1[6]; //задание формата посылки, 2.15
        cmd_1[0] = 0x77;
        cmd_1[1] = 0x05;
        cmd_1[2] = 0x00;
        cmd_1[3] = 0x0C;
        cmd_1[4] = 0x05;
        cmd_1[5] = 0x16;
        std::vector<uint8_t> v_cmd1(cmd_1, cmd_1 + 6);
        m_port.Write(v_cmd1);
        

        uint8_t cmd_2[6]; //задание частоты выдачи данных, 2.17
        cmd_2[0] = 0x77;
        cmd_2[1] = 0x05;
        cmd_2[2] = 0x00;
        cmd_2[3] = 0x56;
        cmd_2[4] = 0x05;
        cmd_2[5] = 0x60;
        std::vector<uint8_t> v_cmd2(cmd_2, cmd_2 + 6);
        m_port.Write(v_cmd2);

        readyReadForTimer();
    }
}

float ThreeBytesToFloat(const uint8_t* buf) {
    float result = 0.0f;
    result += int(buf[0] & 0x0F) * 100;
    result += int((buf[1] & 0xF0) >> 4) * 10;
    result += int(buf[1] & 0x0F);
    result += int((buf[2] & 0xF0) >> 4) * 0.1;
    result += int(buf[2] & 0x0F) * 0.01;
    result *= (buf[0] & 0xF0) ? (-1) : (1);
    return result;
}

float ThreeBytesToFloatAccel(const uint8_t* buf) {
    float result = 0.0;
    result += int(buf[0] & 0x0F);
    result += int((buf[1] & 0xF0) >> 4) * 0.1;
    result += int(buf[1] & 0x0F)*0.01;
    result += int((buf[2] & 0xF0) >> 4) * 0.001;
    result += int(buf[2] & 0x0F) * 0.0001;
    result *= (buf[0] & 0xF0) ? (-1) : (1);
    return result*9.81;
}

float ThreeBytesToFloatMagn(const uint8_t* buf) {
    float result = 0.0;
    result += int(buf[0] & 0x0F)*0.1;
    result += int((buf[1] & 0xF0) >> 4) * 0.01;
    result += int(buf[1] & 0x0F)*0.001;
    result += int((buf[2] & 0xF0) >> 4) * 0.0001;
    result += int(buf[2] & 0x0F) * 0.00001;
    result *= (buf[0] & 0xF0) ? (-1) : (1);
    return result;
}

float FourBytesToFloatQvat(const uint8_t* buf) {
    float result = 0.0;
    result += int(buf[0] & 0x0F);
    result += int((buf[1] & 0xF0) >> 4) * 0.1;
    result += int(buf[1] & 0x0F)*0.01;
    result += int((buf[2] & 0xF0) >> 4) * 0.001;
    result += int(buf[2] & 0x0F) * 0.0001;
    result += int(buf[3] & 0x0F) * 0.00001;
    result += int(buf[3] & 0x0F) * 0.000001;
    result *= (buf[0] & 0xF0) ? (-1) : (1);
    return result;
}

void PrintMsg(DataFromAH127C const& msg) {
    std::cout << "yaw: " << msg.yaw << std::endl;
    std::cout << "pitch: " << msg.pitch << std::endl;
    std::cout << "roll: " << msg.roll << std::endl;
    std::cout << "X_accel: " << msg.X_accel << std::endl;
    std::cout << "Y_accel: " << msg.Y_accel << std::endl;
    std::cout << "Z_accel: " << msg.Z_accel << std::endl;
    std::cout << "X_rate: " << msg.X_rate << std::endl;
    std::cout << "Y_rate: " << msg.Y_rate << std::endl;
    std::cout << "Z_rate: " << msg.Z_rate << std::endl;
    std::cout << "X_magn: " << msg.X_magn << std::endl;
    std::cout << "Y_magn: " << msg.Y_magn << std::endl;
    std::cout << "Z_magn: " << msg.Z_magn << std::endl;
    std::cout << "first_qvat " << msg.first_qvat << std::endl;
    std::cout << "second_qvat " << msg.second_qvat << std::endl;
    std::cout << "third_qvat " << msg.third_qvat << std::endl;
    std::cout << "four_qvat " << msg.four_qvat << std::endl;
}

void AH127Cprotocol::parseBuffer() {
    if (m_buffer.size() <= 4 ) {
        return;
    }

    uint8_t* cal_start_ptr = reinterpret_cast<uint8_t*>(&calibr_start);

    if (flag_start_cal == 1) {
        auto it = std::search(m_buffer.begin(), m_buffer.end(), 
                              cal_start_ptr, cal_start_ptr + sizeof(Header_AH_calibration_start));

        if (it == m_buffer.end()) {
            std::cout << "команда начала калибровки не распознана" << std::endl;
            return; 
        } else {
            flag_calibration_start = 1;
            flag_calibration_end = 0;
            std::cout << "команда калибровки дошла до датчика, можно начинать калибровку" << std::endl;
        }
        flag_start_cal = 0;
    }

    if (flag_finish_cal == 1) {
        uint8_t* cal_end_ptr = reinterpret_cast<uint8_t*>(&calibr_end);
        auto it_cal_end = std::search(m_buffer.begin(), m_buffer.end(), 
                                     cal_end_ptr, cal_end_ptr + sizeof(Header_AH_calibration_end));

        if (it_cal_end == m_buffer.end()) {
            std::cout << "команда окончания калибровки не распознана" << std::endl;
            return;
        } else {
            flag_calibration_start = 0;
            flag_calibration_end = 1;
            std::cout << "команда окончания калибровки дошла до датчика, результат калибровки записан" << std::endl;

            uint8_t cmd_1[6] = {0x77, 0x05, 0x00, 0x0C, 0x05, 0x16};
            std::vector<uint8_t> v_cmd1(cmd_1, cmd_1 + 6);
            m_port.Write(v_cmd1);

            uint8_t cmd_2[6] = {0x77, 0x05, 0x00, 0x56, 0x05, 0x60};
            std::vector<uint8_t> v_cmd2(cmd_2, cmd_2 + 6);
            m_port.Write(v_cmd2);
        }
        flag_finish_cal = 0;
    }

    uint8_t* h_ptr = reinterpret_cast<uint8_t*>(&data.header);
    auto it_header = std::search(m_buffer.begin(), m_buffer.end(), 
                                 h_ptr, h_ptr + sizeof(Header_AH));

    if (it_header == m_buffer.end()) {
        std::cout << "no message" << std::endl;
        return;
    }

    if (it_header != m_buffer.begin()) {
        m_buffer.erase(m_buffer.begin(), it_header);
    }

    if (m_buffer.size() < 57) {
        return;
    }

    std::vector<uint8_t> packetData(m_buffer.begin() + 1, m_buffer.begin() + 57);

    if (correctChecksum(packetData)) {
        DataFromAH127C msg;
        const uint8_t* ptr = m_buffer.data();

        msg.pitch   = ThreeBytesToFloat(ptr + 4);
        msg.roll    = ThreeBytesToFloat(ptr + 7);
        msg.yaw     = ThreeBytesToFloat(ptr + 10);
        msg.X_accel = ThreeBytesToFloatAccel(ptr + 13);
        msg.Y_accel = ThreeBytesToFloatAccel(ptr + 16);
        msg.Z_accel = ThreeBytesToFloatAccel(ptr + 19);
        msg.X_rate  = ThreeBytesToFloat(ptr + 22);
        msg.Y_rate  = ThreeBytesToFloat(ptr + 25);
        msg.Z_rate  = ThreeBytesToFloat(ptr + 28);
        msg.X_magn  = ThreeBytesToFloatMagn(ptr + 31);
        msg.Y_magn  = ThreeBytesToFloatMagn(ptr + 34);
        msg.Z_magn  = ThreeBytesToFloatMagn(ptr + 37);
        msg.first_qvat  = FourBytesToFloatQvat(ptr + 40);
        msg.second_qvat = FourBytesToFloatQvat(ptr + 44);
        msg.third_qvat  = FourBytesToFloatQvat(ptr + 48);
        msg.four_qvat   = FourBytesToFloatQvat(ptr + 52);
        
        data = msg;
        m_buffer.erase(m_buffer.begin(), m_buffer.begin() + 57);
    } else {
        m_buffer.erase(m_buffer.begin());
    }
}