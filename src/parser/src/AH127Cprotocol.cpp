#include "AH127Cprotocol.h"
#include <stdint.h>
#include <vector>
#include <algorithm>
#include <iostream>
#include <chrono>
#include <iomanip>

#define LOG_INFO(msg) std::cout << "[INFO]: " << msg << std::endl
#define LOG_ERR(msg) std::cerr << "[ERROR]: " << msg << std::endl
#define LOG_WARN(msg) std::cerr << "[WARN]: " << msg << std::endl

// Протокол BEWIS (LC-AH127C)
// Скорость: 115200 бод
// Команды:
// 0x04 - запрос трех углов (ответ 14 байт)
// 0x59 - запрос полного кадра (ответ 57 байт)
// 0x1F - запрос адреса (ответ 6 байт)
// 0x56 - включить автовыдачу (параметр 0x05 = все данные)
// 0x0C - установить частоту (0=0Гц, 1=5Гц, 2=10Гц, 3=20Гц, 4=25Гц, 5=50Гц)
// 0x0A - сохранить настройки

// Формат кадра: 0x77 LEN ADDR CMD DATA... CHK
// LEN = длина от ADDR до CHK включительно
// CHK = сумма всех байт от LEN до последнего байта данных & 0xFF

AH127Cprotocol::AH127Cprotocol(std::string portName, int baudRate)
{
    try {
        m_port.Open(portName);
        m_port.SetBaudRate(LibSerial::BaudRate::BAUD_115200);  // ВАЖНО: 115200!
        m_port.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
        m_port.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
        m_port.SetParity(LibSerial::Parity::PARITY_NONE);

        LOG_INFO("Порт " + portName + " открыт, скорость 115200");

        // Инициализация структур для калибровки
        calibr_start.identif = 0x77;
        calibr_start.lenght = 0x05;
        calibr_start.adress = 0x00;
        calibr_start.command = 0x91;
        calibr_start.datafield = 0x00;
        calibr_start.checksum = 0x96;

        calibr_end.identif = 0x77;
        calibr_end.lenght = 0x05;
        calibr_end.adress = 0x00;
        calibr_end.command = 0x92;
        calibr_end.datafield = 0x00;
        calibr_end.checksum = 0x97;

        // ========================================================
        // НАСТРОЙКА ДАТЧИКА НА МАКСИМАЛЬНУЮ ЧАСТОТУ 50 Гц
        // ========================================================

        // 1. Включить автовыдачу (0x56) с параметром 0x05 - все данные
        // Кадр: 77 05 00 56 05 60 (сумма 05+00+56+05=60)
        uint8_t cmd_auto[6] = {0x77, 0x05, 0x00, 0x56, 0x05, 0x60};
        std::vector<uint8_t> v_cmd_auto(cmd_auto, cmd_auto + 6);
        m_port.Write(v_cmd_auto);
        LOG_INFO("Автовыдача включена (0x56 0x05)");

        // 2. Частота 50 Гц (0x0C) - код 5 = 50 Гц (МАКСИМУМ!)
        // Кадр: 77 05 00 0C 05 16 (сумма 05+00+0C+05=16)
        uint8_t cmd_freq[6] = {0x77, 0x05, 0x00, 0x0C, 0x05, 0x16};
        std::vector<uint8_t> v_cmd_freq(cmd_freq, cmd_freq + 6);
        m_port.Write(v_cmd_freq);
        LOG_INFO("Частота 50 Гц установлена (0x0C 0x05) - МАКСИМУМ!");

        // 3. Сохранить настройки (0x0A) - без данных
        // Кадр: 77 03 00 0A (сумма 03+00+0A=0D)
        uint8_t cmd_save[4] = {0x77, 0x03, 0x00, 0x0A};
        std::vector<uint8_t> v_cmd_save(cmd_save, cmd_save + 4);
        m_port.Write(v_cmd_save);
        LOG_INFO("Настройки сохранены (0x0A)");

        last_receive_time = std::chrono::steady_clock::now();

    } catch (const std::exception& e) {
        LOG_ERR("Ошибка открытия порта: " + std::string(e.what()));
        throw;
    }
}

uint8_t AH127Cprotocol::calculateCRC(const uint8_t data[], uint32_t length) {
    unsigned short crc = 0;
    for (uint32_t i = 0; i < length; i++) {
        crc += data[i];
    }
    return crc & 0xFF;
}

bool AH127Cprotocol::correctChecksum(const std::vector<uint8_t> &ba) {
    if (ba.size() < 4) return false;
    // Сумма всех байт от LEN до последнего байта данных == последнему байту
    return calculateCRC(ba.data() + 1, ba.size() - 2) == ba[ba.size() - 1];
}

// Функции преобразования BCD (из диагностического скрипта)
float BCDToFloat(const uint8_t* buf) {
    float result = 0.0;
    result += int(buf[0] & 0x0F) * 100;
    result += int((buf[1] & 0xF0) >> 4) * 10;
    result += int(buf[1] & 0x0F);
    result += int((buf[2] & 0xF0) >> 4) * 0.1;
    result += int(buf[2] & 0x0F) * 0.01;
    if (buf[0] & 0xF0) result = -result;
    return result;
}

float BCDToFloatAccel(const uint8_t* buf) {
    float result = 0.0;
    result += int(buf[0] & 0x0F);
    result += int((buf[1] & 0xF0) >> 4) * 0.1;
    result += int(buf[1] & 0x0F) * 0.01;
    result += int((buf[2] & 0xF0) >> 4) * 0.001;
    result += int(buf[2] & 0x0F) * 0.0001;
    if (buf[0] & 0xF0) result = -result;
    return result * 9.81;  // g -> м/с²
}

float BCDToFloatMagn(const uint8_t* buf) {
    float result = 0.0;
    result += int(buf[0] & 0x0F) * 0.1;
    result += int((buf[1] & 0xF0) >> 4) * 0.01;
    result += int(buf[1] & 0x0F) * 0.001;
    result += int((buf[2] & 0xF0) >> 4) * 0.0001;
    result += int(buf[2] & 0x0F) * 0.00001;
    if (buf[0] & 0xF0) result = -result;
    return result;
}

void AH127Cprotocol::readData() {
    if (m_port.IsDataAvailable()) {
        uint8_t byte;
        int count = 0;
        while (m_port.IsDataAvailable()) {
            m_port.ReadByte(byte);
            m_buffer.push_back(byte);
            count++;
        }

        if (count > 0) {
            if (m_buffer.size() >= 16) {
                std::cout << "[DEBUG] Last 16 bytes: ";
                for (size_t i = m_buffer.size() - 16; i < m_buffer.size(); i++) {
                    printf("%02X ", m_buffer[i]);
                }
                std::cout << std::endl;
            }
        }

        readyReadForTimer();
        parseBuffer();
    }
}

void AH127Cprotocol::readyReadForTimer() {
    last_receive_time = std::chrono::steady_clock::now();
}

void AH127Cprotocol::timeoutSlot() {
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_receive_time).count();

    if (elapsed > 3000) {
        LOG_WARN("Таймаут приема данных, переотправка команд");

        // Переотправка команд (50 Гц)
        uint8_t cmd_auto[6] = {0x77, 0x05, 0x00, 0x56, 0x05, 0x60};
        std::vector<uint8_t> v_cmd_auto(cmd_auto, cmd_auto + 6);
        m_port.Write(v_cmd_auto);

        // Частота 50 Гц (МАКСИМУМ!)
        uint8_t cmd_freq[6] = {0x77, 0x05, 0x00, 0x0C, 0x05, 0x16};
        std::vector<uint8_t> v_cmd_freq(cmd_freq, cmd_freq + 6);
        m_port.Write(v_cmd_freq);

        readyReadForTimer();
    }
}

void AH127Cprotocol::parseBuffer() {
    if (m_buffer.size() <= 4) return;

    // Ищем кадр: 0x77 LEN ADDR CMD ... CHK
    size_t i = 0;
    while (i + 4 < m_buffer.size()) {
        if (m_buffer[i] == 0x77) {
            uint8_t len = m_buffer[i + 1];
            // len = количество байт от ADDR до CHK включительно
            // общая длина кадра = 1 (0x77) + len
            if (i + len + 1 <= m_buffer.size()) {
                std::vector<uint8_t> frame(m_buffer.begin() + i, m_buffer.begin() + i + len + 1);
                if (correctChecksum(frame)) {
                    // Нашли корректный кадр!
                    uint8_t cmd = frame[3];

                    // Полный кадр 0x59 (57 байт: 0x77 + 56 байт данных)
                    if (cmd == 0x59 && len == 56) {
                        const uint8_t* ptr = frame.data();

                        data.pitch = BCDToFloat(ptr + 4);
                        data.roll = BCDToFloat(ptr + 7);
                        data.yaw = BCDToFloat(ptr + 10);
                        data.X_accel = -BCDToFloatAccel(ptr + 13);
                        data.Y_accel = BCDToFloatAccel(ptr + 16);
                        data.Z_accel = BCDToFloatAccel(ptr + 19);
                        data.X_rate = BCDToFloat(ptr + 25);
                        data.Y_rate = BCDToFloat(ptr + 22);
                        data.Z_rate = -BCDToFloat(ptr + 28);
                        data.X_magn = BCDToFloatMagn(ptr + 31);
                        data.Y_magn = BCDToFloatMagn(ptr + 34);
                        data.Z_magn = BCDToFloatMagn(ptr + 37);
                        // Кватернионы пока не парсим
                        data.first_qvat = 0;
                        data.second_qvat = 0;
                        data.third_qvat = 0;
                        data.four_qvat = 0;

                        m_buffer.erase(m_buffer.begin() + i, m_buffer.begin() + i + len + 1);
                        return;
                    }
                    // Три угла 0x84 (14 байт: 0x77 + 13 байт данных)
                    else if (cmd == 0x84 && len == 13) {
                        const uint8_t* ptr = frame.data();
                        data.pitch = BCDToFloat(ptr + 4);
                        data.roll = BCDToFloat(ptr + 7);
                        data.yaw = BCDToFloat(ptr + 10);
                        m_buffer.erase(m_buffer.begin() + i, m_buffer.begin() + i + len + 1);
                        return;
                    }
                }
            }
        }
        i++;
    }

    // Если буфер слишком большой - очищаем
    if (m_buffer.size() > 1024) {
        m_buffer.clear();
    }
}
