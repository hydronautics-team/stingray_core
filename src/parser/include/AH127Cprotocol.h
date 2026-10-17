#ifndef AH127CPROTOCOL_H
#define AH127CPROTOCOL_H

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include <libserial/SerialPort.h>

struct DataFromAH127C
{
    double yaw = 0.0;
    double pitch = 0.0;
    double roll = 0.0;

    // Линейное ускорение, м/с²
    double X_accel = 0.0;
    double Y_accel = 0.0;
    double Z_accel = 0.0;

    // Угловая скорость, град/с
    double X_rate = 0.0;
    double Y_rate = 0.0;
    double Z_rate = 0.0;

    // Магнитометр: единицы исходного протокола
    double X_magn = 0.0;
    double Y_magn = 0.0;
    double Z_magn = 0.0;
};

class AH127Cprotocol
{
public:
    AH127Cprotocol(
        const std::string &portName,
        int baudRate
    );

    ~AH127Cprotocol();

    AH127Cprotocol(const AH127Cprotocol &) = delete;
    AH127Cprotocol &operator=(const AH127Cprotocol &) = delete;

    // Возвращает true, если появился новый полный кадр.
    bool readData();

    // Контроль отсутствия новых измерений.
    void timeoutSlot();

    // Последнее корректное полное измерение.
    DataFromAH127C data;

    // Статистика.
    uint64_t valid_frames = 0;
    uint64_t checksum_errors = 0;
    uint64_t bcd_errors = 0;
    uint64_t discarded_bytes = 0;

    // Время получения последнего полного кадра.
    std::chrono::steady_clock::time_point
        last_sample_time() const
    {
        return last_sample_time_;
    }

private:
    void configureSensor();
    void sendCommand(
        uint8_t command,
        const std::vector<uint8_t> &payload = {}
    );

    bool parseBuffer();

    bool parseFullFrame(
        const uint8_t *frame,
        std::size_t size,
        DataFromAH127C &result
    );

    static uint8_t calculateChecksum(
        const uint8_t *data,
        std::size_t length
    );

    static bool decodeAngle(
        const uint8_t *buf,
        double &value
    );

    static bool decodeAcceleration(
        const uint8_t *buf,
        double &value
    );

    static bool decodeMagnetometer(
        const uint8_t *buf,
        double &value
    );

private:
    LibSerial::SerialPort m_port_;
    std::vector<uint8_t> m_buffer_;

    std::chrono::steady_clock::time_point
        last_sample_time_;

    std::chrono::steady_clock::time_point
        last_reconfigure_time_;

    bool have_sample_ = false;

    static constexpr std::size_t MAX_BUFFER_SIZE = 4096;

    // Настройка по твоему исходному протоколу:
    // LEN=56, полная длина=57 байт.
    static constexpr uint8_t FULL_FRAME_LENGTH = 56;
    static constexpr std::size_t FULL_FRAME_SIZE = 57;
};

#endif
