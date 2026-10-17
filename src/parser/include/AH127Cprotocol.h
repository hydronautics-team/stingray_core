#ifndef AH127CPROTOCOL_H
#define AH127CPROTOCOL_H

#include <stdint.h>
#include <string>
#include <vector>
#include <chrono>
#include <cstddef>
#include <libserial/SerialPort.h>

// Один разобранный кадр "77 38 00 59" (команда 0x59: углы + g + угл. скорости + магнитометр + кватернион).
// Единицы: как в датчике - градусы, g, град/с, Гаусс. Кватернион q0 = скаляр (w), q1..q3 = x,y,z.
struct ImuFrame {
    float pitch = 0.0f, roll = 0.0f, yaw = 0.0f;   // град
    float ax = 0.0f, ay = 0.0f, az = 0.0f;         // g
    float gx = 0.0f, gy = 0.0f, gz = 0.0f;         // град/с
    float mx = 0.0f, my = 0.0f, mz = 0.0f;         // Гаусс
    float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
};

class AH127Cprotocol {
public:
    static constexpr size_t FRAME_LEN = 57;        // 0x77 + 0x38 (56 байт от длины до checksum включительно)
    static constexpr uint8_t CMD_FULL = 0x59;

    // Открывает порт. baud: 2400/4800/9600/19200/38400/57600/115200. Бросает std::exception при ошибке.
    AH127Cprotocol(const std::string& portName, int baudRate);
    // Конструктор без порта - для тестов через feed().
    AH127Cprotocol();
    ~AH127Cprotocol();

    // Настроить датчик: тип автовыдачи = всё (0x56 05), частота (0x0C), при save - сохранить (0x0A).
    // rateHz: 0 (по запросу), 5, 10, 20, 25, 50. Возвращает false, если частота не поддерживается.
    bool configure(int rateHz, bool save);
    // Разовый запрос полного кадра (режим "по запросу"): 77 04 00 59 5D.
    void requestFrame();
    // Прочитать всё, что есть в порту, и разобрать. Возвращает число полных корректных кадров.
    int poll();
    // Подать байты вручную (тесты/эмуляция). Возвращает число полных корректных кадров.
    int feed(const uint8_t* data, size_t n);
    // Переотправить конфигурацию, если тишина дольше silenceMs. Возвращает true, если переотправили.
    bool checkTimeout(int silenceMs);

    const ImuFrame& latest() const { return latest_; }
    bool hasFrame() const { return frames_ok_ > 0; }

    uint64_t framesOk() const { return frames_ok_; }
    uint64_t crcErrors() const { return crc_errors_; }
    uint64_t bytesDropped() const { return bytes_dropped_; }

    void setDebug(bool on) { debug_ = on; }

    // Публичные для тестов.
    static uint8_t checksum(const uint8_t* frame, size_t frameLen); // сумма frame[1..frameLen-2]
    static bool decodeFrame(const uint8_t* frame, ImuFrame& out);   // frame = 57 байт, начиная с 0x77
    static float bcdAngle(const uint8_t* b);   // SXXX.YY
    static float bcdAccel(const uint8_t* b);   // SX.YYYY
    static float bcdMagn(const uint8_t* b);    // S.YYYYY
    static float bcdQuat(const uint8_t* b);    // SX.YYYYYY
    // Собрать команду 77 LEN 00 CMD DATA CHK.
    static std::vector<uint8_t> makeCmd(uint8_t cmd, const std::vector<uint8_t>& data = {});

private:
    void sendCmd(const std::vector<uint8_t>& cmd);
    void sendConfig();

    LibSerial::SerialPort port_;
    bool port_open_ = false;
    std::vector<uint8_t> buf_;
    ImuFrame latest_;
    uint64_t frames_ok_ = 0, crc_errors_ = 0, bytes_dropped_ = 0;
    int rate_code_ = 2;          // 10 Гц по умолчанию
    bool save_settings_ = false;
    bool debug_ = false;
    std::chrono::steady_clock::time_point last_rx_;
};

#endif // AH127CPROTOCOL_H
