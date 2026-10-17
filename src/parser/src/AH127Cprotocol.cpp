#include "AH127Cprotocol.h"

#include <algorithm>
#include <cstdio>
#include <iostream>
#include <stdexcept>

namespace {

LibSerial::BaudRate toBaud(int baud) {
    switch (baud) {
        case 2400:   return LibSerial::BaudRate::BAUD_2400;
        case 4800:   return LibSerial::BaudRate::BAUD_4800;
        case 9600:   return LibSerial::BaudRate::BAUD_9600;
        case 19200:  return LibSerial::BaudRate::BAUD_19200;
        case 38400:  return LibSerial::BaudRate::BAUD_38400;
        case 57600:  return LibSerial::BaudRate::BAUD_57600;
        case 115200: return LibSerial::BaudRate::BAUD_115200;
        default:
            throw std::invalid_argument("Unsupported baud rate: " + std::to_string(baud));
    }
}

// Код частоты автовыдачи для команды 0x0C (по мануалу Bewis).
int rateToCode(int hz) {
    switch (hz) {
        case 0:  return 0;   // режим "по запросу"
        case 5:  return 1;
        case 10: return 2;
        case 20: return 3;
        case 25: return 4;
        case 50: return 5;
        default: return -1;
    }
}

void hexdump(const char* tag, const uint8_t* p, size_t n) {
    std::printf("%s", tag);
    for (size_t i = 0; i < n; i++) std::printf("%02X ", p[i]);
    std::printf("\n");
}

} // namespace

AH127Cprotocol::AH127Cprotocol() : last_rx_(std::chrono::steady_clock::now()) {}

AH127Cprotocol::AH127Cprotocol(const std::string& portName, int baudRate)
    : last_rx_(std::chrono::steady_clock::now())
{
    port_.Open(portName);
    port_.SetBaudRate(toBaud(baudRate));
    port_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
    port_.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
    port_.SetParity(LibSerial::Parity::PARITY_NONE);
    port_.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);
    port_open_ = true;
    port_.FlushIOBuffers();
}

AH127Cprotocol::~AH127Cprotocol() {
    if (port_open_ && port_.IsOpen()) port_.Close();
}

// ---------- BCD ----------

static inline int hi(uint8_t b) { return (b >> 4) & 0x0F; }
static inline int lo(uint8_t b) { return b & 0x0F; }
static inline float sign(uint8_t b) { return (b & 0xF0) ? -1.0f : 1.0f; }

float AH127Cprotocol::bcdAngle(const uint8_t* b) {
    float v = lo(b[0]) * 100.0f + hi(b[1]) * 10.0f + lo(b[1])
            + hi(b[2]) * 0.1f + lo(b[2]) * 0.01f;
    return sign(b[0]) * v;
}

float AH127Cprotocol::bcdAccel(const uint8_t* b) {
    float v = lo(b[0]) + hi(b[1]) * 0.1f + lo(b[1]) * 0.01f
            + hi(b[2]) * 0.001f + lo(b[2]) * 0.0001f;
    return sign(b[0]) * v;
}

float AH127Cprotocol::bcdMagn(const uint8_t* b) {
    float v = lo(b[0]) * 0.1f + hi(b[1]) * 0.01f + lo(b[1]) * 0.001f
            + hi(b[2]) * 0.0001f + lo(b[2]) * 0.00001f;
    return sign(b[0]) * v;
}

float AH127Cprotocol::bcdQuat(const uint8_t* b) {
    float v = lo(b[0]) + hi(b[1]) * 0.1f + lo(b[1]) * 0.01f
            + hi(b[2]) * 0.001f + lo(b[2]) * 0.0001f
            + hi(b[3]) * 0.00001f + lo(b[3]) * 0.000001f;
    return sign(b[0]) * v;
}

// ---------- кадр ----------

uint8_t AH127Cprotocol::checksum(const uint8_t* frame, size_t frameLen) {
    // Сумма от байта длины до последнего байта данных (0x77 не входит), младший байт.
    unsigned sum = 0;
    for (size_t i = 1; i + 1 < frameLen; i++) sum += frame[i];
    return static_cast<uint8_t>(sum & 0xFF);
}

bool AH127Cprotocol::decodeFrame(const uint8_t* f, ImuFrame& out) {
    if (f[0] != 0x77 || f[1] != 0x38 || f[3] != CMD_FULL) return false;
    if (checksum(f, FRAME_LEN) != f[FRAME_LEN - 1]) return false;

    // Порядок по мануалу: pitch, roll, heading; ax, ay, az; gx, gy, gz; mx, my, mz; q0..q3.
    out.pitch = bcdAngle(f + 4);
    out.roll  = bcdAngle(f + 7);
    out.yaw   = bcdAngle(f + 10);
    out.ax = bcdAccel(f + 13);
    out.ay = bcdAccel(f + 16);
    out.az = bcdAccel(f + 19);
    out.gx = bcdAngle(f + 22);
    out.gy = bcdAngle(f + 25);
    out.gz = bcdAngle(f + 28);
    out.mx = bcdMagn(f + 31);
    out.my = bcdMagn(f + 34);
    out.mz = bcdMagn(f + 37);
    out.q0 = bcdQuat(f + 40);
    out.q1 = bcdQuat(f + 44);
    out.q2 = bcdQuat(f + 48);
    out.q3 = bcdQuat(f + 52);
    return true;
}

int AH127Cprotocol::feed(const uint8_t* data, size_t n) {
    if (n == 0) return 0;
    buf_.insert(buf_.end(), data, data + n);
    last_rx_ = std::chrono::steady_clock::now();
    if (debug_) hexdump("[RX] ", data, n);

    int parsed = 0;
    size_t pos = 0;
    while (buf_.size() - pos >= FRAME_LEN) {
        // Ищем заголовок 77 38 00 59.
        if (!(buf_[pos] == 0x77 && buf_[pos + 1] == 0x38 && buf_[pos + 2] == 0x00 && buf_[pos + 3] == CMD_FULL)) {
            pos++;
            bytes_dropped_++;
            continue;
        }
        ImuFrame fr;
        if (decodeFrame(&buf_[pos], fr)) {
            latest_ = fr;
            frames_ok_++;
            parsed++;
            pos += FRAME_LEN;
        } else {
            crc_errors_++;
            if (debug_) hexdump("[CRC FAIL] ", &buf_[pos], FRAME_LEN);
            pos++;
            bytes_dropped_++;
        }
    }
    // Хвост (возможно неполный кадр) оставляем, всё разобранное/мусор выбрасываем.
    if (pos > 0) buf_.erase(buf_.begin(), buf_.begin() + pos);
    // Хвост без заголовка не нужен, кроме последних 3 байт (там может начинаться заголовок).
    if (buf_.size() >= 4 && !(buf_[0] == 0x77 && buf_[1] == 0x38)) {
        auto it = std::find(buf_.begin(), buf_.end(), 0x77);
        size_t drop = (it == buf_.end()) ? buf_.size() : static_cast<size_t>(it - buf_.begin());
        bytes_dropped_ += drop;
        buf_.erase(buf_.begin(), buf_.begin() + drop);
    }
    if (buf_.size() > 4096) { bytes_dropped_ += buf_.size(); buf_.clear(); }
    return parsed;
}

int AH127Cprotocol::poll() {
    if (!port_open_) return 0;
    uint8_t chunk[512];
    size_t n = 0;
    while (port_.IsDataAvailable() && n < sizeof(chunk)) {
        port_.ReadByte(chunk[n], 0);
        n++;
    }
    return feed(chunk, n);
}

// ---------- команды ----------

void AH127Cprotocol::sendCmd(const std::vector<uint8_t>& cmd) {
    if (!port_open_) return;
    if (debug_) hexdump("[TX] ", cmd.data(), cmd.size());
    port_.Write(cmd);
    port_.DrainWriteBuffer();
}

std::vector<uint8_t> AH127Cprotocol::makeCmd(uint8_t cmd, const std::vector<uint8_t>& data) {
    std::vector<uint8_t> f;
    f.push_back(0x77);
    f.push_back(static_cast<uint8_t>(4 + data.size()));   // len: len+addr+cmd+data+chk
    f.push_back(0x00);
    f.push_back(cmd);
    f.insert(f.end(), data.begin(), data.end());
    unsigned s = 0;
    for (size_t i = 1; i < f.size(); i++) s += f[i];
    f.push_back(static_cast<uint8_t>(s & 0xFF));
    return f;
}

void AH127Cprotocol::sendConfig() {
    sendCmd(makeCmd(0x56, {0x05}));                               // автовыдача: всё (углы+g+ω+mag+quat)
    sendCmd(makeCmd(0x0C, {static_cast<uint8_t>(rate_code_)}));  // частота автовыдачи
    if (save_settings_) sendCmd(makeCmd(0x0A));                   // сохранить настройки
}

bool AH127Cprotocol::configure(int rateHz, bool save) {
    int code = rateToCode(rateHz);
    if (code < 0) return false;
    rate_code_ = code;
    save_settings_ = save;
    sendConfig();
    last_rx_ = std::chrono::steady_clock::now();
    return true;
}

void AH127Cprotocol::requestFrame() {
    sendCmd(makeCmd(CMD_FULL));   // 77 04 00 59 5D
}

bool AH127Cprotocol::checkTimeout(int silenceMs) {
    auto now = std::chrono::steady_clock::now();
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_rx_).count();
    if (ms > silenceMs) {
        sendConfig();
        last_rx_ = now;
        return true;
    }
    return false;
}
