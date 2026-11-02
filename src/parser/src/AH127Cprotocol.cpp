#include "AH127Cprotocol.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>

namespace
{

constexpr double GRAVITY = 9.80665;

bool validBCD(uint8_t value)
{
    return value <= 9;
}


// ============================================================
// BCD: ANGLE / GYROSCOPE
// ============================================================

bool decodeBCD5(
    const uint8_t *buf,
    double &value
)
{
    const uint8_t sign = (buf[0] >> 4) & 0x0F;

    const uint8_t d2 = buf[0] & 0x0F;
    const uint8_t d1 = (buf[1] >> 4) & 0x0F;
    const uint8_t d0 = buf[1] & 0x0F;

    const uint8_t f1 = (buf[2] >> 4) & 0x0F;
    const uint8_t f2 = buf[2] & 0x0F;

    if (sign > 1 ||
        !validBCD(d2) ||
        !validBCD(d1) ||
        !validBCD(d0) ||
        !validBCD(f1) ||
        !validBCD(f2))
    {
        return false;
    }

    value =
        d2 * 100.0 +
        d1 * 10.0 +
        d0 +
        f1 * 0.1 +
        f2 * 0.01;

    if (sign == 1)
    {
        value = -value;
    }

    return true;
}

} // namespace


// ============================================================
// CONSTRUCTOR
// ============================================================

AH127Cprotocol::AH127Cprotocol(
    const std::string &portName,
    int baudRate
)
{
    if (baudRate != 115200)
    {
        throw std::invalid_argument(
            "AH127C supports 115200 in this driver"
        );
    }

    m_port_.Open(portName);

    m_port_.SetBaudRate(
        LibSerial::BaudRate::BAUD_115200
    );

    m_port_.SetCharacterSize(
        LibSerial::CharacterSize::CHAR_SIZE_8
    );

    m_port_.SetStopBits(
        LibSerial::StopBits::STOP_BITS_1
    );

    m_port_.SetParity(
        LibSerial::Parity::PARITY_NONE
    );

    const auto now = std::chrono::steady_clock::now();

    last_sample_time_ = now;
    last_reconfigure_time_ = now;

    std::cout
        << "[AH127C] Port opened: "
        << portName
        << " @ 115200"
        << std::endl;

    configureSensor();
}


// ============================================================
// DESTRUCTOR
// ============================================================

AH127Cprotocol::~AH127Cprotocol()
{
    if (m_port_.IsOpen())
    {
        m_port_.Close();
    }
}


// ============================================================
// CHECKSUM
// ============================================================

uint8_t AH127Cprotocol::calculateChecksum(
    const uint8_t *data,
    std::size_t length
)
{
    uint32_t sum = 0;

    for (std::size_t i = 0; i < length; ++i)
    {
        sum += data[i];
    }

    return static_cast<uint8_t>(sum & 0xFF);
}


// ============================================================
// COMMAND SENDING
// ============================================================

void AH127Cprotocol::sendCommand(
    uint8_t command,
    const std::vector<uint8_t> &payload
)
{
    if (payload.size() > 251)
    {
        throw std::invalid_argument(
            "AH127C command payload too large"
        );
    }

    std::vector<uint8_t> frame;

    frame.reserve(payload.size() + 5);

    // START
    frame.push_back(0x77);

    // LENGTH
    frame.push_back(
        static_cast<uint8_t>(payload.size() + 4)
    );

    // ADDRESS
    frame.push_back(0x00);

    // COMMAND
    frame.push_back(command);

    // DATA
    frame.insert(
        frame.end(),
        payload.begin(),
        payload.end()
    );

    // CHECKSUM
    const uint8_t checksum = calculateChecksum(
        frame.data() + 1,
        frame.size() - 1
    );

    frame.push_back(checksum);

    m_port_.Write(frame);
}


// ============================================================
// SENSOR CONFIGURATION
// ============================================================

void AH127Cprotocol::configureSensor()
{
    // Автовыдача всех данных.
    //
    // 77 05 00 56 05 60
    sendCommand(0x56, {0x05});

    // Частота 50 Гц.
    //
    // 77 05 00 0C 05 16
    sendCommand(0x0C, {0x05});

    // Настройки не сохраняются во flash при каждом
    // запуске, чтобы не выполнять лишние записи.
    //
    // Команда сохранения при необходимости:
    //
    // sendCommand(0x0A);
    //
    // Кадр: 77 04 00 0A 0E

    std::cout
        << "[AH127C] Autostream enabled, 50 Hz"
        << std::endl;
}


// ============================================================
// DECODING
// ============================================================

bool AH127Cprotocol::decodeAngle(
    const uint8_t *buf,
    double &value
)
{
    return decodeBCD5(buf, value);
}


bool AH127Cprotocol::decodeAcceleration(
    const uint8_t *buf,
    double &value
)
{
    const uint8_t sign = (buf[0] >> 4) & 0x0F;

    const uint8_t d0 = buf[0] & 0x0F;
    const uint8_t d1 = (buf[1] >> 4) & 0x0F;
    const uint8_t d2 = buf[1] & 0x0F;
    const uint8_t d3 = (buf[2] >> 4) & 0x0F;
    const uint8_t d4 = buf[2] & 0x0F;

    if (sign > 1 ||
        !validBCD(d0) ||
        !validBCD(d1) ||
        !validBCD(d2) ||
        !validBCD(d3) ||
        !validBCD(d4))
    {
        return false;
    }

    double accel_g =
        d0 +
        d1 * 0.1 +
        d2 * 0.01 +
        d3 * 0.001 +
        d4 * 0.0001;

    if (sign == 1)
    {
        accel_g = -accel_g;
    }

    value = accel_g * GRAVITY;

    return true;
}


bool AH127Cprotocol::decodeMagnetometer(
    const uint8_t *buf,
    double &value
)
{
    const uint8_t sign = (buf[0] >> 4) & 0x0F;

    const uint8_t d0 = buf[0] & 0x0F;
    const uint8_t d1 = (buf[1] >> 4) & 0x0F;
    const uint8_t d2 = buf[1] & 0x0F;
    const uint8_t d3 = (buf[2] >> 4) & 0x0F;
    const uint8_t d4 = buf[2] & 0x0F;

    if (sign > 1 ||
        !validBCD(d0) ||
        !validBCD(d1) ||
        !validBCD(d2) ||
        !validBCD(d3) ||
        !validBCD(d4))
    {
        return false;
    }

    value =
        d0 * 0.1 +
        d1 * 0.01 +
        d2 * 0.001 +
        d3 * 0.0001 +
        d4 * 0.00001;

    if (sign == 1)
    {
        value = -value;
    }

    return true;
}


// ============================================================
// FULL FRAME DECODING
// ============================================================

bool AH127Cprotocol::parseFullFrame(
    const uint8_t *frame,
    std::size_t size,
    DataFromAH127C &result
)
{
    if (size != FULL_FRAME_SIZE)
    {
        return false;
    }

    if (frame[0] != 0x77 ||
        frame[1] != FULL_FRAME_LENGTH ||
        frame[3] != 0x59)
    {
        return false;
    }

    DataFromAH127C decoded;

    // ========================================================
    // EULER
    // ========================================================

    if (!decodeAngle(frame + 4, decoded.pitch))
        return false;

    if (!decodeAngle(frame + 7, decoded.roll))
        return false;

    if (!decodeAngle(frame + 10, decoded.yaw))
        return false;

    if (std::abs(decoded.pitch) > 180.0 ||
        std::abs(decoded.roll) > 180.0 ||
        std::abs(decoded.yaw) > 360.0)
    {
        return false;
    }

    // ========================================================
    // ACCELEROMETER
    // ========================================================

    if (!decodeAcceleration(
            frame + 13,
            decoded.X_accel))
        return false;

    if (!decodeAcceleration(
            frame + 16,
            decoded.Y_accel))
        return false;

    if (!decodeAcceleration(
            frame + 19,
            decoded.Z_accel))
        return false;

    // ========================================================
    // GYROSCOPE
    // ========================================================

    if (!decodeAngle(
            frame + 25,
            decoded.X_rate))
        return false;

    if (!decodeAngle(
            frame + 22,
            decoded.Y_rate))
        return false;

    if (!decodeAngle(
            frame + 28,
            decoded.Z_rate))
        return false;

    // ========================================================
    // MAGNETOMETER
    // ========================================================

    if (!decodeMagnetometer(
            frame + 31,
            decoded.X_magn))
        return false;

    if (!decodeMagnetometer(
            frame + 34,
            decoded.Y_magn))
        return false;

    if (!decodeMagnetometer(
            frame + 37,
            decoded.Z_magn))
        return false;

    // ========================================================
    // ORIGINAL AXIS COMPENSATION
    // ========================================================
    //
    // Сохраняем знаки из твоего рабочего драйвера.

    decoded.X_accel = -decoded.X_accel;
    decoded.Z_rate = -decoded.Z_rate;

    // Проверка диапазона.
    if (std::abs(decoded.X_rate) > 400.0 ||
        std::abs(decoded.Y_rate) > 400.0 ||
        std::abs(decoded.Z_rate) > 400.0)
    {
        return false;
    }

    result = decoded;

    return true;
}


// ============================================================
// BUFFER PARSING
// ============================================================

bool AH127Cprotocol::parseBuffer()
{
    bool new_full_frame = false;

    std::size_t offset = 0;

    while (offset < m_buffer_.size())
    {
        // Поиск начала кадра.
        if (m_buffer_[offset] != 0x77)
        {
            ++offset;
            ++discarded_bytes;
            continue;
        }

        // Недостаточно байтов для LEN.
        if (m_buffer_.size() - offset < 2)
        {
            break;
        }

        const uint8_t len = m_buffer_[offset + 1];

        // Длина кадра по используемому протоколу.
        //
        // Ограничиваем ожидаемую длину,
        // чтобы случайный 0x77 не блокировал парсер.

        if (len < 4 || len > 64)
        {
            ++offset;
            ++discarded_bytes;
            continue;
        }

        const std::size_t frame_size =
            static_cast<std::size_t>(len) + 1;

        // Неполный кадр оставляем до следующего чтения.
        if (m_buffer_.size() - offset < frame_size)
        {
            break;
        }

        const uint8_t *frame =
            m_buffer_.data() + offset;

        const uint8_t expected_checksum =
            calculateChecksum(
                frame + 1,
                frame_size - 2
            );

        const uint8_t received_checksum =
            frame[frame_size - 1];

        if (expected_checksum != received_checksum)
        {
            ++checksum_errors;

            ++offset;
            ++discarded_bytes;

            continue;
        }

        const uint8_t command = frame[3];

        // Полный кадр измерений.
        if (command == 0x59 &&
            frame_size == FULL_FRAME_SIZE)
        {
            DataFromAH127C decoded;

            if (parseFullFrame(
                    frame,
                    frame_size,
                    decoded))
            {
                data = decoded;

                last_sample_time_ =
                    std::chrono::steady_clock::now();

                have_sample_ = true;
                new_full_frame = true;

                ++valid_frames;
            }
            else
            {
                ++bcd_errors;
            }
        }

        // Переходим к следующему кадру.
        offset += frame_size;
    }

    // Удаляем все обработанные байты.
    if (offset > 0)
    {
        m_buffer_.erase(
            m_buffer_.begin(),
            m_buffer_.begin() + offset
        );
    }

    if (m_buffer_.size() > MAX_BUFFER_SIZE)
    {
        discarded_bytes += m_buffer_.size();
        m_buffer_.clear();
    }

    return new_full_frame;
}


// ============================================================
// READING
// ============================================================

bool AH127Cprotocol::readData()
{
    constexpr std::size_t MAX_READ_BYTES = 4096;

    std::size_t count = 0;

    while (count < MAX_READ_BYTES &&
           m_port_.IsDataAvailable())
    {
        uint8_t byte = 0;

        m_port_.ReadByte(byte);

        m_buffer_.push_back(byte);

        ++count;
    }

    return parseBuffer();
}


// ============================================================
// TIMEOUT
// ============================================================

void AH127Cprotocol::timeoutSlot()
{
    const auto now =
        std::chrono::steady_clock::now();

    const auto last_activity =
        have_sample_
            ? last_sample_time_
            : last_reconfigure_time_;

    const auto elapsed =
        std::chrono::duration_cast<
            std::chrono::milliseconds
        >(now - last_activity).count();

    if (elapsed <= 3000)
    {
        return;
    }

    const auto since_reconfigure =
        std::chrono::duration_cast<
            std::chrono::milliseconds
        >(now - last_reconfigure_time_).count();

    if (since_reconfigure < 3000)
    {
        return;
    }

    std::cerr
        << "[AH127C] No valid full frames for "
        << elapsed
        << " ms. Reconfiguring..."
        << std::endl;

    configureSensor();

    last_reconfigure_time_ = now;
}
