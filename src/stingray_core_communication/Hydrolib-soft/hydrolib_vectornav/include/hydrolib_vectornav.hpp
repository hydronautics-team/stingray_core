#pragma once

#include <concepts>
#include <cstddef>
#include <cstdint>
#include <cstring>

#include "hydrolib_imu.hpp"
#include "hydrolib_logger.hpp"
#include "hydrolib_return_codes.hpp"
#include "hydrolib_stream_concepts.hpp"

namespace hydrolib
{
template <concepts::stream::ByteFullStreamConcept InputStream, typename Logger>
class VectorNAVParser
{

private:
#pragma pack(push, 1)
    struct Message_
    {
        uint8_t groups;

        uint16_t group_field;

        float yaw;
        float pitch;
        float roll;

        float x_rate;
        float y_rate;
        float z_rate;

        uint16_t crc;
    };
#pragma pack(pop)

public:
    constexpr static unsigned YAW_ADDRESS = offsetof(Message_, yaw);
    constexpr static unsigned PITCH_ADDRESS = offsetof(Message_, pitch);
    constexpr static unsigned ROLL_ADDRESS = offsetof(Message_, roll);
    constexpr static unsigned X_RATE_ADDRESS = offsetof(Message_, x_rate);
    constexpr static unsigned Y_RATE_ADDRESS = offsetof(Message_, y_rate);
    constexpr static unsigned Z_RATE_ADDRESS = offsetof(Message_, z_rate);

private:
    // constexpr static uint32_t HEADER_ =
    //     (0xFA << 0) + (0x01 << 8) + (0x28 << (8 * 2));
    constexpr static uint8_t SYNC_ = 0xFA;
    constexpr static unsigned CRC_LENGTH_ = 2;

    constexpr static char reset_message_[] = "$VNWRG,06,0*XX\r\n";
    constexpr static char init_message_[] = "$VNWRG,75,2,8,01,0028*XX\r\n";

public:
    consteval VectorNAVParser(InputStream &stream, Logger &logger);

public:
    ReturnCode Reset();
    ReturnCode Init();

    ReturnCode Process();

    ReturnCode Read(void *buffer, uint32_t address, uint32_t length);
    ReturnCode Write(const void *buffer, uint32_t address,
                              uint32_t length);

    float GetYaw();
    float GetPitch();
    float GetRoll();

    unsigned GetWrongCRCCount() const;
    unsigned GetRubbishBytesCount() const;
    unsigned GetPackagesCount() const;

    sensors::IMUData GetIMUData();

private:
    uint16_t CalculateCRC_(uint8_t *data, unsigned length);

private:
    InputStream &stream_;

    unsigned current_rx_length_;
    unsigned rx_offset_;

    bool header_found_;

    Message_ current_data_ = {};
    Message_ rx_buffer_ = {};

    unsigned wrong_crc_counter_;
    unsigned rubbish_bytes_counter_;
    unsigned package_counter_;

    Logger &logger_;

    static_assert(sensors::IMUConcept<VectorNAVParser<InputStream, Logger>>,
                  "VectorNAVParser must have GetIMUData() -> IMUData");
};

template <concepts::stream::ByteFullStreamConcept InputStream, typename Logger>
consteval VectorNAVParser<InputStream, Logger>::VectorNAVParser(
    InputStream &stream, Logger &logger)
    : stream_(stream),
      current_rx_length_(0),
      rx_offset_(0),
      header_found_(false),
      wrong_crc_counter_(0),
      rubbish_bytes_counter_(0),
      package_counter_(0),
      logger_(logger)
{
}

template <concepts::stream::ByteFullStreamConcept InputStream, typename Logger>
ReturnCode VectorNAVParser<InputStream, Logger>::Process()
{
    // uint8_t header_buffer;
    // if (read(stream_, &header_buffer, 1) == 1)
    // {
    //     LOG(logger_, hydrolib::logger::LogLevel::DEBUG, "{} ",
    //                      header_buffer);
    //     current_rx_length_++;
    //     if (current_rx_length_ == sizeof(SYNC_) + sizeof(Message_))
    //     {
    //         current_rx_length_ = 0;
    //         LOG(logger_, hydrolib::logger::LogLevel::DEBUG, "\n\r");
    //     }
    // }

    unsigned rubbish_bytes = 0;
    while (!header_found_)
    {
        uint8_t header_buffer;
        if (read(stream_, &header_buffer, 1) != 1)
        {
            if (rubbish_bytes != 0)
            {
                LOG(logger_, hydrolib::logger::LogLevel::WARNING,
                    "Rubbish bytes: {}", rubbish_bytes);
            }
            return ReturnCode::NO_DATA;
        }
        if (header_buffer == SYNC_)
        {
            header_found_ = true;
            if (rubbish_bytes != 0)
            {
                LOG(logger_, hydrolib::logger::LogLevel::WARNING,
                    "Rubbish bytes: {}", rubbish_bytes);
            }
        }
        else
        {
            rubbish_bytes++;
            rubbish_bytes_counter_++;
        }
    }

    int required_data_length = sizeof(Message_) - current_rx_length_;
    int data_read_length = read(
        stream_, reinterpret_cast<uint8_t *>(&rx_buffer_) + current_rx_length_,
        required_data_length);
    if (data_read_length != required_data_length)
    {
        current_rx_length_ += data_read_length;
        return ReturnCode::NO_DATA;
    }

    header_found_ = false;
    current_rx_length_ = 0;

    package_counter_++;

    uint16_t supposed_crc = CalculateCRC_(
        reinterpret_cast<uint8_t *>(&rx_buffer_), sizeof(Message_));

    if (supposed_crc)
    {
        LOG(logger_, hydrolib::logger::LogLevel::WARNING, "Wrong crc");
        wrong_crc_counter_++;
        return ReturnCode::FAIL;
    }

    memcpy(&current_data_, &rx_buffer_, sizeof(Message_));

    LOG(logger_, hydrolib::logger::LogLevel::DEBUG, "Received message");
    LOG(logger_, hydrolib::logger::LogLevel::DEBUG, "yaw: {}",
        static_cast<int>(current_data_.yaw * 100));
    LOG(logger_, hydrolib::logger::LogLevel::DEBUG, "pitch: {}",
        static_cast<int>(current_data_.pitch * 100));
    LOG(logger_, hydrolib::logger::LogLevel::DEBUG, "roll: {}",
        static_cast<int>(current_data_.roll * 100));
    LOG(logger_, hydrolib::logger::LogLevel::DEBUG, "x rate: {}",
        static_cast<int>(current_data_.x_rate * 100));
    LOG(logger_, hydrolib::logger::LogLevel::DEBUG, "y rate: {}",
        static_cast<int>(current_data_.y_rate * 100));
    LOG(logger_, hydrolib::logger::LogLevel::DEBUG, "z rate: {}",
        static_cast<int>(current_data_.z_rate * 100));

    return ReturnCode::OK;
}

template <concepts::stream::ByteFullStreamConcept InputStream, typename Logger>
ReturnCode VectorNAVParser<InputStream, Logger>::Read(void *buffer,
                                                               uint32_t address,
                                                               uint32_t length)
{
    if (address + length > sizeof(Message_))
    {
        return ReturnCode::FAIL;
    }
    memcpy(buffer, reinterpret_cast<uint8_t *>(&current_data_) + address,
           length);
    return ReturnCode::OK;
}

template <concepts::stream::ByteFullStreamConcept InputStream, typename Logger>
ReturnCode
VectorNAVParser<InputStream, Logger>::Write([[maybe_unused]] const void *buffer,
                                            [[maybe_unused]] uint32_t address,
                                            [[maybe_unused]] uint32_t length)
{
    return ReturnCode::FAIL;
}

template <concepts::stream::ByteFullStreamConcept InputStream, typename Logger>
ReturnCode VectorNAVParser<InputStream, Logger>::Reset()
{
    if (write(stream_, reset_message_,
              static_cast<unsigned>(sizeof(reset_message_)) - 1) !=
        static_cast<unsigned>(sizeof(reset_message_)) - 1)
    {
        return ReturnCode::FAIL;
    }
    return ReturnCode::OK;
}

template <concepts::stream::ByteFullStreamConcept InputStream, typename Logger>
ReturnCode VectorNAVParser<InputStream, Logger>::Init()
{
    if (write(stream_, init_message_,
              static_cast<unsigned>(sizeof(init_message_)) - 1) !=
        static_cast<unsigned>(sizeof(init_message_)) - 1)
    {
        return ReturnCode::FAIL;
    }
    return ReturnCode::OK;
}

template <concepts::stream::ByteFullStreamConcept InputStream, typename Logger>
float VectorNAVParser<InputStream, Logger>::GetYaw()
{
    return current_data_.yaw;
}

template <concepts::stream::ByteFullStreamConcept InputStream, typename Logger>
float VectorNAVParser<InputStream, Logger>::GetPitch()
{
    return current_data_.pitch;
}

template <concepts::stream::ByteFullStreamConcept InputStream, typename Logger>
float VectorNAVParser<InputStream, Logger>::GetRoll()
{
    return current_data_.roll;
}

template <concepts::stream::ByteFullStreamConcept InputStream, typename Logger>
unsigned VectorNAVParser<InputStream, Logger>::GetWrongCRCCount() const
{
    return wrong_crc_counter_;
}

template <concepts::stream::ByteFullStreamConcept InputStream, typename Logger>
unsigned VectorNAVParser<InputStream, Logger>::GetRubbishBytesCount() const
{
    return rubbish_bytes_counter_;
}

template <concepts::stream::ByteFullStreamConcept InputStream, typename Logger>
unsigned VectorNAVParser<InputStream, Logger>::GetPackagesCount() const
{
    return package_counter_;
}

template <concepts::stream::ByteFullStreamConcept InputStream, typename Logger>
sensors::IMUData VectorNAVParser<InputStream, Logger>::GetIMUData()
{
    sensors::IMUData data;

    data.yaw_mdeg = static_cast<int>(current_data_.yaw * 1000);
    data.pitch_mdeg = static_cast<int>(current_data_.pitch * 1000);
    data.roll_mdeg = static_cast<int>(current_data_.roll * 1000);

    data.yaw_rate_mdeg_per_s = static_cast<int>(current_data_.z_rate * 1000);
    data.pitch_rate_mdeg_per_s = static_cast<int>(current_data_.y_rate * 1000);
    data.roll_rate_mdeg_per_s = static_cast<int>(current_data_.x_rate * 1000);

    return data;
}

template <concepts::stream::ByteFullStreamConcept InputStream, typename Logger>
uint16_t VectorNAVParser<InputStream, Logger>::CalculateCRC_(uint8_t *data,
                                                             unsigned length)
{
    uint16_t crc = 0;
    for (unsigned i = 0; i < length; i++)
    {
        crc = static_cast<uint8_t>(crc >> 8) | (crc << 8);
        crc ^= data[i];
        crc ^= static_cast<uint8_t>(crc & 0xff) >> 4;
        crc ^= crc << 12;
        crc ^= (crc & 0x00ff) << 5;
    }
    return crc;
}
} // namespace hydrolib