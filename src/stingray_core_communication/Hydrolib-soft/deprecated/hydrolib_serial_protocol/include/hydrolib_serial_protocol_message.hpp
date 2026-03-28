#ifndef HYDROLIB_SERIAL_PROTOCOL_MESSAGE_H_
#define HYDROLIB_SERIAL_PROTOCOL_MESSAGE_H_

#include <cstdint>

namespace hydrolib::serial_protocol
{
union MessageHeader
{
public:
#pragma pack(push, 1)

    struct Common
    {
        uint8_t device_address;
        uint8_t command;
        uint8_t self_address;
        uint8_t message_length;
    };

    struct MemoryAccess
    {
        uint8_t device_address;
        uint8_t command;
        uint8_t self_address;
        uint8_t message_length;
        uint8_t memory_address;
        uint8_t memory_access_length;
    };

    struct Error
    {
        uint8_t device_address;
        uint8_t command;
        uint8_t self_address;
        uint8_t message_length;
        uint8_t error_code;
    };

#pragma pack(pop)

public:
    constexpr static unsigned MAX_MESSAGE_LENGTH = 255;
    constexpr static unsigned ADDRESS_BITS_NUMBER = 4;
    constexpr static unsigned NETWORK_BITS_NUMBER = 8 - ADDRESS_BITS_NUMBER;

    constexpr static uint8_t ADDRESS_MASK = 0xFF >> (8 - ADDRESS_BITS_NUMBER);
    constexpr static uint8_t NETWORK_MASK = ~ADDRESS_MASK;
    constexpr static unsigned CRC_LENGTH = 1;

    constexpr static unsigned MAX_DATA_LENGTH =
        MAX_MESSAGE_LENGTH - sizeof(MessageHeader::MemoryAccess);

public:
    static uint8_t CountCRC(const uint8_t *buffer, unsigned length);
    constexpr static uint8_t GetTrueAddress(uint8_t address, uint8_t network);
    constexpr static uint8_t GetReadableAddress(uint8_t address);

public:
    Common common;
    MemoryAccess memory_access;
    Common responce;
    Error error;
};

inline uint8_t MessageHeader::CountCRC(const uint8_t *buffer, unsigned length)
{
    uint16_t pol = 0x0700;
    uint16_t crc = buffer[0] << 8;
    for (uint8_t i = 1; i < length; i++)
    {
        crc |= buffer[i];
        for (uint8_t j = 0; j < 8; j++)
        {
            if (crc & 0x8000)
            {
                crc = (crc << 1 ^ pol);
            }
            else
            {
                crc = crc << 1;
            }
        }
    }
    return crc >> 8;
}

constexpr inline uint8_t MessageHeader::GetTrueAddress(uint8_t address,
                                                       uint8_t network)
{
    return (network & MessageHeader::NETWORK_MASK) |
           (address & MessageHeader::ADDRESS_MASK);
}

constexpr inline uint8_t MessageHeader::GetReadableAddress(uint8_t true_address)
{
    return true_address & MessageHeader::ADDRESS_MASK;
}

} // namespace hydrolib::serial_protocol

#endif
