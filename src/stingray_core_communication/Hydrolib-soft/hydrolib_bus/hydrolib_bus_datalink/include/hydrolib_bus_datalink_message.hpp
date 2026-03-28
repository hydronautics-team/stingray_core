#pragma once

#include <cstdint>

namespace hydrolib::bus::datalink
{

using AddressType = uint8_t;

struct MessageHeader
{
    uint8_t magic_byte;
    AddressType dest_address;
    AddressType src_address;
    uint8_t length;
    uint8_t cobs_length;
} __attribute__((__packed__));

constexpr uint8_t kMagicByte = 0xAA;
constexpr unsigned kCRCLength = 1;
constexpr unsigned kMaxMessageLength = UINT8_MAX;
constexpr unsigned kMaxDataLength =
    kMaxMessageLength - sizeof(MessageHeader) - kCRCLength;

} // namespace hydrolib::bus::datalink