#pragma once

#include <cstdint>

namespace hydrolib::bus::application
{
enum class Command : uint8_t
{
    WRITE,
    READ,
    RESPONSE,
    ERROR
};

struct MemoryAccessInfo
{
    uint8_t address;
    uint8_t length;
} __attribute__((__packed__));

struct MemoryAccessHeader
{
    Command command;
    MemoryAccessInfo info;
} __attribute__((__packed__));

constexpr unsigned kMaxDataLength = UINT8_MAX;
constexpr unsigned kMaxMessageLength =
    sizeof(MemoryAccessHeader) + kMaxDataLength;
} // namespace hydrolib::bus::application
