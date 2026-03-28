#ifndef HYDROLIB_SERIAL_PROTOCOL_COMMANDS_H_
#define HYDROLIB_SERIAL_PROTOCOL_COMMANDS_H_

#include <cstdint>

namespace hydrolib::serial_protocol
{

enum Command : uint8_t
{
    WRITE = 1,
    READ,
    RESPONCE,
    ERROR
};

enum ErrorCode : uint8_t
{
    NO_ERROR = 1,
    BAD_ADDRESS,
    NO_ACCESS,
    INTERNAL_ERROR
};

union CommandInfo
{
public:
    struct Read
    {
        unsigned source_address;
        unsigned dest_address;
        unsigned memory_address;
        unsigned memory_access_length;
    };

    struct Write
    {
        unsigned source_address;
        unsigned dest_address;
        unsigned memory_address;
        unsigned memory_access_length;

        const uint8_t *data;
    };

    struct Responce
    {
        unsigned source_address;
        unsigned dest_address;
        unsigned data_length;

        const uint8_t *data;
    };

    struct Error
    {
        unsigned source_address;
        unsigned dest_address;
        ErrorCode error_code;
    };

public:
    Read read;
    Write write;
    Responce responce;
    Error error;
};

} // namespace hydrolib::serial_protocol

#endif
