#pragma once

#include <cstdint>

namespace hydrolib::crc
{
inline uint8_t CountCRC8(const uint8_t *buffer, unsigned length)
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
} // namespace hydrolib::crc
