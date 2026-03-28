#include "hydrolib_common.h"
#include "hydrolib_serial_protocol_message.hpp"
#include "test_hydrolib_sp_communicate_env.hpp"

using namespace hydrolib::serial_protocol;

namespace test_core
{
TEST_P(TestHydrolibSerialProtocolCommunicateParametrized, MemReadingTest)
{
    auto param = GetParam();
    uint8_t mem_address = std::get<0>(param);
    uint8_t reading_length = std::get<1>(param);
    uint8_t reading_buffer[PUBLIC_MEMORY_LENGTH];
    for (int i = 0; i < PUBLIC_MEMORY_LENGTH; i++)
    {
        public_memory.memory[i] = i;
    }
    for (uint8_t j = 0; j < 10; j++)
    {
        if (mem_address + reading_length > PUBLIC_MEMORY_LENGTH)
        {
            continue;
        }
        hydrolib_ReturnCode transmit_status = master.TransmitRead(
            SLAVE_ADDRESS, mem_address, reading_length, reading_buffer);
        EXPECT_EQ(HYDROLIB_RETURN_OK, transmit_status);

        slave.ProcessRx();
        master.ProcessRx();
        for (uint8_t i = 0; i < reading_length; i++)
        {
            EXPECT_EQ(public_memory.memory[mem_address + i], reading_buffer[i]);
        }
    }
}

TEST_P(TestHydrolibSerialProtocolCommunicateParametrized,
       MemReadingWithNoizeTest)
{
    auto param = GetParam();
    uint8_t mem_address = std::get<0>(param);
    uint8_t reading_length = std::get<1>(param);
    uint8_t reading_buffer[PUBLIC_MEMORY_LENGTH];
    for (int i = 0; i < PUBLIC_MEMORY_LENGTH; i++)
    {
        public_memory.memory[i] = i;
    }
    for (uint8_t j = 0; j < 10; j++)
    {
        if (mem_address + reading_length > PUBLIC_MEMORY_LENGTH)
        {
            continue;
        }
        transeiver.WriteByte(j);
        hydrolib_ReturnCode transmit_status = master.TransmitRead(
            SLAVE_ADDRESS, mem_address, reading_length, reading_buffer);
        EXPECT_EQ(HYDROLIB_RETURN_OK, transmit_status);

        transeiver.WriteByte(j);

        for (uint8_t i = 0; i < MessageHeader::MAX_MESSAGE_LENGTH; i++)
        {
            hydrolib_ReturnCode result = slave.ProcessRx();
            if (result == HYDROLIB_RETURN_OK)
            {
                break;
            }
            transeiver.WriteByte(j);
        }

        for (uint8_t i = 0; i < MessageHeader::MAX_MESSAGE_LENGTH; i++)
        {
            if (master.ProcessRx())
            {
                break;
            }
            transeiver.WriteByte(j);
        }

        for (uint8_t i = 0; i < reading_length; i++)
        {
            EXPECT_EQ(public_memory.memory[mem_address + i], reading_buffer[i]);
        }
    }
}
} // namespace test_core
