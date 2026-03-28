#include "hydrolib_serial_protocol_message.hpp"
#include "test_hydrolib_sp_communicate_env.hpp"

using namespace hydrolib::serial_protocol;

TEST_P(TestHydrolibSerialProtocolCommunicateParametrized, MemWritingTest)
{
    auto param = GetParam();
    uint8_t mem_address = std::get<0>(param);
    uint8_t writing_length = std::get<1>(param);
    for (uint8_t j = 0; j < 10; j++)
    {
        if (mem_address + writing_length > PUBLIC_MEMORY_LENGTH)
        {
            continue;
        }
        hydrolib_ReturnCode transmit_status = master.TransmitWrite(
            SLAVE_ADDRESS, mem_address, writing_length, test_data);
        EXPECT_EQ(HYDROLIB_RETURN_OK, transmit_status);

        hydrolib_ReturnCode receive_status = slave.ProcessRx();
        EXPECT_EQ(HYDROLIB_RETURN_OK, receive_status);
        for (uint8_t i = 0; i < writing_length; i++)
        {
            EXPECT_EQ(test_data[i], public_memory.memory[mem_address + i]);
        }
    }
}

TEST_P(TestHydrolibSerialProtocolCommunicateParametrized,
       MemWritingWithNoizeTest)
{
    auto param = GetParam();
    uint8_t mem_address = std::get<0>(param);
    uint8_t writing_length = std::get<1>(param);
    for (uint8_t j = 0; j < 10; j++)
    {
        if (mem_address + writing_length > PUBLIC_MEMORY_LENGTH)
        {
            continue;
        }
        transeiver.WriteByte(j);
        hydrolib_ReturnCode transmit_status = master.TransmitWrite(
            SLAVE_ADDRESS, mem_address, writing_length, test_data);

        EXPECT_EQ(HYDROLIB_RETURN_OK, transmit_status);

        for (uint8_t i = 0; i < MessageHeader::MAX_MESSAGE_LENGTH; i++)
        {
            if (slave.ProcessRx())
            {
                break;
            }
            transeiver.WriteByte(j);
        }
        for (uint8_t i = 0; i < writing_length; i++)
        {
            EXPECT_EQ(test_data[i], public_memory.memory[mem_address + i]);
        }
    }
}
