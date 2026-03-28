#include "hydrolib_common.h"
#include "hydrolib_serial_protocol_commands.hpp"
#include "hydrolib_serial_protocol_message.hpp"
#include "test_hydrolib_sp_serialize_env.hpp"
#include <gtest/gtest.h>

using namespace hydrolib::serial_protocol;

namespace test_core
{
TEST_F(TestHydrolibSerialProtocolSerialize, ResponceTest)
{
    uint8_t mem_address = 2;
    uint8_t data_length = 6;
    for (unsigned j = 0; j < 500; j++)
    {
        if (mem_address + data_length > PUBLIC_MEMORY_LENGTH)
        {
            continue;
        }
        CommandInfo transmitted_info;
        transmitted_info.responce = {.source_address = SERIALIZER_ADDRESS,
                                     .dest_address = DESERIALIZER_ADDRESS,
                                     .data_length = data_length,
                                     .data = test_data + mem_address};
        hydrolib_ReturnCode transmit_status =
            serializer.Process(Command::RESPONCE, transmitted_info);
        EXPECT_EQ(HYDROLIB_RETURN_OK, transmit_status);

        hydrolib_ReturnCode receive_status = deserializer.Process();
        EXPECT_EQ(HYDROLIB_RETURN_OK, receive_status);

        Command command = deserializer.GetCommand();
        EXPECT_EQ(Command::RESPONCE, command);

        CommandInfo received_info = deserializer.GetInfo();
        EXPECT_EQ(SERIALIZER_ADDRESS, received_info.responce.source_address);
        EXPECT_EQ(DESERIALIZER_ADDRESS, received_info.responce.dest_address);
        EXPECT_EQ(data_length, received_info.responce.data_length);

        for (uint8_t i = 0; i < data_length; i++)
        {
            EXPECT_EQ(test_data[mem_address + i],
                      received_info.responce.data[i]);
        }
    }
}

TEST_F(TestHydrolibSerialProtocolSerialize, ResponceWithNoizeTest)
{
    uint8_t mem_address = 2;
    uint8_t data_length = 6;
    for (unsigned j = 0; j < 500; j++)
    {
        if (mem_address + data_length > PUBLIC_MEMORY_LENGTH)
        {
            continue;
        }

        stream.WriteByte(j);

        CommandInfo transmitted_info;
        transmitted_info.responce = {.source_address = SERIALIZER_ADDRESS,
                                     .dest_address = DESERIALIZER_ADDRESS,
                                     .data_length = data_length,
                                     .data = test_data + mem_address};
        hydrolib_ReturnCode transmit_status =
            serializer.Process(Command::RESPONCE, transmitted_info);
        EXPECT_EQ(HYDROLIB_RETURN_OK, transmit_status);

        hydrolib_ReturnCode receive_status;
        for (unsigned i = 0; i < MessageHeader::MAX_MESSAGE_LENGTH; i++)
        {
            receive_status = deserializer.Process();
            if (receive_status != HYDROLIB_RETURN_NO_DATA)
            {
                break;
            }
            stream.WriteByte(j);
        }

        EXPECT_EQ(HYDROLIB_RETURN_OK, receive_status);

        Command command = deserializer.GetCommand();
        EXPECT_EQ(Command::RESPONCE, command);

        CommandInfo received_info = deserializer.GetInfo();
        EXPECT_EQ(SERIALIZER_ADDRESS, received_info.responce.source_address);
        EXPECT_EQ(DESERIALIZER_ADDRESS, received_info.responce.dest_address);
        EXPECT_EQ(data_length, received_info.responce.data_length);

        for (uint8_t i = 0; i < data_length; i++)
        {
            EXPECT_EQ(test_data[mem_address + i], received_info.write.data[i]);
        }
    }
}
} // namespace test_core
