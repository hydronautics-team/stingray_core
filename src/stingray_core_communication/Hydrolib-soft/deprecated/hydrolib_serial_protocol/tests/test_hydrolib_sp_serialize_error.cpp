#include "hydrolib_common.h"
#include "hydrolib_serial_protocol_commands.hpp"
#include "hydrolib_serial_protocol_message.hpp"
#include "test_hydrolib_sp_serialize_env.hpp"
#include <gtest/gtest.h>

using namespace hydrolib::serial_protocol;

namespace test_core
{
TEST_F(TestHydrolibSerialProtocolSerialize, ErrorTest)
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
        transmitted_info.error = {.source_address = SERIALIZER_ADDRESS,
                                  .dest_address = DESERIALIZER_ADDRESS,
                                  .error_code = ErrorCode::NO_ERROR};
        hydrolib_ReturnCode transmit_status =
            serializer.Process(Command::ERROR, transmitted_info);
        EXPECT_EQ(HYDROLIB_RETURN_OK, transmit_status);

        hydrolib_ReturnCode receive_status = deserializer.Process();
        EXPECT_EQ(HYDROLIB_RETURN_OK, receive_status);

        Command command = deserializer.GetCommand();
        EXPECT_EQ(Command::ERROR, command);

        CommandInfo received_info = deserializer.GetInfo();
        EXPECT_EQ(SERIALIZER_ADDRESS, received_info.error.source_address);
        EXPECT_EQ(DESERIALIZER_ADDRESS, received_info.error.dest_address);
        EXPECT_EQ(ErrorCode::NO_ERROR, received_info.error.error_code);
    }
}

TEST_F(TestHydrolibSerialProtocolSerialize, ErrorWithNoizeTest)
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
        transmitted_info.error = {.source_address = SERIALIZER_ADDRESS,
                                  .dest_address = DESERIALIZER_ADDRESS,
                                  .error_code = ErrorCode::NO_ERROR};
        hydrolib_ReturnCode transmit_status =
            serializer.Process(Command::ERROR, transmitted_info);
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
        EXPECT_EQ(Command::ERROR, command);

        CommandInfo received_info = deserializer.GetInfo();
        EXPECT_EQ(SERIALIZER_ADDRESS, received_info.error.source_address);
        EXPECT_EQ(DESERIALIZER_ADDRESS, received_info.error.dest_address);
        EXPECT_EQ(ErrorCode::NO_ERROR, received_info.error.error_code);
    }
}
} // namespace test_core
