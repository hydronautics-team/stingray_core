#include "hydrolib_common.h"
#include "hydrolib_serial_protocol_commands.hpp"
#include "test_hydrolib_sp_interpreter_env.hpp"
#include <gtest/gtest.h>

using namespace hydrolib::serial_protocol;

namespace test_core
{
TEST_F(TestHydrolibSerialProtocolInterpreter, SlaveWrite)
{
    uint8_t mem_address = 2;
    uint8_t data_length = 6;

    CommandInfo info;
    info.write = {.source_address = 0,
                  .dest_address = 0,
                  .memory_address = mem_address,
                  .memory_access_length = data_length,
                  .data = test_data};

    auto result = interpreter.ProcessRx(Command::WRITE, info);
    EXPECT_EQ(result, HYDROLIB_RETURN_OK);

    for (int i = 0; i < data_length; i++)
    {
        EXPECT_EQ(public_memory.memory[mem_address + i], test_data[i]);
    }
}
TEST_F(TestHydrolibSerialProtocolInterpreter, SlaveRead)
{
    uint8_t mem_address = 2;
    uint8_t data_length = 6;
    uint8_t source_address = 4;

    CommandInfo info;
    info.read = {.source_address = source_address,
                 .dest_address = 0,
                 .memory_address = mem_address,
                 .memory_access_length = data_length};

    auto result = interpreter.ProcessRx(Command::READ, info);
    EXPECT_EQ(result, HYDROLIB_RETURN_OK);

    EXPECT_EQ(transmitter.command, Command::RESPONCE);
    EXPECT_EQ(transmitter.command_info.responce.data_length, data_length);
    EXPECT_EQ(transmitter.command_info.responce.dest_address, source_address);

    for (int i = 0; i < data_length; i++)
    {
        EXPECT_EQ(transmitter.command_info.responce.data[i],
                  public_memory.memory[mem_address + i]);
    }
}

TEST_F(TestHydrolibSerialProtocolInterpreter, MasterWrite)
{
    uint8_t mem_address = 2;
    uint8_t data_length = 6;
    uint8_t dest_address = 4;

    auto result = interpreter.TransmitWrite(dest_address, mem_address,
                                            data_length, test_data);
    EXPECT_EQ(result, HYDROLIB_RETURN_OK);

    EXPECT_EQ(transmitter.command, Command::WRITE);
    EXPECT_EQ(transmitter.command_info.write.dest_address, dest_address);
    EXPECT_EQ(transmitter.command_info.write.memory_address, mem_address);
    EXPECT_EQ(transmitter.command_info.write.memory_access_length, data_length);

    for (int i = 0; i < data_length; i++)
    {
        EXPECT_EQ(transmitter.command_info.write.data[i], test_data[i]);
    }
}

TEST_F(TestHydrolibSerialProtocolInterpreter, MasterRead)
{
    uint8_t mem_address = 2;
    uint8_t data_length = 6;
    uint8_t dest_address = 4;
    uint8_t buffer[PUBLIC_MEMORY_LENGTH];

    auto result = interpreter.TransmitRead(dest_address, mem_address,
                                           data_length, buffer);
    EXPECT_EQ(result, HYDROLIB_RETURN_OK);

    EXPECT_EQ(transmitter.command, Command::READ);
    EXPECT_EQ(transmitter.command_info.read.dest_address, dest_address);
    EXPECT_EQ(transmitter.command_info.read.memory_address, mem_address);
    EXPECT_EQ(transmitter.command_info.read.memory_access_length, data_length);

    CommandInfo info;
    info.responce = {.source_address = dest_address,
                     .dest_address = 0,
                     .data_length = data_length,
                     .data = test_data};

    auto responce_result = interpreter.ProcessRx(Command::RESPONCE, info);
    EXPECT_EQ(responce_result, HYDROLIB_RETURN_OK);

    for (int i = 0; i < data_length; i++)
    {
        EXPECT_EQ(buffer[i], test_data[i]);
    }
}
} // namespace test_core