#pragma once

#include <iostream>

#include <gtest/gtest.h>

#include "hydrolib_bus_application_commands.hpp"
#include "hydrolib_bus_application_master.hpp"
#include "hydrolib_bus_application_slave.hpp"
#include "hydrolib_log_distributor.hpp"
#include "hydrolib_return_codes.hpp"

class TestLogStream
{
};

int write([[maybe_unused]] TestLogStream &stream, const void *dest,
          unsigned length);

class TestPublicMemory
{
public:
    static constexpr unsigned kPublicMemoryLength = 10;

public:
    TestPublicMemory()
    {
        for (unsigned i = 0; i < kPublicMemoryLength; i++)
        {
            memory[i] = i;
        }
    }

public:
    hydrolib::ReturnCode Read(void *read_buffer, unsigned address,
                              unsigned length)
    {
        if (address + length > kPublicMemoryLength)
        {
            return hydrolib::ReturnCode::FAIL;
        }
        memcpy(read_buffer, &memory[address], length);
        return hydrolib::ReturnCode::OK;
    }

public:
    hydrolib::ReturnCode Write(const void *write_buffer, unsigned address,
                               unsigned length)
    {
        if (address + length > kPublicMemoryLength)
        {
            return hydrolib::ReturnCode::FAIL;
        }
        memcpy(&memory[address], write_buffer, length);
        return hydrolib::ReturnCode::OK;
    }

public:
    uint8_t memory[kPublicMemoryLength];
};

class TestStream
{
public:
    uint8_t buffer[hydrolib::bus::application::kMaxMessageLength];
    unsigned length = 0;
    bool has_data = false;
};

int write(TestStream &stream, const void *dest, unsigned length);
int read(TestStream &stream, void *dest, unsigned length);

class TestHydrolibBusApplication : public ::testing::Test
{
protected:
    TestHydrolibBusApplication();

protected:
    TestStream stream;
    TestPublicMemory memory;

    hydrolib::bus::application::Master<
        TestStream, hydrolib::logger::LogDistributor<TestLogStream>>
        master;
    hydrolib::bus::application::Slave<
        TestPublicMemory, hydrolib::logger::LogDistributor<TestLogStream>,
        TestStream>
        slave;

    uint8_t test_data[TestPublicMemory::kPublicMemoryLength];
};
