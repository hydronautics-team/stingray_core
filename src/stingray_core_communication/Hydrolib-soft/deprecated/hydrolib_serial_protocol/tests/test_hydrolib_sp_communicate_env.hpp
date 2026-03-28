#ifndef TEST_HYDROLIB_SP_COMMUNICATE_ENV_H_
#define TEST_HYDROLIB_SP_COMMUNICATE_ENV_H_

#include "hydrolib_common.h"
#include "hydrolib_log_distributor.hpp"
#include "hydrolib_serial_protocol_master.hpp"
#include "hydrolib_serial_protocol_slave.hpp"

#include <cstddef>
#include <cstring>
#include <deque>
#include <functional>
#include <iostream>
#include <utility>

#include <gtest/gtest.h>

using namespace hydrolib::serial_protocol;
using namespace hydrolib::logger;

#define PUBLIC_MEMORY_LENGTH 10

#define MASTER_ADDRESS 3
#define SLAVE_ADDRESS 4

class TestLogStream
{
public:
    hydrolib_ReturnCode Push(const void *data, unsigned length)
    {
        for (unsigned i = 0; i < length; i++)
        {
            std::cout << (reinterpret_cast<const char *>(data))[i];
        }
        return HYDROLIB_RETURN_OK;
    }
    hydrolib_ReturnCode Open() { return HYDROLIB_RETURN_OK; };
    hydrolib_ReturnCode Close() { return HYDROLIB_RETURN_OK; };
};

inline int write(TestLogStream &stream, const void *dest, unsigned length)
{
    hydrolib_ReturnCode result = stream.Push(dest, length);
    if (result == HYDROLIB_RETURN_OK)
    {
        return length;
    }
    return 0;
}

class TestPublicMemory
{
public:
    TestPublicMemory()
    {
        for (int i = 0; i < PUBLIC_MEMORY_LENGTH; i++)
        {
            memory[i] = i;
        }
    }

public:
    hydrolib_ReturnCode Read(void *read_buffer, unsigned address,
                             unsigned length)
    {
        if (address + length > PUBLIC_MEMORY_LENGTH)
        {
            return HYDROLIB_RETURN_FAIL;
        }
        memcpy(read_buffer, &memory[address], length);
        return HYDROLIB_RETURN_OK;
    }

public:
    hydrolib_ReturnCode Write(const void *write_buffer, unsigned address,
                              unsigned length)
    {
        if (address + length > PUBLIC_MEMORY_LENGTH)
        {
            return HYDROLIB_RETURN_FAIL;
        }
        memcpy(&memory[address], write_buffer, length);
        return HYDROLIB_RETURN_OK;
    }

public:
    uint8_t memory[PUBLIC_MEMORY_LENGTH];
};

class TestTranseiver
{
private:
    std::deque<uint8_t> queue;

public:
    hydrolib_ReturnCode Read(void *buffer, uint32_t length,
                             uint32_t shift) const;

    void Drop(uint32_t number);

    void Clear();

    hydrolib_ReturnCode Push(const void *data, uint32_t length);

    void WriteByte(uint8_t data);
};
inline int read(TestTranseiver &stream, void *dest, unsigned length)
{
    hydrolib_ReturnCode result = stream.Read(dest, length, 0);
    if (result == HYDROLIB_RETURN_OK)
    {
        stream.Drop(length);
        return length;
    }
    return 0;
}

inline int write(TestTranseiver &stream, const void *dest, unsigned length)
{
    hydrolib_ReturnCode result = stream.Push(dest, length);
    if (result == HYDROLIB_RETURN_OK)
    {
        return length;
    }
    return 0;
}

class TestHydrolibSerialProtocolCommunicate : public ::testing::Test
{
protected:
    TestHydrolibSerialProtocolCommunicate();

protected:
    TestPublicMemory public_memory;
    TestTranseiver transeiver;
    uint8_t test_data[PUBLIC_MEMORY_LENGTH];

    Master<TestTranseiver, TestPublicMemory, std::function<void()>,
           LogDistributor<TestLogStream>>
        master;

    Slave<TestTranseiver, TestPublicMemory, LogDistributor<TestLogStream>>
        slave;

    bool responce_flag;
};

class TestHydrolibSerialProtocolCommunicateParametrized
    : public TestHydrolibSerialProtocolCommunicate,
      public ::testing::WithParamInterface<std::tuple<uint16_t, uint16_t>>
{
};

#endif
