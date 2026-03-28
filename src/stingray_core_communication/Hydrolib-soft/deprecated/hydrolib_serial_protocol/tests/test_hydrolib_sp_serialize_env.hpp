#ifndef TEST_HYDROLIB_SP_SERIALIZE_ENV_H_
#define TEST_HYDROLIB_SP_SERIALIZE_ENV_H_

#include "hydrolib_common.h"
#include "hydrolib_log_distributor.hpp"
#include "hydrolib_serial_protocol_commands.hpp"
#include "hydrolib_serial_protocol_deserializer.hpp"
#include "hydrolib_serial_protocol_serializer.hpp"

#include <deque>
#include <iostream>

#include <gtest/gtest.h>

using namespace hydrolib::serial_protocol;
using namespace hydrolib::logger;

#define PUBLIC_MEMORY_LENGTH 20

#define SERIALIZER_ADDRESS 3
#define DESERIALIZER_ADDRESS 4

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

class TestStream
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
inline int read(TestStream &stream, void *dest, unsigned length)
{
    hydrolib_ReturnCode result = stream.Read(dest, length, 0);
    if (result == HYDROLIB_RETURN_OK)
    {
        stream.Drop(length);
        return length;
    }
    return 0;
}

inline int write(TestStream &stream, const void *dest, unsigned length)
{
    hydrolib_ReturnCode result = stream.Push(dest, length);
    if (result == HYDROLIB_RETURN_OK)
    {
        return length;
    }
    return 0;
}

class TestHydrolibSerialProtocolSerialize : public ::testing::Test
{
protected:
    TestHydrolibSerialProtocolSerialize();
    TestStream stream;

    Serializer<TestStream, LogDistributor<TestLogStream>> serializer;
    Deserializer<TestStream, LogDistributor<TestLogStream>> deserializer;

    uint8_t test_data[PUBLIC_MEMORY_LENGTH];
};

class TestHydrolibSerialProtocolSerializeParametrized
    : public TestHydrolibSerialProtocolSerialize,
      public ::testing::WithParamInterface<std::tuple<uint16_t, uint16_t>>
{
};

#endif
