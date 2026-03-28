#include "test_hydrolib_bus_application.hpp"
#include <gtest/gtest.h>

TestLogStream log_stream;
hydrolib::logger::LogDistributor distributor("[%s] [%l] %m\n", log_stream);
hydrolib::logger::Logger logger("Application", 0, distributor);

int write([[maybe_unused]] TestLogStream &stream, const void *dest,
          unsigned length)
{
    for (unsigned i = 0; i < length; i++)
    {
        std::cout << (reinterpret_cast<const char *>(dest))[i];
    }
    return length;
}

int read(TestStream &stream, void *dest, unsigned length)
{
    if (stream.has_data)
    {
        stream.has_data = false;
        memcpy(dest, stream.buffer, length);
        return stream.length;
    }
    return 0;
}

int write(TestStream &stream, const void *dest, unsigned length)
{
    stream.has_data = true;
    stream.length = length;
    memcpy(stream.buffer, dest, length);
    return length;
}

TestHydrolibBusApplication::TestHydrolibBusApplication()
    : master(stream, logger), slave(stream, memory, logger)
{
    for (unsigned i = 0; i < TestPublicMemory::kPublicMemoryLength; i++)
    {
        test_data[i] = i;
    }
}

TEST_F(TestHydrolibBusApplication, WriteTest)
{
    unsigned address = 0;
    unsigned length = 5;
    uint8_t buffer[TestPublicMemory::kPublicMemoryLength];
    master.Write(test_data, address, length);
    slave.Process();
    memory.Read(buffer, address, length);
    for (unsigned i = 0; i < length; i++)
    {
        EXPECT_EQ(buffer[i], test_data[i]);
    }
    master.Write(test_data + TestPublicMemory::kPublicMemoryLength - length,
                 address, length);
    slave.Process();
    memory.Read(buffer, address, length);
    for (unsigned i = 0; i < length; i++)
    {
        EXPECT_EQ(
            buffer[i],
            test_data[TestPublicMemory::kPublicMemoryLength - length + i]);
    }
}

TEST_F(TestHydrolibBusApplication, ReadTest)
{
    unsigned address = 0;
    unsigned length = 5;
    memory.Write(test_data, 0, TestPublicMemory::kPublicMemoryLength);
    uint8_t buffer[TestPublicMemory::kPublicMemoryLength];
    master.Read(buffer, address, length);
    slave.Process();
    master.Process();
    for (unsigned i = 0; i < length; i++)
    {
        EXPECT_EQ(buffer[i], test_data[address + i]);
    }
}
