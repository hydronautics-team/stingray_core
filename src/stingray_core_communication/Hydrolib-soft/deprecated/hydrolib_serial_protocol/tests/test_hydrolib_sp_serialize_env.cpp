#include "test_hydrolib_sp_serialize_env.hpp"
#include "hydrolib_logger.hpp"

inline TestLogStream log_stream;
inline LogDistributor distributor("[%s] [%l] %m\n", log_stream);
inline Logger logger("Serializer", 0, distributor);

hydrolib_ReturnCode TestStream::Read(void *buffer, uint32_t length,
                                     uint32_t shift) const
{
    uint8_t *byte_buffer = reinterpret_cast<uint8_t *>(buffer);
    if (shift + length > queue.size())
    {
        return HYDROLIB_RETURN_FAIL;
    }
    for (uint32_t i = 0; i < length; i++)
    {
        byte_buffer[i] = queue[shift + i];
    }
    return HYDROLIB_RETURN_OK;
}

void TestStream::Drop(uint32_t number)
{
    if (number > queue.size())
    {
        queue.clear();
    }
    else
    {
        for (uint32_t i = 0; i < number; i++)
        {
            queue.pop_front();
        }
    }
}

void TestStream::Clear() { queue.clear(); }

hydrolib_ReturnCode TestStream::Push(const void *data, uint32_t length)
{
    for (uint32_t i = 0; i < length; i++)
    {
        queue.push_back(reinterpret_cast<const uint8_t *>(data)[i]);
    }
    return HYDROLIB_RETURN_OK;
}

void TestStream::WriteByte(uint8_t data) { queue.push_back(data); }

TestHydrolibSerialProtocolSerialize::TestHydrolibSerialProtocolSerialize()
    : serializer(SERIALIZER_ADDRESS, stream, logger),
      deserializer(DESERIALIZER_ADDRESS, stream, logger)
{
    distributor.SetAllFilters(0, LogLevel::DEBUG);
    for (int i = 0; i < PUBLIC_MEMORY_LENGTH; i++)
    {
        test_data[i] = i;
    }
}

// INSTANTIATE_TEST_CASE_P(
//     Test, TestHydrolibSerialProtocolSerializeParametrized,
//     ::testing::Combine(::testing::Range<uint16_t>(0, PUBLIC_MEMORY_LENGTH),
//                        ::testing::Range<uint16_t>(1,
//                                                   PUBLIC_MEMORY_LENGTH +
//                                                   1)));
