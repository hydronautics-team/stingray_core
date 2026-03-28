#include "test_hydrolib_rq_env.hpp"

class TestHydrolibRingQueueRead : public TestHydrolibRingQueue,
                                  public ::testing::WithParamInterface<std::tuple<uint16_t, uint16_t>>
{
};

INSTANTIATE_TEST_CASE_P(
    Test,
    TestHydrolibRingQueueRead,
    ::testing::Combine(
        ::testing::Range<uint16_t>(0, DEFAULT_CAPACITY),
        ::testing::Range<uint16_t>(1, DEFAULT_CAPACITY + 1)));

TEST_P(TestHydrolibRingQueueRead, Read)
{
    auto param = GetParam();
    uint16_t shift = std::get<0>(param);
    uint16_t length = std::get<1>(param);

    for (uint16_t i = 0; i < DEFAULT_CAPACITY; i++)
    {
        hydrolib_ReturnCode filling_status = hydrolib_RingQueue_PushByte(&ring_buffer, i);
        EXPECT_EQ(filling_status, hydrolib_ReturnCode::HYDROLIB_RETURN_OK);
    }

    uint8_t data[DEFAULT_CAPACITY];
    hydrolib_ReturnCode read_status = hydrolib_RingQueue_Read(&ring_buffer, data, length, shift);
    if (length + shift > DEFAULT_CAPACITY)
    {
        EXPECT_EQ(read_status, hydrolib_ReturnCode::HYDROLIB_RETURN_FAIL);
    }
    else
    {
        EXPECT_EQ(read_status, hydrolib_ReturnCode::HYDROLIB_RETURN_OK);

        for (uint8_t i = 0; i < length; i++)
        {
            EXPECT_EQ(shift + i, data[i]);
        }
    }
}

TEST_P(TestHydrolibRingQueueRead, ReadShifted)
{
    auto param = GetParam();
    uint16_t shift = std::get<0>(param);
    uint16_t length = std::get<1>(param);

    for (uint16_t i = 0; i < DEFAULT_CAPACITY; i++)
    {
        hydrolib_ReturnCode filling_status = hydrolib_RingQueue_PushByte(&ring_buffer, i);
        EXPECT_EQ(filling_status, hydrolib_ReturnCode::HYDROLIB_RETURN_OK);
    }

    hydrolib_RingQueue_Drop(&ring_buffer, DEFAULT_CAPACITY / 2);

    for (uint16_t i = 0; i < DEFAULT_CAPACITY / 2; i++)
    {
        hydrolib_ReturnCode filling_status =
            hydrolib_RingQueue_PushByte(&ring_buffer, DEFAULT_CAPACITY + i);
        EXPECT_EQ(filling_status, hydrolib_ReturnCode::HYDROLIB_RETURN_OK);
    }

    uint8_t data[DEFAULT_CAPACITY];
    hydrolib_ReturnCode read_status = hydrolib_RingQueue_Read(&ring_buffer, data, length, shift);
    if (length + shift > DEFAULT_CAPACITY)
    {
        EXPECT_EQ(read_status, hydrolib_ReturnCode::HYDROLIB_RETURN_FAIL);
    }
    else
    {
        EXPECT_EQ(read_status, hydrolib_ReturnCode::HYDROLIB_RETURN_OK);

        for (uint8_t i = 0; i < length; i++)
        {
            EXPECT_EQ(shift + i + DEFAULT_CAPACITY / 2, data[i]);
        }
    }
}
