#include "test_hydrolib_rq_env.hpp"

class TestHydrolibRingQueuePush : public TestHydrolibRingQueue,
                                  public ::testing::WithParamInterface<uint16_t>
{
};

INSTANTIATE_TEST_CASE_P(
    Test,
    TestHydrolibRingQueuePush,
    ::testing::Range<uint16_t>(1, DEFAULT_CAPACITY));

TEST_P(TestHydrolibRingQueuePush, Push)
{
    uint16_t length = GetParam();
    uint8_t data[DEFAULT_CAPACITY];
    for (uint16_t i = 0; i < DEFAULT_CAPACITY; i++)
    {
        data[i] = i;
    }

    hydrolib_ReturnCode push_status = hydrolib_RingQueue_Push(&ring_buffer, data, length);
    EXPECT_EQ(push_status, HYDROLIB_RETURN_OK);

    for (uint16_t i = 0; i < length; i++)
    {
        uint8_t read_byte;
        hydrolib_ReturnCode read_status = hydrolib_RingQueue_ReadByte(&ring_buffer, &read_byte, i);
        EXPECT_EQ(read_status, HYDROLIB_RETURN_OK);
        EXPECT_EQ(i, read_byte);
    }
}

TEST_P(TestHydrolibRingQueuePush, PushShifted)
{
    uint16_t length = GetParam();
    uint8_t data[DEFAULT_CAPACITY];
    for (uint16_t i = 0; i < DEFAULT_CAPACITY; i++)
    {
        data[i] = i;
    }

    for (uint16_t i = 0; i < DEFAULT_CAPACITY / 2; i++)
    {
        hydrolib_ReturnCode filling_status =
            hydrolib_RingQueue_PushByte(&ring_buffer, 0);
        EXPECT_EQ(filling_status, hydrolib_ReturnCode::HYDROLIB_RETURN_OK);
    }

    hydrolib_RingQueue_Drop(&ring_buffer, DEFAULT_CAPACITY / 2);

    hydrolib_ReturnCode push_status = hydrolib_RingQueue_Push(&ring_buffer, data, length);
    EXPECT_EQ(push_status, HYDROLIB_RETURN_OK);

    for (uint16_t i = 0; i < length; i++)
    {
        uint8_t read_byte;
        hydrolib_ReturnCode read_status = hydrolib_RingQueue_ReadByte(&ring_buffer, &read_byte, i);
        EXPECT_EQ(read_status, HYDROLIB_RETURN_OK);
        EXPECT_EQ(i, read_byte);
    }
}

TEST_F(TestHydrolibRingQueue, PushOverSimple)
{
    uint8_t data[DEFAULT_CAPACITY + 1];
    for (uint16_t i = 0; i < DEFAULT_CAPACITY + 1; i++)
    {
        data[i] = i;
    }

    hydrolib_ReturnCode push_status = hydrolib_RingQueue_Push(&ring_buffer, data, DEFAULT_CAPACITY + 1);
    EXPECT_EQ(push_status, HYDROLIB_RETURN_FAIL);
}

TEST_P(TestHydrolibRingQueuePush, PushOverComplex)
{
    uint16_t length = GetParam();
    uint8_t data[DEFAULT_CAPACITY];
    for (uint16_t i = 0; i < DEFAULT_CAPACITY; i++)
    {
        data[i] = i;
    }

    for (uint16_t i = 0; i < length; i++)
    {
        hydrolib_ReturnCode filling_status =
            hydrolib_RingQueue_PushByte(&ring_buffer, data[i]);
        EXPECT_EQ(filling_status, hydrolib_ReturnCode::HYDROLIB_RETURN_OK);
    }

    hydrolib_ReturnCode push_status = hydrolib_RingQueue_Push(&ring_buffer, data, DEFAULT_CAPACITY - length + 1);
    EXPECT_EQ(push_status, HYDROLIB_RETURN_FAIL);

    for (uint16_t i = 0; i < length; i++)
    {
        uint8_t read_byte;
        hydrolib_ReturnCode read_status = hydrolib_RingQueue_ReadByte(&ring_buffer, &read_byte, i);
        EXPECT_EQ(read_status, HYDROLIB_RETURN_OK);
        EXPECT_EQ(i, read_byte);
    }
}

TEST_P(TestHydrolibRingQueuePush, PushToLimit)
{
    uint16_t length = GetParam();
    uint8_t data[DEFAULT_CAPACITY];
    for (uint16_t i = 0; i < DEFAULT_CAPACITY; i++)
    {
        data[i] = i;
    }

    for (uint16_t i = length; i <= DEFAULT_CAPACITY; i += length)
    {
        hydrolib_ReturnCode push_status = hydrolib_RingQueue_Push(&ring_buffer, data + i - length, length);
        EXPECT_EQ(push_status, HYDROLIB_RETURN_OK);
    }

    for (uint16_t i = 0; i < DEFAULT_CAPACITY / length * length; i++)
    {
        uint8_t read_byte;
        hydrolib_ReturnCode read_status = hydrolib_RingQueue_ReadByte(&ring_buffer, &read_byte, i);
        EXPECT_EQ(read_status, HYDROLIB_RETURN_OK);
        EXPECT_EQ(i, read_byte);
    }
}
