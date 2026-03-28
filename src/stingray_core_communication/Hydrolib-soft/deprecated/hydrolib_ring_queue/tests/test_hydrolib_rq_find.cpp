#include "test_hydrolib_rq_env.hpp"

TEST_P(TestHydrolibRingQueueCommon, FindByte)
{
    uint16_t shift = GetParam();

    for (uint16_t i = 0; i < buffer_capacity / 2; i++)
    {
        hydrolib_ReturnCode push_status = hydrolib_RingQueue_PushByte(&ring_buffer, i);
        EXPECT_EQ(push_status, hydrolib_ReturnCode::HYDROLIB_RETURN_OK);
    }

    for (uint16_t i = 0; i < buffer_capacity / 2; i++)
    {
        uint8_t read_byte;
        hydrolib_ReturnCode emptying_status = hydrolib_RingQueue_PullByte(&ring_buffer, &read_byte);
        EXPECT_EQ(emptying_status, hydrolib_ReturnCode::HYDROLIB_RETURN_OK);
    }

    for (uint16_t i = 0; i < buffer_capacity; i++)
    {
        hydrolib_ReturnCode filling_status = hydrolib_RingQueue_PushByte(&ring_buffer, i);
        EXPECT_EQ(filling_status, hydrolib_ReturnCode::HYDROLIB_RETURN_OK);
    }

    for (uint16_t i = 0; i < buffer_capacity; i++)
    {
        uint16_t found_index = hydrolib_RingQueue_FindByte(&ring_buffer, i, shift);
        if (i >= shift)
        {
            EXPECT_EQ(i, found_index);
        }
        else
        {
            EXPECT_EQ((uint16_t)(-1), found_index);
        }
    }

    uint16_t found_index = hydrolib_RingQueue_FindByte(&ring_buffer, buffer_capacity, shift);
    EXPECT_EQ((uint16_t)(-1), found_index);
}

TEST_P(TestHydrolibRingQueueCommon, Find2BytesLE)
{
    uint16_t shift = GetParam();

    for (uint16_t i = 0; i < buffer_capacity / 2; i++)
    {
        hydrolib_ReturnCode push_status = hydrolib_RingQueue_PushByte(&ring_buffer, i);
        EXPECT_EQ(push_status, hydrolib_ReturnCode::HYDROLIB_RETURN_OK);
    }

    for (uint16_t i = 0; i < buffer_capacity / 2; i++)
    {
        uint8_t read_byte;
        hydrolib_ReturnCode emptying_status = hydrolib_RingQueue_PullByte(&ring_buffer, &read_byte);
        EXPECT_EQ(emptying_status, hydrolib_ReturnCode::HYDROLIB_RETURN_OK);
    }

    for (uint16_t i = 0; i < buffer_capacity; i++)
    {
        hydrolib_ReturnCode filling_status = hydrolib_RingQueue_PushByte(&ring_buffer, i);
        EXPECT_EQ(filling_status, hydrolib_ReturnCode::HYDROLIB_RETURN_OK);
    }

    for (uint16_t i = 0; i < buffer_capacity - 1; i++)
    {
        uint16_t test_bytes = i | (i + 1) << 8;
        uint16_t found_index = hydrolib_RingQueue_Find2BytesLE(&ring_buffer, test_bytes, shift);
        if (i >= shift)
        {
            EXPECT_EQ(i, found_index);
        }
        else
        {
            EXPECT_EQ((uint16_t)(-1), found_index);
        }
    }

    uint16_t found_index = hydrolib_RingQueue_Find2BytesLE(&ring_buffer, buffer_capacity, shift);
    EXPECT_EQ((uint16_t)(-1), found_index);
}

TEST_P(TestHydrolibRingQueueCommon, Find4BytesLE)
{
    uint16_t shift = GetParam();

    for (uint16_t i = 0; i < buffer_capacity / 2; i++)
    {
        hydrolib_ReturnCode push_status = hydrolib_RingQueue_PushByte(&ring_buffer, i);
        EXPECT_EQ(push_status, hydrolib_ReturnCode::HYDROLIB_RETURN_OK);
    }

    for (uint16_t i = 0; i < buffer_capacity / 2; i++)
    {
        uint8_t read_byte;
        hydrolib_ReturnCode emptying_status = hydrolib_RingQueue_PullByte(&ring_buffer, &read_byte);
        EXPECT_EQ(emptying_status, hydrolib_ReturnCode::HYDROLIB_RETURN_OK);
    }

    for (uint16_t i = 0; i < buffer_capacity; i++)
    {
        hydrolib_ReturnCode filling_status = hydrolib_RingQueue_PushByte(&ring_buffer, i);
        EXPECT_EQ(filling_status, hydrolib_ReturnCode::HYDROLIB_RETURN_OK);
    }

    for (uint16_t i = 0; i < buffer_capacity - 3; i++)
    {
        uint32_t test_bytes = i | (i + 1) << 8 | (i + 2) << 16 | (i + 3) << 24;
        uint16_t found_index = hydrolib_RingQueue_Find4BytesLE(&ring_buffer, test_bytes, shift);
        if (i >= shift)
        {
            EXPECT_EQ(i, found_index);
        }
        else
        {
            EXPECT_EQ((uint16_t)(-1), found_index);
        }
    }

    uint16_t found_index = hydrolib_RingQueue_Find4BytesLE(&ring_buffer, buffer_capacity, shift);
    EXPECT_EQ((uint16_t)(-1), found_index);
}
