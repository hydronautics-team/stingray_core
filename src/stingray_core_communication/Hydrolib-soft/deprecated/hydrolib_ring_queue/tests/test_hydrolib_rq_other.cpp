#include "test_hydrolib_rq_env.hpp"

TEST_F(TestHydrolibRingQueue, IsEmpty)
{
    bool is_empty_initial = hydrolib_RingQueue_IsEmpty(&ring_buffer);
    EXPECT_TRUE(is_empty_initial);

    uint8_t write_byte = 1;
    hydrolib_ReturnCode push_status = hydrolib_RingQueue_PushByte(&ring_buffer, write_byte);
    EXPECT_EQ(push_status, hydrolib_ReturnCode::HYDROLIB_RETURN_OK);

    bool is_empty_push = hydrolib_RingQueue_IsEmpty(&ring_buffer);
    EXPECT_FALSE(is_empty_push);

    uint8_t read_byte;
    hydrolib_ReturnCode pull_status = hydrolib_RingQueue_PullByte(&ring_buffer, &read_byte);
    EXPECT_EQ(pull_status, hydrolib_ReturnCode::HYDROLIB_RETURN_OK);

    bool is_empty_pull = hydrolib_RingQueue_IsEmpty(&ring_buffer);
    EXPECT_TRUE(is_empty_pull);
}

TEST_F(TestHydrolibRingQueue, IsFull)
{
    bool is_full_initial = hydrolib_RingQueue_IsFull(&ring_buffer);
    EXPECT_FALSE(is_full_initial);

    for (uint16_t i = 0; i < buffer_capacity; i++)
    {
        hydrolib_ReturnCode push_status = hydrolib_RingQueue_PushByte(&ring_buffer, i);
        EXPECT_EQ(push_status, hydrolib_ReturnCode::HYDROLIB_RETURN_OK);
    }

    bool is_full_after_filling = hydrolib_RingQueue_IsFull(&ring_buffer);
    EXPECT_TRUE(is_full_after_filling);

    uint8_t read_byte;
    hydrolib_ReturnCode pull_status = hydrolib_RingQueue_PullByte(&ring_buffer, &read_byte);
    EXPECT_EQ(pull_status, hydrolib_ReturnCode::HYDROLIB_RETURN_OK);

    bool is_full_after_pull = hydrolib_RingQueue_IsFull(&ring_buffer);
    EXPECT_FALSE(is_full_after_pull);

    uint8_t write_byte = 1;
    hydrolib_ReturnCode finall_push_status = hydrolib_RingQueue_PushByte(&ring_buffer, write_byte);
    EXPECT_EQ(finall_push_status, hydrolib_ReturnCode::HYDROLIB_RETURN_OK);

    bool is_full_after_push = hydrolib_RingQueue_IsFull(&ring_buffer);
    EXPECT_TRUE(is_full_after_push);
}

TEST_P(TestHydrolibRingQueueCommon, Length)
{
    uint16_t push_number = GetParam();
    for (uint16_t i = 0; i < push_number; i++)
    {
        hydrolib_ReturnCode push_status = hydrolib_RingQueue_PushByte(&ring_buffer, i);
        EXPECT_EQ(push_status, hydrolib_ReturnCode::HYDROLIB_RETURN_OK);

        uint16_t length = hydrolib_RingQueue_GetLength(&ring_buffer);
        EXPECT_EQ(i + 1, length);
    }
    for (uint16_t i = 0; i < push_number; i++)
    {
        uint8_t read_byte;
        hydrolib_ReturnCode pull_status = hydrolib_RingQueue_PullByte(&ring_buffer, &read_byte);
        EXPECT_EQ(pull_status, hydrolib_ReturnCode::HYDROLIB_RETURN_OK);

        uint16_t length = hydrolib_RingQueue_GetLength(&ring_buffer);
        EXPECT_EQ(push_number - i - 1, length);
    }
}

TEST_P(TestHydrolibRingQueueCommon, Clear)
{
    uint16_t push_number = GetParam();
    for (uint16_t i = 0; i < push_number; i++)
    {
        hydrolib_ReturnCode push_status = hydrolib_RingQueue_PushByte(&ring_buffer, i);
        EXPECT_EQ(push_status, hydrolib_ReturnCode::HYDROLIB_RETURN_OK);
    }
    hydrolib_RingQueue_Clear(&ring_buffer);

    bool is_empty = hydrolib_RingQueue_IsEmpty(&ring_buffer);
    EXPECT_TRUE(is_empty);
}

TEST_P(TestHydrolibRingQueueCommon, Drop)
{
    uint16_t drop_number = GetParam();
    for (uint16_t i = 0; i < buffer_capacity; i++)
    {
        hydrolib_ReturnCode filling_status = hydrolib_RingQueue_PushByte(&ring_buffer, i);
        EXPECT_EQ(filling_status, hydrolib_ReturnCode::HYDROLIB_RETURN_OK);
    }
    hydrolib_ReturnCode drop_status = hydrolib_RingQueue_Drop(&ring_buffer, drop_number);
    EXPECT_EQ(drop_status, hydrolib_ReturnCode::HYDROLIB_RETURN_OK);

    uint16_t length = hydrolib_RingQueue_GetLength(&ring_buffer);
    EXPECT_EQ(buffer_capacity-drop_number, length);

    uint8_t read_byte = -1;
    hydrolib_ReturnCode read_status = hydrolib_RingQueue_ReadByte(&ring_buffer, &read_byte, 0);
    EXPECT_EQ(read_status, hydrolib_ReturnCode::HYDROLIB_RETURN_OK);
    EXPECT_EQ(drop_number, read_byte);

    hydrolib_ReturnCode wrong_drop_status = hydrolib_RingQueue_Drop(&ring_buffer, buffer_capacity+1);
    EXPECT_EQ(wrong_drop_status, hydrolib_ReturnCode::HYDROLIB_RETURN_FAIL);
}
