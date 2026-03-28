#include "test_hydrolib_rq_env.hpp"

TEST_F(TestHydrolibRingQueue, PullByte)
{
    hydrolib_RingQueue_Init(&ring_buffer, buffer, buffer_capacity);

    uint8_t write_byte = 1;
    hydrolib_ReturnCode push_status = hydrolib_RingQueue_PushByte(&ring_buffer, write_byte);
    EXPECT_EQ(push_status, hydrolib_ReturnCode::HYDROLIB_RETURN_OK);

    uint8_t read_byte = -1;
    hydrolib_ReturnCode pull_status = hydrolib_RingQueue_PullByte(&ring_buffer, &read_byte);
    EXPECT_EQ(pull_status, hydrolib_ReturnCode::HYDROLIB_RETURN_OK);
    EXPECT_EQ(write_byte, read_byte);
}

TEST_F(TestHydrolibRingQueue, PullSomeBytes)
{
    for (uint8_t i = 0; i < buffer_capacity; i++)
    {
        hydrolib_ReturnCode push_status = hydrolib_RingQueue_PushByte(&ring_buffer, i);
        EXPECT_EQ(push_status, hydrolib_ReturnCode::HYDROLIB_RETURN_OK);
    }

    for (uint8_t i = 0; i < buffer_capacity; i++)
    {
        uint8_t read_byte = -1;
        hydrolib_ReturnCode pull_status = hydrolib_RingQueue_PullByte(&ring_buffer, &read_byte);
        EXPECT_EQ(pull_status, hydrolib_ReturnCode::HYDROLIB_RETURN_OK);
        EXPECT_EQ(i, read_byte);
    }
}

TEST_F(TestHydrolibRingQueue, PullByteFromEmpty)
{
    uint8_t read_byte = -1;
    hydrolib_ReturnCode initial_pull_status = hydrolib_RingQueue_PullByte(&ring_buffer, &read_byte);
    EXPECT_EQ(initial_pull_status, hydrolib_ReturnCode::HYDROLIB_RETURN_FAIL);

    uint8_t write_byte = 1;
    hydrolib_ReturnCode push_status = hydrolib_RingQueue_PushByte(&ring_buffer, write_byte);
    EXPECT_EQ(push_status, hydrolib_ReturnCode::HYDROLIB_RETURN_OK);

    read_byte = -1;
    hydrolib_ReturnCode correct_pull_status = hydrolib_RingQueue_PullByte(&ring_buffer, &read_byte);
    EXPECT_EQ(correct_pull_status, hydrolib_ReturnCode::HYDROLIB_RETURN_OK);
    EXPECT_EQ(write_byte, read_byte);

    hydrolib_ReturnCode final_pull_status = hydrolib_RingQueue_PullByte(&ring_buffer, &read_byte);
    EXPECT_EQ(final_pull_status, hydrolib_ReturnCode::HYDROLIB_RETURN_FAIL);
}

TEST_F(TestHydrolibRingQueue, PullByteFromEmptyAfterFilling)
{
    for (uint8_t i = 0; i < buffer_capacity; i++)
    {
        hydrolib_ReturnCode push_status = hydrolib_RingQueue_PushByte(&ring_buffer, i);
        EXPECT_EQ(push_status, hydrolib_ReturnCode::HYDROLIB_RETURN_OK);
    }

    uint8_t read_byte;
    for (uint8_t i = 0; i < buffer_capacity; i++)
    {
        hydrolib_ReturnCode pull_status = hydrolib_RingQueue_PullByte(&ring_buffer, &read_byte);
        EXPECT_EQ(pull_status, hydrolib_ReturnCode::HYDROLIB_RETURN_OK);
    }

    hydrolib_ReturnCode pull_empty_status = hydrolib_RingQueue_PullByte(&ring_buffer, &read_byte);
    EXPECT_EQ(pull_empty_status, hydrolib_ReturnCode::HYDROLIB_RETURN_FAIL);
}
