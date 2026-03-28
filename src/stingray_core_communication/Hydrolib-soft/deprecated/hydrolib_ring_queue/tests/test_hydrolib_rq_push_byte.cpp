#include "test_hydrolib_rq_env.hpp"

TEST_F(TestHydrolibRingQueue, PushByte)
{
    uint8_t write_byte = 1;
    hydrolib_ReturnCode push_status = hydrolib_RingQueue_PushByte(&ring_buffer, write_byte);
    EXPECT_EQ(push_status, hydrolib_ReturnCode::HYDROLIB_RETURN_OK);

    uint8_t read_byte;
    hydrolib_ReturnCode read_status = hydrolib_RingQueue_ReadByte(&ring_buffer, &read_byte, 0);
    EXPECT_EQ(read_status, hydrolib_ReturnCode::HYDROLIB_RETURN_OK);
    EXPECT_EQ(write_byte, read_byte);
}

TEST_F(TestHydrolibRingQueue, PushSomeBytes)
{
    hydrolib_RingQueue_Init(&ring_buffer, buffer, buffer_capacity);

    for (uint8_t i = 0; i < buffer_capacity; i++)
    {
        hydrolib_ReturnCode push_status = hydrolib_RingQueue_PushByte(&ring_buffer, i);
        EXPECT_EQ(push_status, hydrolib_ReturnCode::HYDROLIB_RETURN_OK);

        uint8_t read_byte;
        hydrolib_ReturnCode read_status = hydrolib_RingQueue_ReadByte(&ring_buffer, &read_byte, i);
        EXPECT_EQ(read_status, hydrolib_ReturnCode::HYDROLIB_RETURN_OK);
        EXPECT_EQ(i, read_byte);
    }
}

TEST_F(TestHydrolibRingQueue, PushBytesToLimit)
{
    hydrolib_RingQueue_Init(&ring_buffer, buffer, buffer_capacity);

    for (uint8_t i = 0; i < buffer_capacity; i++)
    {
        hydrolib_ReturnCode push_status = hydrolib_RingQueue_PushByte(&ring_buffer, i);
        EXPECT_EQ(push_status, hydrolib_ReturnCode::HYDROLIB_RETURN_OK);
    }

    uint8_t write_byte = 1;
    hydrolib_ReturnCode full_push_status = hydrolib_RingQueue_PushByte(&ring_buffer, write_byte);
    EXPECT_EQ(full_push_status, hydrolib_ReturnCode::HYDROLIB_RETURN_FAIL);

    uint8_t read_byte;
    hydrolib_ReturnCode read_status = hydrolib_RingQueue_ReadByte(&ring_buffer, &read_byte, buffer_capacity - 1);
    EXPECT_EQ(read_status, hydrolib_ReturnCode::HYDROLIB_RETURN_OK);
    EXPECT_EQ(buffer_capacity - 1, read_byte);
}
