#include "test_hydrolib_rq_env.hpp"

TEST_P(TestHydrolibRingQueueCommon, PullAndPushBytes)
{
    uint16_t pull_number = GetParam() + 1;

    uint16_t repite_number = buffer_capacity / pull_number + 1;

    uint32_t test_buffer_length = repite_number * pull_number + buffer_capacity;
    uint8_t *test_buffer = new uint8_t[test_buffer_length];

    for (uint32_t i = 0; i < test_buffer_length; i++)
    {
        test_buffer[i] = i;
    }

    uint32_t pull_index = 0;
    uint32_t push_index = 0;

    for (uint16_t i = 0; i < buffer_capacity; i++)
    {
        hydrolib_ReturnCode filling_push_status = hydrolib_RingQueue_PushByte(&ring_buffer, test_buffer[push_index]);
        EXPECT_EQ(filling_push_status, hydrolib_ReturnCode::HYDROLIB_RETURN_OK);
        push_index++;
    }

    for (uint16_t i = 0; i < repite_number; i++)
    {
        bool is_full = hydrolib_RingQueue_IsFull(&ring_buffer);
        EXPECT_TRUE(is_full);

        bool is_empty = hydrolib_RingQueue_IsEmpty(&ring_buffer);
        EXPECT_FALSE(is_empty);

        for (uint16_t j = 0; j < pull_number; j++)
        {
            uint8_t read_byte;
            hydrolib_ReturnCode first_pull_status = hydrolib_RingQueue_PullByte(&ring_buffer, &read_byte);
            EXPECT_EQ(first_pull_status, hydrolib_ReturnCode::HYDROLIB_RETURN_OK);
            EXPECT_EQ(test_buffer[pull_index], read_byte);
            pull_index++;

            is_full = hydrolib_RingQueue_IsFull(&ring_buffer);
            EXPECT_FALSE(is_full);

            uint16_t length = hydrolib_RingQueue_GetLength(&ring_buffer);
            EXPECT_EQ(buffer_capacity - j - 1, length);

            if (j != buffer_capacity - 1)
            {
                is_empty = hydrolib_RingQueue_IsEmpty(&ring_buffer);
                EXPECT_FALSE(is_empty);
            }
            else
            {
                is_empty = hydrolib_RingQueue_IsEmpty(&ring_buffer);
                EXPECT_TRUE(is_empty);
            }
        }
        for (uint16_t j = 0; j < pull_number; j++)
        {
            hydrolib_ReturnCode filling_push_status = hydrolib_RingQueue_PushByte(&ring_buffer, test_buffer[push_index]);
            EXPECT_EQ(filling_push_status, hydrolib_ReturnCode::HYDROLIB_RETURN_OK);
            push_index++;

            is_empty = hydrolib_RingQueue_IsEmpty(&ring_buffer);
            EXPECT_FALSE(is_empty);

            uint16_t length = hydrolib_RingQueue_GetLength(&ring_buffer);
            EXPECT_EQ(buffer_capacity - pull_number + j + 1, length);

            if (j != pull_number - 1)
            {
                is_full = hydrolib_RingQueue_IsFull(&ring_buffer);
                EXPECT_FALSE(is_full);
            }
            else
            {
                is_full = hydrolib_RingQueue_IsFull(&ring_buffer);
                EXPECT_TRUE(is_full);
            }
        }
    }
}
