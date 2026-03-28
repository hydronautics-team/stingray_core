#ifndef TEST_HYDROLIB_RQ_ENV_H_
#define TEST_HYDROLIB_RQ_ENV_H_

#include "hydrolib_ring_queue.h"

#include <gtest/gtest.h>

#define DEFAULT_CAPACITY 16

class TestHydrolibRingQueue : public ::testing::Test
{
protected:
    TestHydrolibRingQueue()
    {
        buffer_capacity = DEFAULT_CAPACITY;
        buffer = new uint8_t[buffer_capacity];
        hydrolib_RingQueue_Init(&ring_buffer, buffer, buffer_capacity);
    }

    hydrolib_RingQueue ring_buffer;
    uint8_t buffer_capacity;
    uint8_t *buffer;

    ~TestHydrolibRingQueue()
    {
        delete buffer;
    }
};

class TestHydrolibRingQueueCommon : public TestHydrolibRingQueue,
                                    public ::testing::WithParamInterface<uint16_t>
{
};

#endif
