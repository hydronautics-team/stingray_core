#include "hydrolib_ring_queue.h"

#include <string.h>

void hydrolib_RingQueue_Init(hydrolib_RingQueue *self, void *buffer, uint16_t byte_capacity)
{
    self->buffer = buffer;
    self->capacity = byte_capacity;
    self->head = 0;
    self->tail = 0;
    self->length = 0;
}

void hydrolib_RingQueue_Clear(hydrolib_RingQueue *self)
{
    self->head = 0;
    self->tail = 0;
    self->length = 0;
}

hydrolib_ReturnCode hydrolib_RingQueue_Drop(hydrolib_RingQueue *self, uint16_t number)
{
    if (number > self->length)
    {
        return HYDROLIB_RETURN_FAIL;
    }
    self->head = (self->head + number) % self->capacity;
    self->length -= number;
    return HYDROLIB_RETURN_OK;
}

hydrolib_ReturnCode hydrolib_RingQueue_PushByte(hydrolib_RingQueue *self, uint8_t byte)
{
    if (self->length + 1 > self->capacity)
    {
        return HYDROLIB_RETURN_FAIL;
    }

    self->buffer[self->tail] = byte;
    self->tail = (self->tail + 1) % self->capacity;
    self->length++;

    return HYDROLIB_RETURN_OK;
}

hydrolib_ReturnCode hydrolib_RingQueue_Push(hydrolib_RingQueue *self, const void *data, uint16_t data_length)
{
    if (self->length + data_length > self->capacity)
    {
        return HYDROLIB_RETURN_FAIL;
    }

    uint16_t forward_length = self->capacity - self->tail;
    if (forward_length >= data_length)
    {
        memcpy(self->buffer + self->tail, data, data_length);
    }
    else
    {
        memcpy(self->buffer + self->tail, data, forward_length);
        memcpy(self->buffer, (uint8_t *)data + forward_length, data_length - forward_length);
    }
    self->tail = (self->tail + data_length) % self->capacity;
    self->length += data_length;

    return HYDROLIB_RETURN_OK;
}

hydrolib_ReturnCode hydrolib_RingQueue_PullByte(hydrolib_RingQueue *self, uint8_t *byte)
{
    if (self->length == 0)
    {
        return HYDROLIB_RETURN_FAIL;
    }

    *byte = self->buffer[self->head];
    self->head = (self->head + 1) % self->capacity;
    self->length--;

    return HYDROLIB_RETURN_OK;
}

hydrolib_ReturnCode hydrolib_RingQueue_Pull(hydrolib_RingQueue *self, void *data, uint16_t data_length)
{
    if (self->length < data_length)
    {
        return HYDROLIB_RETURN_FAIL;
    }

    uint16_t forward_length = self->capacity - self->head;
    if (forward_length >= data_length)
    {
        memcpy(data, self->buffer + self->head, data_length);
    }
    else
    {
        memcpy(data, self->buffer + self->head, forward_length);
        memcpy((uint8_t *)data + forward_length, self->buffer, data_length - forward_length);
    }
    self->head = (self->head + data_length) % self->capacity;
    self->length -= data_length;

    return HYDROLIB_RETURN_OK;
}

hydrolib_ReturnCode hydrolib_RingQueue_ReadByte(const hydrolib_RingQueue *self, uint8_t *data, uint16_t shift)
{
    if (shift >= self->length)
    {
        return HYDROLIB_RETURN_FAIL;
    }

    uint16_t buffer_index = (self->head + shift) % self->capacity;
    *data = self->buffer[buffer_index];

    return HYDROLIB_RETURN_OK;
}

hydrolib_ReturnCode hydrolib_RingQueue_Read2BytesLE(hydrolib_RingQueue *self, uint16_t *data, uint16_t shift)
{
    if (shift >= self->length - 1)
    {
        return HYDROLIB_RETURN_FAIL;
    }
    uint16_t buffer_index = (self->head + shift) % self->capacity;

    if (buffer_index == self->capacity - 1)
    {
        *data = self->buffer[buffer_index] | self->buffer[0] << 8;
    }
    else
    {
        *data = *(uint16_t *)(self->buffer + buffer_index);
    }

    return HYDROLIB_RETURN_OK;
}

hydrolib_ReturnCode hydrolib_RingQueue_Read4BytesLE(hydrolib_RingQueue *self, uint32_t *data, uint16_t shift)
{
    if (shift >= self->length - 3)
    {
        return HYDROLIB_RETURN_FAIL;
    }
    uint16_t buffer_index = (self->head + shift) % self->capacity;

    if (buffer_index == self->capacity - 1)
    {
        *data = self->buffer[buffer_index] | *((uint32_t *)self->buffer) << 8;
    }
    else if (buffer_index == self->capacity - 2)
    {
        *data = *(uint16_t *)(self->buffer + buffer_index) | *((uint32_t *)self->buffer) << 16;
    }
    else if (buffer_index == self->capacity - 3)
    {
        *data = *(uint32_t *)(self->buffer + buffer_index - 1) >> 8 | self->buffer[0] << 24;
    }
    else
    {
        *data = *(uint32_t *)(self->buffer + buffer_index);
    }

    return HYDROLIB_RETURN_OK;
}

hydrolib_ReturnCode hydrolib_RingQueue_Read(const hydrolib_RingQueue *self,
                                            void *data, uint16_t data_length, uint16_t shift)
{
    if (shift + data_length > self->length)
    {
        return HYDROLIB_RETURN_FAIL;
    }
    uint16_t forward_length = self->capacity - self->head;
    if (forward_length > shift)
    {
        if (shift + data_length > forward_length)
        {
            memcpy(data, self->buffer + self->head + shift, forward_length - shift);
            memcpy((uint8_t *)data + forward_length - shift,
                   self->buffer, data_length - (forward_length - shift));
        }
        else
        {
            memcpy(data, self->buffer + self->head + shift, data_length);
        }
    }
    else
    {
        memcpy(data, self->buffer + shift - forward_length, data_length);
    }
    return HYDROLIB_RETURN_OK;
}

uint16_t hydrolib_RingQueue_FindByte(hydrolib_RingQueue *self, uint8_t target_byte, uint16_t shift)
{
    for (uint16_t i = shift; i < self->length; i++)
    {
        uint16_t current_index = (self->head + i) % self->capacity;
        if (self->buffer[current_index] == target_byte)
        {
            return i;
        }
    }
    return -1;
}

uint16_t hydrolib_RingQueue_Find2BytesLE(hydrolib_RingQueue *self, uint16_t target_bytes, uint16_t shift)
{
    for (uint16_t i = shift; i < self->length; i++)
    {
        uint16_t current_index = (self->head + i) % self->capacity;
        uint16_t current_data;
        if (current_index == self->capacity - 1)
        {
            current_data = self->buffer[current_index] | self->buffer[0] << 8;
        }
        else
        {
            current_data = *(uint16_t *)(self->buffer + current_index);
        }
        if (current_data == target_bytes)
        {
            return i;
        }
    }
    return -1;
}

uint16_t hydrolib_RingQueue_Find4BytesLE(hydrolib_RingQueue *self, uint32_t target_bytes, uint16_t shift)
{
    for (uint16_t i = shift; i < self->length; i++)
    {
        uint16_t current_index = (self->head + i) % self->capacity;
        uint32_t current_data;
        if (current_index == self->capacity - 1)
        {
            current_data = self->buffer[current_index] | *((uint32_t *)self->buffer) << 8;
        }
        else if (current_index == self->capacity - 2)
        {
            current_data = *(uint16_t *)(self->buffer + current_index) | *((uint32_t *)self->buffer) << 16;
        }
        else if (current_index == self->capacity - 3)
        {
            current_data = *(uint32_t *)(self->buffer + current_index - 1) >> 8 | self->buffer[0] << 24;
        }
        else
        {
            current_data = *(uint32_t *)(self->buffer + current_index);
        }
        if (current_data == target_bytes)
        {
            return i;
        }
    }
    return -1;
}

uint16_t hydrolib_RingQueue_GetLength(hydrolib_RingQueue *self)
{
    return self->length;
}

uint16_t hydrolib_RingQueue_GetCapacity(hydrolib_RingQueue *self)
{
    return self->capacity;
}

bool hydrolib_RingQueue_IsEmpty(hydrolib_RingQueue *self)
{
    return self->length == 0;
}

bool hydrolib_RingQueue_IsFull(hydrolib_RingQueue *self)
{
    return self->length == self->capacity;
}
