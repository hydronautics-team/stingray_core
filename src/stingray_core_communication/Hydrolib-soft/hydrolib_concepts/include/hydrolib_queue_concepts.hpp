#ifndef HYDROLIB_QUEUE_CONCEPTS_H_
#define HYDROLIB_QUEUE_CONCEPTS_H_

#include "hydrolib_common.h"

#include <concepts>

namespace hydrolib::concepts::queue
{
template <typename T>
concept ReadableByteQueue =
    requires(T queue, void *data, unsigned data_length, unsigned shift) {
        {
            queue.Read(data, data_length, shift)
        } -> std::same_as<hydrolib_ReturnCode>;
        queue.Drop(data_length);
        queue.Clear();
    };
} // namespace hydrolib::concepts::queue

#endif
