#ifndef HYDROLIB_STREAM_CONCEPTS_H_
#define HYDROLIB_STREAM_CONCEPTS_H_

#include <concepts>
#include <cstdint>

namespace hydrolib::concepts::stream
{

template <typename T>
concept ByteWritableStreamConcept =
    requires(T stream, const void *source, unsigned length) {
        { write(stream, source, length) } -> std::convertible_to<int>;
    };

template <typename T>
concept ByteReadableStreamConcept =
    requires(T stream, void *dest, unsigned length) {
        { read(stream, dest, length) } -> std::convertible_to<int>;
    };

template <typename T>
concept ByteFullStreamConcept =
    ByteWritableStreamConcept<T> && ByteReadableStreamConcept<T>;
} // namespace hydrolib::concepts::stream

#endif
