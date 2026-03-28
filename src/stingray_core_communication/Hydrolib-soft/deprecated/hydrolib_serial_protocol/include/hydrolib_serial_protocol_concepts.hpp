#ifndef HYDROLIB_SERIAL_PROTOCOL_CONCEPTS_H_
#define HYDROLIB_SERIAL_PROTOCOL_CONCEPTS_H_

#include "hydrolib_queue_concepts.hpp"
#include "hydrolib_stream_concepts.hpp"

namespace hydrolib::serial_protocol
{
template <typename T>
concept CallbackConcept = requires(T callback) { callback(); };

template <typename T>
concept TransceiverConcept = concepts::stream::ByteWritableStreamConcept<T> &&
                             concepts::stream::ByteReadableStreamConcept<T>;

} // namespace hydrolib::serial_protocol

#endif
