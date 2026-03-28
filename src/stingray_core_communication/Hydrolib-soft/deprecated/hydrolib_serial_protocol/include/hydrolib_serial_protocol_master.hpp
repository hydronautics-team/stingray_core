#ifndef HYDROLIB_SERIAL_PROTOCOL_MASTER_H_
#define HYDROLIB_SERIAL_PROTOCOL_MASTER_H_

#include "hydrolib_common.h"
#include "hydrolib_serial_protocol_concepts.hpp"
#include "hydrolib_serial_protocol_deserializer.hpp"
#include "hydrolib_serial_protocol_interpreter.hpp"
#include "hydrolib_serial_protocol_serializer.hpp"

namespace hydrolib::serial_protocol
{
template <TransceiverConcept Transeiver, PublicMemoryConcept Memory,
          CallbackConcept Callback, logger::LogDistributorConcept Distributor>
class Master
{
public:
    constexpr Master(uint8_t address, Transeiver &transeiver,
                     Memory &public_memory, logger::Logger<Distributor> &logger,
                     Callback responce_callback, uint8_t network = 0xA0);

public:
    hydrolib_ReturnCode ProcessRx();
    hydrolib_ReturnCode TransmitWrite(unsigned device_address,
                                      unsigned memory_address,
                                      unsigned memory_length,
                                      const uint8_t *data);
    hydrolib_ReturnCode TransmitRead(unsigned device_address,
                                     unsigned memory_address,
                                     unsigned memory_length, uint8_t *data);

private:
    logger::Logger<Distributor> &logger_;

    Serializer<Transeiver, Distributor> serializer_;
    Deserializer<Transeiver, Distributor> deserializer_;

    Interpreter<Memory, Distributor, Serializer<Transeiver, Distributor>>
        interpreter_;

    const Callback responce_callback_;
};

template <TransceiverConcept Transeiver, PublicMemoryConcept Memory,
          CallbackConcept Callback, logger::LogDistributorConcept Distributor>
constexpr Master<Transeiver, Memory, Callback, Distributor>::Master(
    uint8_t address, Transeiver &transeiver, Memory &public_memory,
    logger::Logger<Distributor> &logger, Callback responce_callback,
    uint8_t network)
    : logger_(logger),
      serializer_(address, transeiver, logger, network),
      deserializer_(address, transeiver, logger, network),
      interpreter_(public_memory, serializer_, logger),
      responce_callback_(responce_callback)
{
}

template <TransceiverConcept Transeiver, PublicMemoryConcept Memory,
          CallbackConcept Callback, logger::LogDistributorConcept Distributor>
hydrolib_ReturnCode
Master<Transeiver, Memory, Callback, Distributor>::ProcessRx()
{
    hydrolib_ReturnCode deserialization_res = deserializer_.Process();
    if (deserialization_res != HYDROLIB_RETURN_OK)
    {
        return deserialization_res;
    }

    hydrolib_ReturnCode interpritation_res = interpreter_.ProcessRx(
        deserializer_.GetCommand(), deserializer_.GetInfo());
    if (interpritation_res != HYDROLIB_RETURN_OK)
    {
        return interpritation_res;
    }
    responce_callback_();

    return HYDROLIB_RETURN_OK;
}

template <TransceiverConcept Transeiver, PublicMemoryConcept Memory,
          CallbackConcept Callback, logger::LogDistributorConcept Distributor>
hydrolib_ReturnCode
Master<Transeiver, Memory, Callback, Distributor>::TransmitWrite(
    unsigned device_address, unsigned memory_address, unsigned memory_length,
    const uint8_t *data)
{
    return interpreter_.TransmitWrite(device_address, memory_address,
                                      memory_length, data);
}

template <TransceiverConcept Transeiver, PublicMemoryConcept Memory,
          CallbackConcept Callback, logger::LogDistributorConcept Distributor>
hydrolib_ReturnCode
Master<Transeiver, Memory, Callback, Distributor>::TransmitRead(
    unsigned device_address, unsigned memory_address, unsigned memory_length,
    uint8_t *data)
{
    return interpreter_.TransmitRead(device_address, memory_address,
                                     memory_length, data);
}

} // namespace hydrolib::serial_protocol

#endif
