#ifndef HYDROLIB_SERIAL_PROTOCOL_SLAVE_H_
#define HYDROLIB_SERIAL_PROTOCOL_SLAVE_H_

#include "hydrolib_common.h"
#include "hydrolib_serial_protocol_concepts.hpp"
#include "hydrolib_serial_protocol_deserializer.hpp"
#include "hydrolib_serial_protocol_interpreter.hpp"
#include "hydrolib_serial_protocol_serializer.hpp"

namespace hydrolib::serial_protocol
{
template <TransceiverConcept Transeiver, PublicMemoryConcept Memory,
          logger::LogDistributorConcept Distributor>
class Slave
{
public:
    constexpr Slave(uint8_t address, Transeiver &transeiver,
                    Memory &public_memory, logger::Logger<Distributor> &logger,
                    uint8_t network = 0xA0);

public:
    hydrolib_ReturnCode ProcessRx();

private:
    logger::Logger<Distributor> &logger_;

    Serializer<Transeiver, Distributor> serializer_;
    Deserializer<Transeiver, Distributor> deserializer_;

    Interpreter<Memory, Distributor, Serializer<Transeiver, Distributor>>
        interpreter_;
};

template <TransceiverConcept Transeiver, PublicMemoryConcept Memory,
          logger::LogDistributorConcept Distributor>
constexpr Slave<Transeiver, Memory, Distributor>::Slave(
    uint8_t address, Transeiver &transeiver, Memory &public_memory,
    logger::Logger<Distributor> &logger, uint8_t network)
    : logger_(logger),
      serializer_(address, transeiver, logger, network),
      deserializer_(address, transeiver, logger, network),
      interpreter_(public_memory, serializer_, logger)
{
}

template <TransceiverConcept Transeiver, PublicMemoryConcept Memory,
          logger::LogDistributorConcept Distributor>
hydrolib_ReturnCode Slave<Transeiver, Memory, Distributor>::ProcessRx()
{
    hydrolib_ReturnCode deserialization_res = deserializer_.Process();
    if (deserialization_res != HYDROLIB_RETURN_OK)
    {
        return deserialization_res;
    }

    return interpreter_.ProcessRx(deserializer_.GetCommand(),
                                  deserializer_.GetInfo());
}

} // namespace hydrolib::serial_protocol

#endif
