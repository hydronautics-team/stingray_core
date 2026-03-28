#ifndef HYDROLIB_SERIAL_PROTOCOL_SERIALIZER_H_
#define HYDROLIB_SERIAL_PROTOCOL_SERIALIZER_H_

#include <cstdint>
#include <cstring>

#include "hydrolib_common.h"
#include "hydrolib_logger.hpp"
#include "hydrolib_serial_protocol_commands.hpp"
#include "hydrolib_serial_protocol_message.hpp"
#include "hydrolib_stream_concepts.hpp"

namespace hydrolib::serial_protocol
{
template <concepts::stream::ByteWritableStreamConcept TxStream,
          logger::LogDistributorConcept Distributor>
class Serializer
{
public:
    constexpr Serializer(uint8_t address, TxStream &tx_stream,
                         logger::Logger<Distributor> &logger,
                         uint8_t network = 0xA0);

public:
    hydrolib_ReturnCode Process(Command command, CommandInfo info);

private:
    void SerializeRead_(CommandInfo info);
    void SerializeWrite_(CommandInfo info);
    void SerializeResponce_(CommandInfo info);
    void SerializeError_(CommandInfo info);

private:
    logger::Logger<Distributor> &logger_;

    const uint8_t self_address_;
    const uint8_t network_;

    TxStream &tx_stream_;

    uint8_t current_message_[MessageHeader::MAX_MESSAGE_LENGTH];
    unsigned current_message_length_;

    MessageHeader *current_header_;
};

template <concepts::stream::ByteWritableStreamConcept TxStream,
          logger::LogDistributorConcept Distributor>
constexpr Serializer<TxStream, Distributor>::Serializer(
    uint8_t address, TxStream &tx_stream, logger::Logger<Distributor> &logger,
    uint8_t network)
    : logger_(logger),
      self_address_(MessageHeader::GetTrueAddress(address, network)),
      network_(network),
      tx_stream_(tx_stream),
      current_message_length_(0),
      current_header_(reinterpret_cast<MessageHeader *>(&current_message_))
{
    for (unsigned i = 0; i < MessageHeader::MAX_MESSAGE_LENGTH; i++)
    {
        current_message_[i] = 0;
    }
}

template <concepts::stream::ByteWritableStreamConcept TxStream,
          logger::LogDistributorConcept Distributor>
hydrolib_ReturnCode Serializer<TxStream, Distributor>::Process(Command command,
                                                               CommandInfo info)
{
    current_header_->common.command = command;
    current_header_->common.self_address = self_address_;
    switch (command)
    {
    case READ:
        SerializeRead_(info);
        break;
    case WRITE:
        SerializeWrite_(info);
        break;
    case RESPONCE:
        SerializeResponce_(info);
        break;
    case ERROR:
        SerializeError_(info);
        break;
    }
    current_message_[current_message_length_ - MessageHeader::CRC_LENGTH] =
        MessageHeader::CountCRC(current_message_,
                                current_header_->common.message_length -
                                    MessageHeader::CRC_LENGTH);
    int res = write(tx_stream_, current_message_, current_message_length_);
    if (res != static_cast<int>(current_message_length_))
    {
        return HYDROLIB_RETURN_FAIL;
    }
    return HYDROLIB_RETURN_OK;
}

template <concepts::stream::ByteWritableStreamConcept TxStream,
          logger::LogDistributorConcept Distributor>
void Serializer<TxStream, Distributor>::SerializeRead_(CommandInfo info)
{
    current_header_->memory_access.device_address =
        MessageHeader::GetTrueAddress(info.read.dest_address, network_);
    current_header_->memory_access.memory_address = info.read.memory_address;
    current_header_->memory_access.memory_access_length =
        info.read.memory_access_length;
    current_message_length_ =
        sizeof(MessageHeader::MemoryAccess) + MessageHeader::CRC_LENGTH;
    current_header_->memory_access.message_length = current_message_length_;
}

template <concepts::stream::ByteWritableStreamConcept TxStream,
          logger::LogDistributorConcept Distributor>
void Serializer<TxStream, Distributor>::SerializeWrite_(CommandInfo info)
{
    current_header_->memory_access.device_address =
        MessageHeader::GetTrueAddress(info.write.dest_address, network_);
    current_header_->memory_access.memory_address = info.write.memory_address;
    current_header_->memory_access.memory_access_length =
        info.write.memory_access_length;
    current_message_length_ = sizeof(MessageHeader::MemoryAccess) +
                              info.write.memory_access_length +
                              MessageHeader::CRC_LENGTH;
    current_header_->memory_access.message_length = current_message_length_;
    memcpy(current_message_ + sizeof(MessageHeader::MemoryAccess),
           info.write.data, info.write.memory_access_length);
}

template <concepts::stream::ByteWritableStreamConcept TxStream,
          logger::LogDistributorConcept Distributor>
void Serializer<TxStream, Distributor>::SerializeResponce_(CommandInfo info)
{
    current_header_->responce.device_address =
        MessageHeader::GetTrueAddress(info.responce.dest_address, network_);
    current_message_length_ = sizeof(MessageHeader::Common) +
                              info.responce.data_length +
                              MessageHeader::CRC_LENGTH;
    current_header_->responce.message_length = current_message_length_;
    memcpy(current_message_ + sizeof(MessageHeader::Common), info.responce.data,
           info.responce.data_length);
}

template <concepts::stream::ByteWritableStreamConcept TxStream,
          logger::LogDistributorConcept Distributor>
void Serializer<TxStream, Distributor>::SerializeError_(CommandInfo info)
{
    current_header_->error.device_address =
        MessageHeader::GetTrueAddress(info.responce.dest_address, network_);
    current_message_length_ =
        sizeof(MessageHeader::Error) + MessageHeader::CRC_LENGTH;
    current_header_->error.message_length = current_message_length_;
    current_header_->error.error_code = info.error.error_code;
}

} // namespace hydrolib::serial_protocol

#endif
