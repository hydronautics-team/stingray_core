#pragma once

#include "hydrolib_bus_datalink_message.hpp"
#include "hydrolib_crc.hpp"
#include "hydrolib_logger.hpp"
#include "hydrolib_return_codes.hpp"
#include "hydrolib_stream_concepts.hpp"
#include <cstring>

namespace hydrolib::bus::datalink
{
template <concepts::stream::ByteWritableStreamConcept TxStream,
          logger::LogDistributorConcept Distributor>
class Serializer
{
public:
    constexpr Serializer(AddressType self_address, TxStream &tx_stream,
                         logger::Logger<Distributor> &logger);

public:
    ReturnCode Process(AddressType dest_address, const void *data,
                       unsigned data_length);

public:
    static void COBSEncoding(uint8_t magic_byte, uint8_t *data,
                             unsigned data_length);

private:
    const AddressType address_;
    TxStream &tx_stream_;
    logger::Logger<Distributor> &logger_;

    uint8_t current_message_[kMaxMessageLength];
    MessageHeader *current_header_;
};

template <concepts::stream::ByteWritableStreamConcept TxStream,
          logger::LogDistributorConcept Distributor>
constexpr Serializer<TxStream, Distributor>::Serializer(
    AddressType address, TxStream &tx_stream,
    logger::Logger<Distributor> &logger)
    : address_(address),
      tx_stream_(tx_stream),
      logger_(logger),
      current_header_(reinterpret_cast<MessageHeader *>(current_message_))
{
    current_header_->magic_byte = kMagicByte;
    current_header_->cobs_length = 0;
    for (unsigned i = 1; i < kMaxMessageLength; i++)
    {
        current_message_[i] = 0;
    }
}

template <concepts::stream::ByteWritableStreamConcept TxStream,
          logger::LogDistributorConcept Distributor>
ReturnCode Serializer<TxStream, Distributor>::Process(AddressType dest_address,
                                                      const void *data,
                                                      unsigned data_length)
{
    current_header_->dest_address = dest_address;
    current_header_->src_address = address_;
    current_header_->length =
        static_cast<uint8_t>(sizeof(MessageHeader) + data_length + kCRCLength);
    memcpy(current_message_ + sizeof(MessageHeader), data, data_length);
    current_message_[sizeof(MessageHeader) + data_length] =
        crc::CountCRC8(current_message_, sizeof(MessageHeader) + data_length);

    COBSEncoding(kMagicByte,
                 current_message_ + sizeof(MessageHeader) -
                     sizeof(MessageHeader::cobs_length),
                 current_header_->length - sizeof(MessageHeader) +
                     sizeof(MessageHeader::cobs_length));

    int res = write(tx_stream_, current_message_, current_header_->length);
    if (res < 0)
    {
        return ReturnCode::ERROR;
    }
    else if (res != current_header_->length)
    {
        return ReturnCode::OVERFLOW;
    }
    else
    {
        return ReturnCode::OK;
    }
}

template <concepts::stream::ByteWritableStreamConcept TxStream,
          logger::LogDistributorConcept Distributor>
void Serializer<TxStream, Distributor>::COBSEncoding(uint8_t magic_byte,
                                                     uint8_t *data,
                                                     unsigned data_length)
{
    unsigned last_appearance = 0;
    for (unsigned i = 1; i < data_length; i++)
    {
        if (data[i] == magic_byte)
        {
            data[last_appearance] = i - last_appearance;
            last_appearance = i;
        }
    }
    data[last_appearance] = 0;
}

} // namespace hydrolib::bus::datalink
