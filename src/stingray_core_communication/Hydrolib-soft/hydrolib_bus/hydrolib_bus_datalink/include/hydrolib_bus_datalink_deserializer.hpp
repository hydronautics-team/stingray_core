#pragma once

#include "hydrolib_bus_datalink_message.hpp"
#include "hydrolib_crc.hpp"
#include "hydrolib_logger.hpp"
#include "hydrolib_return_codes.hpp"
#include "hydrolib_stream_concepts.hpp"

namespace hydrolib::bus::datalink
{
template <concepts::stream::ByteReadableStreamConcept RxStream,
          logger::LogDistributorConcept Distributor>
class Deserializer
{
public:
    constexpr Deserializer(AddressType address, RxStream &rx_stream,
                           logger::Logger<Distributor> &logger);

public:
    ReturnCode Process();

    AddressType GetSourceAddress() const;
    const uint8_t *GetData();
    unsigned GetDataLength() const;

public:
    static void COBSDecoding(uint8_t magic_byte, uint8_t *data,
                             unsigned data_length);

private:
    ReturnCode FindHeader_();
    ReturnCode ParseHeader_();

    bool CheckCRC_();

private:
    const AddressType self_address_;

    RxStream &rx_stream_;
    logger::Logger<Distributor> &logger_;

    unsigned current_processed_length_;
    uint8_t *current_rx_buffer_;
    uint8_t *next_rx_buffer_;
    bool message_ready_;

    uint8_t first_rx_buffer_[kMaxMessageLength];
    uint8_t second_rx_buffer_[kMaxMessageLength];

    MessageHeader *current_header_;
};

template <concepts::stream::ByteReadableStreamConcept RxStream,
          logger::LogDistributorConcept Distributor>
constexpr Deserializer<RxStream, Distributor>::Deserializer(
    AddressType address, RxStream &rx_stream,
    logger::Logger<Distributor> &logger)
    : self_address_(address),
      rx_stream_(rx_stream),
      logger_(logger),
      current_processed_length_(0),
      current_rx_buffer_(first_rx_buffer_),
      next_rx_buffer_(second_rx_buffer_),
      message_ready_(false),
      current_header_(reinterpret_cast<MessageHeader *>(current_rx_buffer_))
{
    for (unsigned i = 0; i < kMaxMessageLength; i++)
    {
        first_rx_buffer_[i] = 0;
        second_rx_buffer_[i] = 0;
    }
}

template <concepts::stream::ByteReadableStreamConcept RxStream,
          logger::LogDistributorConcept Distributor>
ReturnCode Deserializer<RxStream, Distributor>::Process()
{
    while (1) // TODO: bad practice
    {
        if (current_processed_length_ == 0)
        {
            ReturnCode res = FindHeader_();
            if (res != ReturnCode::OK)
            {
                return res;
            }
        }

        if (current_processed_length_ < sizeof(MessageHeader))
        {
            ReturnCode res = ParseHeader_();
            if (res == ReturnCode::FAIL)
            {
                continue;
            }
            if (res != ReturnCode::OK)
            {
                return res;
            }
        }

        current_processed_length_ +=
            read(rx_stream_, current_rx_buffer_ + current_processed_length_,
                 current_header_->length - current_processed_length_);

        if (current_processed_length_ != current_header_->length)
        {
            return ReturnCode::NO_DATA;
        }

        COBSDecoding(kMagicByte,
                     current_rx_buffer_ + sizeof(MessageHeader) -
                         sizeof(MessageHeader::cobs_length),
                     current_header_->length - sizeof(MessageHeader) +
                         sizeof(MessageHeader::cobs_length));

        current_processed_length_ = 0;
        if (CheckCRC_())
        {
            uint8_t *temp = current_rx_buffer_;
            current_rx_buffer_ = next_rx_buffer_;
            next_rx_buffer_ = temp;
            message_ready_ = true;
            return ReturnCode::OK;
        }
    }
}

template <concepts::stream::ByteReadableStreamConcept RxStream,
          logger::LogDistributorConcept Distributor>
AddressType Deserializer<RxStream, Distributor>::GetSourceAddress() const
{
    if (message_ready_)
    {
        MessageHeader *header =
            reinterpret_cast<MessageHeader *>(next_rx_buffer_);
        return header->src_address;
    }
    return 0;
}

template <concepts::stream::ByteReadableStreamConcept RxStream,
          logger::LogDistributorConcept Distributor>
const uint8_t *Deserializer<RxStream, Distributor>::GetData()
{
    if (message_ready_)
    {
        message_ready_ = false;
        return next_rx_buffer_ + sizeof(MessageHeader);
    }
    return nullptr;
}

template <concepts::stream::ByteReadableStreamConcept RxStream,
          logger::LogDistributorConcept Distributor>
unsigned Deserializer<RxStream, Distributor>::GetDataLength() const
{
    if (message_ready_)
    {
        MessageHeader *header =
            reinterpret_cast<MessageHeader *>(next_rx_buffer_);
        return header->length - sizeof(MessageHeader) - kCRCLength;
    }
    return 0;
}

template <concepts::stream::ByteReadableStreamConcept RxStream,
          logger::LogDistributorConcept Distributor>
void Deserializer<RxStream, Distributor>::COBSDecoding(uint8_t magic_byte,
                                                       uint8_t *data,
                                                       unsigned data_length)
{
    unsigned current_appearance = data[0];
    while (data[current_appearance] != 0)
    {
        unsigned next_appearance =
            current_appearance + data[current_appearance];
        data[current_appearance] = magic_byte;
        current_appearance = next_appearance;
        if (current_appearance > data_length)
        {
            return; // TODO: Make troubleshouting
        }
    }
    return;
}

template <concepts::stream::ByteReadableStreamConcept RxStream,
          logger::LogDistributorConcept Distributor>
ReturnCode Deserializer<RxStream, Distributor>::FindHeader_()
{
    int res = read(rx_stream_, current_rx_buffer_, sizeof(kMagicByte));
    while (res != 0)
    {
        if (res < 0)
        {
            return ReturnCode::ERROR;
        }
        if (current_rx_buffer_[0] == kMagicByte)
        {
            current_processed_length_ = sizeof(kMagicByte);
            return ReturnCode::OK;
        }
        res = read(rx_stream_, current_rx_buffer_, sizeof(kMagicByte));
    }
    return ReturnCode::NO_DATA;
}

template <concepts::stream::ByteReadableStreamConcept RxStream,
          logger::LogDistributorConcept Distributor>
ReturnCode Deserializer<RxStream, Distributor>::ParseHeader_()
{
    int res = read(rx_stream_, current_rx_buffer_ + current_processed_length_,
                   sizeof(MessageHeader) - current_processed_length_);
    if (res < 0)
    {
        return ReturnCode::ERROR;
    }
    current_processed_length_ += res;
    if (current_processed_length_ != sizeof(MessageHeader))
    {
        return ReturnCode::NO_DATA;
    }
    if (current_header_->dest_address != self_address_)
    {
        current_processed_length_ = 0;
        return ReturnCode::FAIL;
    }
    return ReturnCode::OK;
}

template <concepts::stream::ByteReadableStreamConcept RxStream,
          logger::LogDistributorConcept Distributor>
bool Deserializer<RxStream, Distributor>::CheckCRC_()
{
    uint8_t target_crc = crc::CountCRC8(current_rx_buffer_,
                                        current_header_->length - kCRCLength);

    uint8_t current_crc =
        current_rx_buffer_[current_header_->length - kCRCLength];

    if (target_crc != current_crc)
    {
        LOG(logger_, logger::LogLevel::WARNING,
                         "Wrong CRC: expected {}, got {}", target_crc,
                         current_crc);
        return false;
    }
    return true;
}

} // namespace hydrolib::bus::datalink
