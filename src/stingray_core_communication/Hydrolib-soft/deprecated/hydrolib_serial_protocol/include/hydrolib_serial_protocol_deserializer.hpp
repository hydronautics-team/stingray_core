#ifndef HYDROLIB_SERIAL_PROTOCOL_DESERIALIZER_H_
#define HYDROLIB_SERIAL_PROTOCOL_DESERIALIZER_H_

#include <cstdint>

#include "hydrolib_common.h"
#include "hydrolib_logger.hpp"
#include "hydrolib_queue_concepts.hpp"
#include "hydrolib_serial_protocol_commands.hpp"
#include "hydrolib_serial_protocol_message.hpp"
#include "hydrolib_stream_concepts.hpp"

namespace hydrolib::serial_protocol
{

template <concepts::stream::ByteReadableStreamConcept RxStream,
          logger::LogDistributorConcept Distributor>
class Deserializer
{
public:
    constexpr Deserializer(uint8_t address, RxStream &rx_stream,
                           logger::Logger<Distributor> &logger,
                           uint8_t network = 0xA0);

public:
    hydrolib_ReturnCode Process();

    Command GetCommand();
    CommandInfo GetInfo();

private:
    hydrolib_ReturnCode AimHeader_();
    hydrolib_ReturnCode ProcessCommonHeader_();
    hydrolib_ReturnCode ProcessMessage_();
    bool CheckCRC_();

private:
    logger::Logger<Distributor> &logger_;

    const uint8_t network_;
    const uint8_t self_readable_address_;

    const uint8_t self_address_;

    RxStream &rx_stream_;

    unsigned current_message_length_;
    unsigned current_processed_length_;
    uint8_t rx_buffer_[2 * MessageHeader::MAX_MESSAGE_LENGTH]; // TODO: Remove second buffer
    int offset_;
    int rx_buffer_length_;
    int last_message_offset_;

    bool message_ready_;

    Command current_command_;

    MessageHeader *current_header_;
};

template <concepts::stream::ByteReadableStreamConcept RxStream,
          logger::LogDistributorConcept Distributor>
constexpr Deserializer<RxStream, Distributor>::Deserializer(
    uint8_t address, RxStream &rx_stream, logger::Logger<Distributor> &logger,
    uint8_t network)
    : logger_(logger),
      network_(network),
      self_readable_address_(address),
      self_address_(MessageHeader::GetTrueAddress(address, network)),
      rx_stream_(rx_stream),
      current_message_length_(0),
      current_processed_length_(0),
      offset_(0),
      rx_buffer_length_(0),
      last_message_offset_(0),
      message_ready_(false),
      current_command_(Command::RESPONCE),
      current_header_(reinterpret_cast<MessageHeader *>(&rx_buffer_))
{
    for (unsigned i = 0; i < 2 * MessageHeader::MAX_MESSAGE_LENGTH; i++)
    {
        rx_buffer_[i] = 0;
    }
}

template <concepts::stream::ByteReadableStreamConcept RxStream,
          logger::LogDistributorConcept Distributor>
Command Deserializer<RxStream, Distributor>::GetCommand()
{
    MessageHeader *message_header =
        reinterpret_cast<MessageHeader *>(rx_buffer_ + last_message_offset_);
    return static_cast<Command>(message_header->common.command);
}

template <concepts::stream::ByteReadableStreamConcept RxStream,
          logger::LogDistributorConcept Distributor>
CommandInfo Deserializer<RxStream, Distributor>::GetInfo()
{
    CommandInfo info;

    MessageHeader *message_header =
        reinterpret_cast<MessageHeader *>(rx_buffer_ + last_message_offset_);

    switch (message_header->common.command)
    {
    case Command::READ:
        info.read = {.source_address = MessageHeader::GetReadableAddress(
                         message_header->memory_access.self_address),
                     .dest_address = self_readable_address_,
                     .memory_address =
                         message_header->memory_access.memory_address,
                     .memory_access_length =
                         message_header->memory_access.memory_access_length};
        break;
    case Command::WRITE:
        info.write = {.source_address = MessageHeader::GetReadableAddress(
                          message_header->memory_access.self_address),
                      .dest_address = self_readable_address_,
                      .memory_address =
                          message_header->memory_access.memory_address,
                      .memory_access_length =
                          message_header->memory_access.memory_access_length,
                      .data = rx_buffer_ + last_message_offset_ +
                              sizeof(MessageHeader::MemoryAccess)};
        break;
    case Command::RESPONCE:
        info.responce = {
            .source_address = MessageHeader::GetReadableAddress(
                message_header->responce.self_address),
            .dest_address = self_readable_address_,
            .data_length = message_header->responce.message_length -
                           static_cast<uint8_t>(sizeof(MessageHeader::Common)) -
                           MessageHeader::CRC_LENGTH,
            .data = rx_buffer_ + last_message_offset_ +
                    sizeof(MessageHeader::Common)};
        break;
    case Command::ERROR:
        info.error = {.source_address = MessageHeader::GetReadableAddress(
                          message_header->error.self_address),
                      .dest_address = self_readable_address_,
                      .error_code = static_cast<ErrorCode>(
                          message_header->error.error_code)};
        break;
    }

    message_ready_ = false;

    return info;
}

template <concepts::stream::ByteReadableStreamConcept RxStream,
          logger::LogDistributorConcept Distributor>
hydrolib_ReturnCode Deserializer<RxStream, Distributor>::Process()
{
    if (message_ready_)
    {
        return HYDROLIB_RETURN_FAIL;
    }
    bool message_found = false;
    while (!message_found)
    {
        if (current_processed_length_ < sizeof(self_address_))
        {
            hydrolib_ReturnCode header_search_res = AimHeader_();
            if (header_search_res != HYDROLIB_RETURN_OK)
            {
                return header_search_res;
            }
        }
        if (current_processed_length_ < sizeof(MessageHeader::Common))
        {
            hydrolib_ReturnCode header_process_res = ProcessCommonHeader_();
            if (header_process_res == HYDROLIB_RETURN_FAIL)
            {
                continue;
            }
            if (header_process_res != HYDROLIB_RETURN_OK)
            {
                return header_process_res;
            }
        }
        hydrolib_ReturnCode message_process_res = ProcessMessage_();
        if (message_process_res == HYDROLIB_RETURN_FAIL)
        {
            continue;
        }
        if (message_process_res != HYDROLIB_RETURN_OK)
        {
            return message_process_res;
        }

        message_found = CheckCRC_();
    }

    last_message_offset_ = offset_;
    message_ready_ = true;
    offset_ += current_header_->common.message_length;

    return HYDROLIB_RETURN_OK;
}

template <concepts::stream::ByteReadableStreamConcept RxStream,
          logger::LogDistributorConcept Distributor>
hydrolib_ReturnCode Deserializer<RxStream, Distributor>::AimHeader_()
{
    int read_byte_count = sizeof(self_address_);

    if (rx_buffer_length_ - offset_ <
        static_cast<int>(sizeof(
            self_address_))) // TODO: >Rework for more then one byte of header
    {
        if (rx_buffer_length_ <= offset_)
        {
            offset_ = 0;
            rx_buffer_length_ = 0;
        }
        read_byte_count =
            read(rx_stream_, &rx_buffer_[offset_], sizeof(self_address_));
        rx_buffer_length_ += read_byte_count;
    }
    while (read_byte_count == sizeof(self_address_))
    {
        if (rx_buffer_[offset_] == self_address_)
        {
            current_processed_length_ = sizeof(self_address_);
            return HYDROLIB_RETURN_OK;
        }
        else
        {
            offset_++;
            LOG(logger_, logger::LogLevel::WARNING, "Rubbish bytes");
        }

        if (rx_buffer_length_ - offset_ <
            static_cast<int>(sizeof(self_address_)))
        {
            if (rx_buffer_length_ <= offset_)
            {
                offset_ = 0;
                rx_buffer_length_ = 0;
            }
            read_byte_count =
                read(rx_stream_, &rx_buffer_[offset_], sizeof(self_address_));
            rx_buffer_length_ += read_byte_count;
        }
    }
    return HYDROLIB_RETURN_NO_DATA;
}

template <concepts::stream::ByteReadableStreamConcept RxStream,
          logger::LogDistributorConcept Distributor>
hydrolib_ReturnCode Deserializer<RxStream, Distributor>::ProcessCommonHeader_()
{
    int required_length = static_cast<int>(sizeof(MessageHeader::Common)) -
                          (rx_buffer_length_ - offset_);
    if (required_length > 0)
    {
        int read_byte_count =
            read(rx_stream_, &rx_buffer_[rx_buffer_length_], required_length);
        rx_buffer_length_ += read_byte_count;

        if (read_byte_count != required_length)
        {
            return HYDROLIB_RETURN_NO_DATA;
        }
    }

    current_processed_length_ = sizeof(MessageHeader::Common);
    current_header_ = reinterpret_cast<MessageHeader *>(rx_buffer_ + offset_);

    // if (current_header_->common.message_length <
    //         sizeof(MessageHeader::Common) ||
    //     (current_header_->common.message_length >
    //      MessageHeader::MAX_MESSAGE_LENGTH))
    if (current_header_->common.message_length < sizeof(MessageHeader::Common))
    {
        LOG(logger_, logger::LogLevel::WARNING,
            "Wrong message length: {}", current_header_->common.message_length);
        offset_++;
        current_processed_length_ = 0;

        return HYDROLIB_RETURN_FAIL;
    }

    return HYDROLIB_RETURN_OK;
}

template <concepts::stream::ByteReadableStreamConcept RxStream,
          logger::LogDistributorConcept Distributor>
hydrolib_ReturnCode Deserializer<RxStream, Distributor>::ProcessMessage_()
{
    int required_length =
        static_cast<int>(current_header_->common.message_length) -
        (rx_buffer_length_ - offset_);
    if (required_length > 0)
    {
        int read_byte_count =
            read(rx_stream_, &rx_buffer_[rx_buffer_length_], required_length);
        rx_buffer_length_ += read_byte_count;

        if (read_byte_count != required_length)
        {
            return HYDROLIB_RETURN_NO_DATA;
        }
    }

    if (current_header_->common.command == Command::WRITE)
    {
        unsigned target_length = current_header_->common.message_length -
                                 sizeof(MessageHeader::MemoryAccess) -
                                 MessageHeader::CRC_LENGTH;
        if (current_header_->memory_access.memory_access_length !=
            target_length)
        {
            LOG(logger_, logger::LogLevel::WARNING,
                "Wrong data length: expected {}, in header {}", target_length,
                current_header_->memory_access.memory_access_length);
            offset_++;
            current_processed_length_ = 0;
            return HYDROLIB_RETURN_FAIL;
        }
    } // TODO сделать такде для всез остальных комманд
    else if (current_header_->common.command != Command::READ &&
             current_header_->common.command != Command::RESPONCE &&
             current_header_->common.command != Command::ERROR)
    {
        LOG(logger_, logger::LogLevel::WARNING, "Wrong command: {}",
            static_cast<unsigned>(current_header_->common.command));
        offset_++;
        current_processed_length_ = 0;
        return HYDROLIB_RETURN_FAIL;
    }
    current_processed_length_ = 0;
    return HYDROLIB_RETURN_OK;
}

template <concepts::stream::ByteReadableStreamConcept RxStream,
          logger::LogDistributorConcept Distributor>
bool Deserializer<RxStream, Distributor>::CheckCRC_()
{
    uint8_t target_crc = MessageHeader::CountCRC(
        rx_buffer_ + offset_,
        current_header_->common.message_length - MessageHeader::CRC_LENGTH);

    uint8_t current_crc = rx_buffer_[current_header_->common.message_length -
                                     MessageHeader::CRC_LENGTH + offset_];

    if (target_crc != current_crc)
    {
        LOG(logger_, logger::LogLevel::WARNING,
                         "Wrong CRC: expected {}, got {}", target_crc,
                         current_crc);
        offset_++;
        return false;
    }
    return true;
}

} // namespace hydrolib::serial_protocol

#endif
