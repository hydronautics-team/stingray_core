#pragma once

#include <cstdint>
#include <cstring>
#include <unistd.h>

#include "hydrolib_bus_application_commands.hpp"
#include "hydrolib_logger.hpp"
#include "hydrolib_return_codes.hpp"
#include "hydrolib_stream_concepts.hpp"

namespace hydrolib::bus::application
{
template <concepts::stream::ByteFullStreamConcept TxRxStream,
          logger::LogDistributorConcept Distributor>
class Master
{
public:
    constexpr Master(TxRxStream &stream, logger::Logger<Distributor> &logger);

public:
    bool Process();
    void Read(void *data, unsigned address, unsigned length);
    void Write(const void *data, unsigned address, unsigned length);

private:
    TxRxStream &stream_;
    logger::Logger<Distributor> &logger_;

    void *requested_data_;
    int requested_length_;

    uint8_t rx_buffer_[kMaxMessageLength];
    uint8_t tx_buffer_[kMaxMessageLength];
};

template <concepts::stream::ByteFullStreamConcept TxRxStream,
          logger::LogDistributorConcept Distributor>
constexpr Master<TxRxStream, Distributor>::Master(
    TxRxStream &stream, logger::Logger<Distributor> &logger)
    : stream_(stream),
      logger_(logger),
      requested_data_(nullptr),
      requested_length_(0)
{
    for (unsigned i = 0; i < kMaxMessageLength; i++)
    {
        rx_buffer_[i] = 0;
        tx_buffer_[i] = 0;
    }
}

template <concepts::stream::ByteFullStreamConcept TxRxStream,
          logger::LogDistributorConcept Distributor>
bool Master<TxRxStream, Distributor>::Process()
{
    if (!requested_data_)
    {
        return false;
    }

    int read_length = read(stream_, rx_buffer_, kMaxMessageLength);
    if (!read_length)
    {
        return false;
    }

    Command command = *reinterpret_cast<Command *>(rx_buffer_);
    switch (command)
    {
    case Command::RESPONSE:
        if (requested_length_ + static_cast<int>(sizeof(Command)) !=
            read_length)
        {
            return false;
        }
        memcpy(requested_data_, rx_buffer_ + sizeof(Command),
               requested_length_);
        requested_data_ = nullptr;
        return true;
    case Command::ERROR:
    case Command::READ:
    case Command::WRITE:
    default:
        LOG(logger_, logger::LogLevel::WARNING, "Wrong command");
        return false;
    }
}

template <concepts::stream::ByteFullStreamConcept TxRxStream,
          logger::LogDistributorConcept Distributor>
void Master<TxRxStream, Distributor>::Read(void *data, unsigned address,
                                           unsigned length)
{
    requested_data_ = data;
    requested_length_ = length;

    MemoryAccessHeader *header =
        reinterpret_cast<MemoryAccessHeader *>(tx_buffer_);
    header->command = Command::READ;
    header->info.address = address;
    header->info.length = length;

    write(stream_, tx_buffer_, sizeof(MemoryAccessHeader));
}

template <concepts::stream::ByteFullStreamConcept TxRxStream,
          logger::LogDistributorConcept Distributor>
void Master<TxRxStream, Distributor>::Write(const void *data, unsigned address,
                                            unsigned length)
{
    MemoryAccessHeader *header =
        reinterpret_cast<MemoryAccessHeader *>(tx_buffer_);
    header->command = Command::WRITE;
    header->info.address = address;
    header->info.length = length;

    memcpy(tx_buffer_ + sizeof(MemoryAccessHeader), data, length);

    write(stream_, tx_buffer_, sizeof(MemoryAccessHeader) + length);
}

} // namespace hydrolib::bus::application
