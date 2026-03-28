#pragma once

#include <cstdint>
#include <cstring>

#include "hydrolib_bus_application_commands.hpp"
#include "hydrolib_logger.hpp"
#include "hydrolib_return_codes.hpp"
#include "hydrolib_stream_concepts.hpp"

namespace hydrolib::bus::application
{
template <typename T>
concept PublicMemoryConcept =
    requires(T mem, void *read_buffer, const void *write_buffer,
             unsigned address, unsigned length) {
        { mem.Read(read_buffer, address, length) } -> std::same_as<ReturnCode>;

        {
            mem.Write(write_buffer, address, length)
        } -> std::same_as<ReturnCode>;
    };

template <PublicMemoryConcept Memory, logger::LogDistributorConcept Distributor,
          concepts::stream::ByteFullStreamConcept TxRxStream>
class Slave
{
public:
    constexpr Slave(TxRxStream &stream, Memory &memory,
                    logger::Logger<Distributor> &logger);

public:
    void Process();

private:
    TxRxStream &stream_;
    Memory &memory_;

    logger::Logger<Distributor> &logger_;

    uint8_t rx_buffer_[kMaxMessageLength];
    uint8_t tx_buffer_[kMaxMessageLength];
};

template <PublicMemoryConcept Memory, logger::LogDistributorConcept Distributor,
          concepts::stream::ByteFullStreamConcept TxRxStream>
constexpr Slave<Memory, Distributor, TxRxStream>::Slave(
    TxRxStream &stream, Memory &memory, logger::Logger<Distributor> &logger)
    : stream_(stream), memory_(memory), logger_(logger)
{
    for (unsigned i = 0; i < kMaxMessageLength; i++)
    {
        rx_buffer_[i] = 0;
    }
    for (unsigned i = 0; i < kMaxMessageLength; i++)
    {
        tx_buffer_[i] = 0;
    }
}

template <PublicMemoryConcept Memory, logger::LogDistributorConcept Distributor,
          concepts::stream::ByteFullStreamConcept TxRxStream>
void Slave<Memory, Distributor, TxRxStream>::Process()
{
    int read_length = read(stream_, rx_buffer_, kMaxMessageLength);
    if (!read_length)
    {
        return;
    }

    Command command = *reinterpret_cast<Command *>(rx_buffer_);
    switch (command)
    {
    case Command::READ:
    {
        MemoryAccessInfo *info =
            reinterpret_cast<MemoryAccessInfo *>(rx_buffer_ + sizeof(Command));
        ReturnCode res = memory_.Read(tx_buffer_ + sizeof(Command),
                                      info->address, info->length);
        if (res == ReturnCode::OK)
        {
            LOG(logger_, logger::LogLevel::INFO,
                             "Transmitting {} bytes from {}", info->length,
                             info->address);
            *reinterpret_cast<Command *>(tx_buffer_) = Command::RESPONSE;
            write(stream_, tx_buffer_, sizeof(Command) + info->length);
        }
        else
        {
            LOG(logger_, logger::LogLevel::WARNING,
                             "Can't read {} bytes from {}", info->length,
                             info->address);
            *reinterpret_cast<Command *>(tx_buffer_) = Command::ERROR;
            write(stream_, tx_buffer_, sizeof(Command));
        }
    }
    break;
    case Command::WRITE:
    {
        MemoryAccessInfo *info =
            reinterpret_cast<MemoryAccessInfo *>(rx_buffer_ + sizeof(Command));
        ReturnCode res = memory_.Write(rx_buffer_ + sizeof(Command) +
                                           sizeof(MemoryAccessInfo),
                                       info->address, info->length);
        if (res != ReturnCode::OK)
        {
            LOG(logger_, logger::LogLevel::WARNING,
                             "Can't write {} bytes to {}", info->length,
                             info->address);
            *reinterpret_cast<Command *>(tx_buffer_) = Command::ERROR;
            write(stream_, tx_buffer_, sizeof(Command));
        }
        else
        {
            LOG(logger_, logger::LogLevel::INFO, "Wrote {} bytes to {}",
                             info->length, info->address);
        }
    }
    break;
    case Command::ERROR:
    case Command::RESPONSE:
    default:
        LOG(logger_, logger::LogLevel::WARNING, "Wrong command");
        *reinterpret_cast<Command *>(tx_buffer_) = Command::ERROR;
        write(stream_, tx_buffer_, sizeof(Command));
        break;
    }
}
} // namespace hydrolib::bus::application
