#ifndef HYDROLIB_SERIAL_PROTOCOL_INTERPRETER_H_
#define HYDROLIB_SERIAL_PROTOCOL_INTERPRETER_H_

#include <cstdint>
#include <cstring>

#include "hydrolib_common.h"
#include "hydrolib_logger.hpp"
#include "hydrolib_serial_protocol_commands.hpp"
#include "hydrolib_serial_protocol_message.hpp"

namespace hydrolib::serial_protocol
{

template <typename T>
concept PublicMemoryConcept =
    requires(T mem, void *read_buffer, const void *write_buffer,
             unsigned address, unsigned length) {
        {
            mem.Read(read_buffer, address, length)
        } -> std::same_as<hydrolib_ReturnCode>;

        {
            mem.Write(write_buffer, address, length)
        } -> std::same_as<hydrolib_ReturnCode>;
    };

template <typename T>
concept TransmitterConcept =
    requires(T transmitter, Command command, CommandInfo info) {
        {
            transmitter.Process(command, info)
        } -> std::same_as<hydrolib_ReturnCode>;
    };

template <PublicMemoryConcept Memory, logger::LogDistributorConcept Distributor,
          TransmitterConcept Transmitter>
class Interpreter
{
private:
    enum class FSMInput_
    {
        PROCESS_RECEIVE,
        REQUEST_TRANSMIT
    };
    enum class FSMState_
    {
        IDLE,
        WAITING_TO_RESPONCE
    };

public:
    constexpr Interpreter(Memory &memory, Transmitter &transmitter,
                          logger::Logger<Distributor> &logger);

    hydrolib_ReturnCode ProcessRx(Command command, CommandInfo command_info);
    hydrolib_ReturnCode TransmitWrite(unsigned device_address,
                                      unsigned memory_address,
                                      unsigned memory_length,
                                      const uint8_t *data);
    hydrolib_ReturnCode TransmitRead(unsigned device_address,
                                     unsigned memory_address,
                                     unsigned memory_length, uint8_t *data);

private:
    hydrolib_ReturnCode ProcessRequest_(Command command,
                                        CommandInfo command_info);
    hydrolib_ReturnCode ProcessWrite_(CommandInfo info);
    hydrolib_ReturnCode ProcessRead_(CommandInfo info);
    hydrolib_ReturnCode ProcessResponce_(Command command, CommandInfo info);

private:
    bool processing_master_;
    Command current_command_;
    CommandInfo current_command_info_;
    uint8_t read_buffer_[MessageHeader::MAX_MESSAGE_LENGTH];

private:
    logger::Logger<Distributor> &logger_;

    Memory &memory_;

    Transmitter &transmitter_;

    FSMState_ state_;

    unsigned responding_device_;
    uint8_t *responce_buffer_;
    unsigned responce_length_;
};

template <PublicMemoryConcept Memory, logger::LogDistributorConcept Distributor,
          TransmitterConcept Transmitter>
constexpr Interpreter<Memory, Distributor, Transmitter>::Interpreter(
    Memory &memory, Transmitter &transmitter,
    logger::Logger<Distributor> &logger)
    : logger_(logger),
      memory_(memory),
      transmitter_(transmitter),
      state_(FSMState_::IDLE),
      responding_device_(0),
      responce_buffer_(nullptr),
      responce_length_(0)
{
}

template <PublicMemoryConcept Memory, logger::LogDistributorConcept Distributor,
          TransmitterConcept Transmitter>
hydrolib_ReturnCode Interpreter<Memory, Distributor, Transmitter>::ProcessRx(
    Command command, CommandInfo command_info)
{
    hydrolib_ReturnCode res;
    switch (state_)
    {
    case FSMState_::IDLE:
        return ProcessRequest_(command, command_info);
    case FSMState_::WAITING_TO_RESPONCE:
        res = ProcessResponce_(command, command_info);
        if (res == HYDROLIB_RETURN_OK)
        {
            state_ = FSMState_::IDLE;
        }
        return res;
    default:
        return HYDROLIB_RETURN_ERROR;
    }
}

template <PublicMemoryConcept Memory, logger::LogDistributorConcept Distributor,
          TransmitterConcept Transmitter>
hydrolib_ReturnCode
Interpreter<Memory, Distributor, Transmitter>::TransmitWrite(
    unsigned device_address, unsigned memory_address, unsigned memory_length,
    const uint8_t *data)
{
    CommandInfo info;
    info.write = {.source_address =
                      0, // Correct address will bet set by serializer
                  .dest_address = device_address,
                  .memory_address = memory_address,
                  .memory_access_length = memory_length,
                  .data = data};
    hydrolib_ReturnCode res = transmitter_.Process(Command::WRITE, info);
    if (res == HYDROLIB_RETURN_OK)
    {
        state_ = FSMState_::WAITING_TO_RESPONCE;
    }
    return res;
}

template <PublicMemoryConcept Memory, logger::LogDistributorConcept Distributor,
          TransmitterConcept Transmitter>
hydrolib_ReturnCode Interpreter<Memory, Distributor, Transmitter>::TransmitRead(
    unsigned device_address, unsigned memory_address, unsigned memory_length,
    uint8_t *data)
{
    CommandInfo info;
    info.read = {.source_address =
                     0, // Correct address will bet set by serializer
                 .dest_address = device_address,
                 .memory_address = memory_address,
                 .memory_access_length = memory_length};
    hydrolib_ReturnCode res = transmitter_.Process(Command::READ, info);
    if (res == HYDROLIB_RETURN_OK)
    {
        state_ = FSMState_::WAITING_TO_RESPONCE;
        responding_device_ = device_address;
        responce_buffer_ = data;
        responce_length_ = memory_length;
    }
    return res;
}

template <PublicMemoryConcept Memory, logger::LogDistributorConcept Distributor,
          TransmitterConcept Transmitter>
hydrolib_ReturnCode
Interpreter<Memory, Distributor, Transmitter>::ProcessRequest_(
    Command command, CommandInfo command_info)
{
    switch (command)
    {
    case READ:
        return ProcessRead_(command_info);
    case WRITE:
        return ProcessWrite_(command_info);
    case RESPONCE:
    case ERROR:
        LOG(logger_, logger::LogLevel::WARNING, "Got unsupposed responce");
        return HYDROLIB_RETURN_FAIL;
    default:
        return HYDROLIB_RETURN_ERROR;
    }
}

template <PublicMemoryConcept Memory, logger::LogDistributorConcept Distributor,
          TransmitterConcept Transmitter>
hydrolib_ReturnCode
Interpreter<Memory, Distributor, Transmitter>::ProcessWrite_(CommandInfo info)
{
    hydrolib_ReturnCode res =
        memory_.Write(info.write.data, info.write.memory_address,
                      info.write.memory_access_length);
    if (res != HYDROLIB_RETURN_OK)
    {
        CommandInfo responce_info;
        responce_info.error = {.source_address = info.read.dest_address,
                               .dest_address = info.read.source_address,
                               .error_code = ErrorCode::BAD_ADDRESS};
        return transmitter_.Process(Command::ERROR, responce_info);
    }
    return HYDROLIB_RETURN_OK;
}

template <PublicMemoryConcept Memory, logger::LogDistributorConcept Distributor,
          TransmitterConcept Transmitter>
hydrolib_ReturnCode
Interpreter<Memory, Distributor, Transmitter>::ProcessRead_(CommandInfo info)
{

    hydrolib_ReturnCode read_res = memory_.Read(
        read_buffer_, info.read.memory_address, info.read.memory_access_length);
    if (read_res != HYDROLIB_RETURN_OK)
    {
        CommandInfo responce_info;
        responce_info.error = {.source_address = info.read.dest_address,
                               .dest_address = info.read.source_address,
                               .error_code = ErrorCode::BAD_ADDRESS};
        return transmitter_.Process(Command::ERROR, responce_info);
    }
    else
    {
        CommandInfo responce_info;
        responce_info.responce = {.source_address = info.read.dest_address,
                                  .dest_address = info.read.source_address,
                                  .data_length = info.read.memory_access_length,
                                  .data = read_buffer_};
        return transmitter_.Process(Command::RESPONCE, responce_info);
    }
}

template <PublicMemoryConcept Memory, logger::LogDistributorConcept Distributor,
          TransmitterConcept Transmitter>
hydrolib_ReturnCode
Interpreter<Memory, Distributor, Transmitter>::ProcessResponce_(
    Command command, CommandInfo info)
{
    switch (command)
    {
    case RESPONCE:
        if (responding_device_ != info.responce.source_address)
        {
            LOG(logger_, logger::LogLevel::WARNING,
                "Got responce from wrong address: supposed {}, got {}",
                responding_device_, info.responce.source_address);
            return HYDROLIB_RETURN_FAIL;
        }
        if (responce_length_ != info.responce.data_length)
        {
            LOG(logger_, logger::LogLevel::WARNING,
                "Got wrong responce length: supposed {}, got {}",
                responce_length_, info.responce.data_length);
            return HYDROLIB_RETURN_FAIL;
        }
        memcpy(responce_buffer_, info.responce.data, info.responce.data_length);
        return HYDROLIB_RETURN_OK;
    case ERROR:
        LOG(logger_, logger::LogLevel::WARNING, "Got error responce: {}",
            static_cast<unsigned>(info.error.error_code));
        return HYDROLIB_RETURN_FAIL;
    default:
        LOG(logger_, logger::LogLevel::WARNING,
            "Got request instead of responce");
        return HYDROLIB_RETURN_FAIL;
    }
}

} // namespace hydrolib::serial_protocol

#endif
