#pragma once

#include "hydrolib_bus_datalink_deserializer.hpp"
#include "hydrolib_bus_datalink_message.hpp"
#include "hydrolib_bus_datalink_serializer.hpp"
#include <cstring>

namespace hydrolib::bus::datalink
{
template <concepts::stream::ByteFullStreamConcept RxTxStream,
          logger::LogDistributorConcept Distributor, int MATES_COUNT = 3>
class Stream;

template <concepts::stream::ByteFullStreamConcept RxTxStream,
          logger::LogDistributorConcept Distributor, int MATES_COUNT = 3>
class StreamManager
{
    friend class Stream<RxTxStream, Distributor, MATES_COUNT>;

private:
    using SerializerType = Serializer<RxTxStream, Distributor>;
    using DeserializerType = Deserializer<RxTxStream, Distributor>;

public:
    constexpr StreamManager(AddressType self_address, RxTxStream &stream,
                            logger::Logger<Distributor> &logger);

public:
    void Process();

private:
    RxTxStream &stream_;
    logger::Logger<Distributor> &logger_;

    const AddressType self_address_;

    DeserializerType deserializer_;

    Stream<RxTxStream, Distributor, MATES_COUNT> *streams_[MATES_COUNT] = {
        nullptr};
    int streams_count_;
};

template <concepts::stream::ByteFullStreamConcept RxTxStream,
          logger::LogDistributorConcept Distributor, int MATES_COUNT>
class Stream
{
    friend class StreamManager<RxTxStream, Distributor, MATES_COUNT>;

public:
    constexpr Stream(
        StreamManager<RxTxStream, Distributor, MATES_COUNT> &stream_manager,
        AddressType mate_address);

public:
    int Read(void *dest, unsigned length);
    int Write(const void *dest, unsigned length);

private:
    StreamManager<RxTxStream, Distributor, MATES_COUNT> &stream_manager_;
    const AddressType mate_address_;

    uint8_t buffer[kMaxMessageLength] = {0}; // TODO: Make queue
    unsigned message_length_;
};

template <concepts::stream::ByteFullStreamConcept RxTxStream,
          logger::LogDistributorConcept Distributor, int MATES_COUNT>
constexpr StreamManager<RxTxStream, Distributor, MATES_COUNT>::StreamManager(
    AddressType self_address, RxTxStream &stream,
    logger::Logger<Distributor> &logger)
    : stream_(stream),
      logger_(logger),
      self_address_(self_address),
      deserializer_(self_address, stream, logger),
      streams_count_(0)
{
}

template <concepts::stream::ByteFullStreamConcept RxTxStream,
          logger::LogDistributorConcept Distributor, int MATES_COUNT>
void StreamManager<RxTxStream, Distributor, MATES_COUNT>::Process()
{
    ReturnCode result = deserializer_.Process();
    if (result == ReturnCode::OK)
    {
        AddressType message_source_address = deserializer_.GetSourceAddress();
        unsigned message_length = deserializer_.GetDataLength();
        const uint8_t *message_data = deserializer_.GetData();

        for (int i = 0; i < streams_count_; i++)
        {
            if (streams_[i]->mate_address_ == message_source_address)
            {
                std::memcpy(streams_[i]->buffer, message_data, message_length);
                streams_[i]->message_length_ = message_length;
                break;
            }
        }
    }
}

template <concepts::stream::ByteFullStreamConcept RxTxStream,
          logger::LogDistributorConcept Distributor, int MATES_COUNT>
constexpr Stream<RxTxStream, Distributor, MATES_COUNT>::Stream(
    StreamManager<RxTxStream, Distributor, MATES_COUNT> &stream_manager,
    AddressType mate_address)
    : stream_manager_(stream_manager),
      mate_address_(mate_address),
      message_length_(0)
{
    stream_manager.streams_[stream_manager.streams_count_] = this;
    stream_manager.streams_count_++;
}

template <concepts::stream::ByteFullStreamConcept RxTxStream,
          logger::LogDistributorConcept Distributor, int MATES_COUNT>
int Stream<RxTxStream, Distributor, MATES_COUNT>::Read(void *dest,
                                                       unsigned length)
{
    if (length > message_length_)
    {
        length = message_length_;
    }
    std::memcpy(dest, buffer, length);
    return length;
}

template <concepts::stream::ByteFullStreamConcept RxTxStream,
          logger::LogDistributorConcept Distributor, int MATES_COUNT>
int Stream<RxTxStream, Distributor, MATES_COUNT>::Write(const void *dest,
                                                        unsigned length)
{
    typename StreamManager<RxTxStream, Distributor, MATES_COUNT>::SerializerType
        serializer(stream_manager_.self_address_, stream_manager_.stream_,
                   stream_manager_.logger_);
    ReturnCode result = serializer.Process(mate_address_, dest, length);

    if (result == ReturnCode::OK)
    {
        return length;
    }

    return 0;
}

template <hydrolib::concepts::stream::ByteFullStreamConcept RxTxStream,
          hydrolib::logger::LogDistributorConcept Distributor,
          int MATES_COUNT = 3>
int read(typename hydrolib::bus::datalink::Stream<RxTxStream, Distributor,
                                                  MATES_COUNT> &stream,
         void *dest, unsigned length);

template <hydrolib::concepts::stream::ByteFullStreamConcept RxTxStream,
          hydrolib::logger::LogDistributorConcept Distributor,
          int MATES_COUNT = 3>
int write(typename hydrolib::bus::datalink::Stream<RxTxStream, Distributor,
                                                   MATES_COUNT> &stream,
          const void *dest, unsigned length);

template <hydrolib::concepts::stream::ByteFullStreamConcept RxTxStream,
          hydrolib::logger::LogDistributorConcept Distributor, int MATES_COUNT>
int read(typename hydrolib::bus::datalink::Stream<RxTxStream, Distributor,
                                                  MATES_COUNT> &stream,
         void *dest, unsigned length)
{
    return stream.Read(dest, length);
}

template <hydrolib::concepts::stream::ByteFullStreamConcept RxTxStream,
          hydrolib::logger::LogDistributorConcept Distributor, int MATES_COUNT>
int write(typename hydrolib::bus::datalink::Stream<RxTxStream, Distributor,
                                                   MATES_COUNT> &stream,
          const void *dest, unsigned length)
{
    return stream.Write(dest, length);
}

} // namespace hydrolib::bus::datalink
