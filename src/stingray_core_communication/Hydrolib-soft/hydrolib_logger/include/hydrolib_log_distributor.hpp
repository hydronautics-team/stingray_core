#ifndef HYDROLIB_LOG_DISTRIBUTOR_H_
#define HYDROLIB_LOG_DISTRIBUTOR_H_

#include "hydrolib_return_codes.hpp"
#include "hydrolib_cstring.hpp"
#include "hydrolib_log.hpp"
#include "hydrolib_stream_concepts.hpp"

namespace hydrolib::logger
{
template <concepts::stream::ByteWritableStreamConcept... Streams>
class LogDistributor
{
public:
    static constexpr size_t MAX_LOGGERS_COUNT = 50; // TODO make template
    static constexpr unsigned kMaxLogLength = 100;

private:
    template <typename Observer,
              concepts::stream::ByteWritableStreamConcept... Streams_>
    class LogDistributingNode_;

    template <typename NextNode,
              concepts::stream::ByteWritableStreamConcept Stream,
              concepts::stream::ByteWritableStreamConcept... Streams_>
    class LogDistributingNode_<NextNode, Stream, Streams_...>
        : public LogDistributingNode_<
              LogDistributingNode_<NextNode, Stream, Streams_...>, Streams_...>
    {
    public:
        consteval LogDistributingNode_(NextNode *next_node, Stream &stream,
                                       Streams_ &...streams);

    public:
        template <typename... Ts>
        bool Notify(unsigned source_id, LogLevel level);

        ReturnCode Push(const char *source, int length) const;

        ReturnCode SetLogFiltration(unsigned depth, unsigned logger_id,
                                             LogLevel level);

        void SetLogFiltrationsForAll(unsigned logger_id, LogLevel level);

    private:
        LogLevel level_filter_[MAX_LOGGERS_COUNT];
        Stream &output_stream_;

        bool stream_enable_;

        NextNode *next_node_;
    };

    template <typename NextNode>
    class LogDistributingNode_<NextNode>
    {
    public:
        consteval LogDistributingNode_(NextNode *next_node)
            : head_node(next_node)
        {
        }

    public:
        template <typename... Ts>
        bool Notify(
            [[maybe_unused]] unsigned source_id,
            [[maybe_unused]] LogLevel level) const // TODO workaround, need fix
        {
            return true;
        }

        ReturnCode Push([[maybe_unused]] const char *source,
                                 [[maybe_unused]] std::size_t length) const
        {
            return ReturnCode::OK;
        }

        ReturnCode
        SetLogFiltration([[maybe_unused]] unsigned depth,
                         [[maybe_unused]] unsigned logger_id,
                         [[maybe_unused]] LogLevel level)
        {
            return ReturnCode::OK;
        }

        void SetLogFiltrationsForAll([[maybe_unused]] unsigned logger_id,
                                     [[maybe_unused]] LogLevel level)
        {
        }

    public:
        NextNode *head_node;
    };

public:
    consteval LogDistributor(char *format_string, Streams &...streams);

public:
    template <typename... Ts>
    void Notify(unsigned source_id, Log<Ts...> &log, Ts... params) const;

    ReturnCode SetFilter(unsigned stream_number, unsigned logger_id,
                                  LogLevel level);
    ReturnCode SetAllFilters(unsigned logger_id, LogLevel level);

private:
    LogDistributingNode_<LogDistributingNode_<void>, Streams...>
        distributing_list_;

    char *format_string_; // TODO Make CString (and add coping)
};

template <concepts::stream::ByteWritableStreamConcept... Streams>
consteval LogDistributor<Streams...>::LogDistributor(char *format_string,
                                                     Streams &...streams)
    : distributing_list_(nullptr, streams...), format_string_(format_string)
{
}

template <concepts::stream::ByteWritableStreamConcept... Streams>
template <typename... Ts>
void LogDistributor<Streams...>::Notify(unsigned source_id, Log<Ts...> &log,
                                        Ts... params) const
{
    strings::CString<kMaxLogLength> log_buffer;
    if (distributing_list_.head_node->Notify(source_id, log.level))
    {
        log.ToBytes(format_string_, log_buffer, params...);
        distributing_list_.head_node->Push(log_buffer, log_buffer.GetLength());
    }
}

template <concepts::stream::ByteWritableStreamConcept... Streams>
ReturnCode
LogDistributor<Streams...>::SetFilter(unsigned stream_number,
                                      unsigned logger_id, LogLevel level)
{
    if (logger_id >= MAX_LOGGERS_COUNT)
    {
        return ReturnCode::FAIL;
    }
    return distributing_list_.head_node->SetLogFiltration(
        sizeof...(Streams) - 1 - stream_number, logger_id, level);
}

template <concepts::stream::ByteWritableStreamConcept... Streams>
ReturnCode
LogDistributor<Streams...>::SetAllFilters(unsigned logger_id, LogLevel level)
{
    if (logger_id >= MAX_LOGGERS_COUNT)
    {
        return ReturnCode::FAIL;
    }
    distributing_list_.head_node->SetLogFiltrationsForAll(logger_id, level);
    return ReturnCode::OK;
}

template <concepts::stream::ByteWritableStreamConcept... Streams>
template <typename NextNode, concepts::stream::ByteWritableStreamConcept Stream,
          concepts::stream::ByteWritableStreamConcept... Streams_>
consteval LogDistributor<Streams...>::LogDistributingNode_<
    NextNode, Stream, Streams_...>::LogDistributingNode_(NextNode *next_node,
                                                         Stream &stream,
                                                         Streams_ &...streams)
    : LogDistributingNode_<LogDistributingNode_<NextNode, Stream, Streams_...>,
                           Streams_...>(this, streams...),
      output_stream_(stream),
      stream_enable_(false),
      next_node_(next_node)
{
    for (size_t i = 0; i < MAX_LOGGERS_COUNT; i++)
    {
        level_filter_[i] = LogLevel::NO_LEVEL;
    }
}

template <concepts::stream::ByteWritableStreamConcept... Streams>
template <typename NextNode, concepts::stream::ByteWritableStreamConcept Stream,
          concepts::stream::ByteWritableStreamConcept... Streams_>
template <typename... Ts>
bool LogDistributor<Streams...>::LogDistributingNode_<
    NextNode, Stream, Streams_...>::Notify(unsigned source_id, LogLevel level)
{
    bool res = level_filter_[source_id] != LogLevel::NO_LEVEL &&
               level >= level_filter_[source_id];
    stream_enable_ = res;
    if (res)
    {
        if (next_node_)
        {
            next_node_->Notify(source_id, level);
        }
        return true;
    }
    if (next_node_)
    {
        return res || next_node_->Notify(source_id, level);
    }
    else
    {
        return res;
    }
}

template <concepts::stream::ByteWritableStreamConcept... Streams>
template <typename NextNode, concepts::stream::ByteWritableStreamConcept Stream,
          concepts::stream::ByteWritableStreamConcept... Streams_>
ReturnCode LogDistributor<Streams...>::LogDistributingNode_<
    NextNode, Stream, Streams_...>::Push(const char *source, int length) const
{
    ReturnCode self_res = ReturnCode::OK;
    ReturnCode other_res = ReturnCode::OK;
    if (next_node_)
    {
        other_res = next_node_->Push(source, length);
    }
    if (stream_enable_)
    {
        int write_count = write(output_stream_, source, length);
        if (write_count != length)
        {
            self_res = ReturnCode::ERROR;
        }
    }
    if (self_res != ReturnCode::OK)
    {
        return self_res;
    }
    if (other_res != ReturnCode::OK)
    {
        return other_res;
    }
    return ReturnCode::OK;
}

template <concepts::stream::ByteWritableStreamConcept... Streams>
template <typename NextNode, concepts::stream::ByteWritableStreamConcept Stream,
          concepts::stream::ByteWritableStreamConcept... Streams_>
ReturnCode LogDistributor<Streams...>::LogDistributingNode_<
    NextNode, Stream, Streams_...>::SetLogFiltration(unsigned depth,
                                                     unsigned logger_id,
                                                     LogLevel level)
{
    if (depth == 0)
    {
        level_filter_[logger_id] = level;
        return ReturnCode::OK;
    }
    if (next_node_)
    {
        return next_node_->SetLogFiltration(depth - 1, logger_id, level);
    }
    else
    {
        return ReturnCode::FAIL;
    }
}

template <concepts::stream::ByteWritableStreamConcept... Streams>
template <typename NextNode, concepts::stream::ByteWritableStreamConcept Stream,
          concepts::stream::ByteWritableStreamConcept... Streams_>
void LogDistributor<Streams...>::LogDistributingNode_<
    NextNode, Stream, Streams_...>::SetLogFiltrationsForAll(unsigned logger_id,
                                                            LogLevel level)
{
    level_filter_[logger_id] = level;
    if (next_node_)
    {
        next_node_->SetLogFiltrationsForAll(logger_id, level);
    }
}

} // namespace hydrolib::logger

#endif
