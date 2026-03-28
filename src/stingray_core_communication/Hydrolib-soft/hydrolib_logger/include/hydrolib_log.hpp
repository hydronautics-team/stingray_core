#ifndef HYDROLIB_LOG_H_
#define HYDROLIB_LOG_H_

#include <cstring>

#include "hydrolib_return_codes.hpp"
#include "hydrolib_cstring.hpp"
#include "hydrolib_formatable_string.hpp"

namespace hydrolib::logger
{

enum class LogLevel
{
    NO_LEVEL = 0,
    DEBUG = 1,
    INFO = 2,
    WARNING = 3,
    ERROR = 4,
    CRITICAL = 5
};

class LogInfo
{
public:
    constexpr static size_t MAX_NAME_LENGTH = 50;
    constexpr static size_t MAX_FORMAT_STRING_LENGTH = 20;
    constexpr static int TRANSLATION_ERROR = -1;

    constexpr static char DEBUG_STR[] = "DEBUG";
    constexpr static char INFO_STR[] = "INFO";
    constexpr static char WARNING_STR[] = "WARNING";
    constexpr static char ERROR_STR[] = "ERROR";
    constexpr static char CRITICAL_STR[] = "CRITICAL";

public:
    enum SpecialSymbols
    {
        MESSAGE = 'm',
        SOURCE_PROCESS = 's',
        LEVEL = 'l'
    };
};

template <typename... ArgTypes>
class Log
{

public:
    template <concepts::stream::ByteWritableStreamConcept DestType>
    ReturnCode ToBytes(const char *format_string, DestType &buffer,
                       ArgTypes... other) const;

private:
    template <concepts::stream::ByteWritableStreamConcept DestType>
    ReturnCode TranslateMessage_(DestType &buffer, ArgTypes... other) const;

    template <concepts::stream::ByteWritableStreamConcept DestType>
    ReturnCode TranslateLevel_(DestType &buffer) const;

    template <concepts::stream::ByteWritableStreamConcept DestType>
    ReturnCode TranslateSource_(DestType &buffer) const;

    template <concepts::stream::ByteWritableStreamConcept DestType>
    ReturnCode BurstWrite_(DestType &buffer, const char *burst_start,
                           int &burst_length) const;

public:
    strings::StaticFormatableString<ArgTypes...> message;
    LogLevel level;
    const strings::CString<LogInfo::MAX_NAME_LENGTH> *process_name;
};

template <typename... ArgTypes>
template <concepts::stream::ByteWritableStreamConcept DestType>
ReturnCode Log<ArgTypes...>::ToBytes(const char *format_string,
                                     DestType &buffer, ArgTypes... other) const
{
    unsigned format_index = 0;

    const char *burst_start = nullptr;
    int burst_length = 0;

    while (format_string[format_index])
    {
        if (format_string[format_index] == '%')
        {
            ReturnCode burst_res =
                BurstWrite_(buffer, burst_start, burst_length);
            if (burst_res != ReturnCode::OK)
            {
                return burst_res;
            }

            format_index++;

            switch (format_string[format_index])
            {
            case LogInfo::SpecialSymbols::MESSAGE:
            {
                ReturnCode message_res = TranslateMessage_(buffer, other...);
                if (message_res != ReturnCode::OK)
                {
                    return message_res;
                }
                format_index++;
                break;
            }

            case LogInfo::SpecialSymbols::LEVEL:
            {
                ReturnCode level_res = TranslateLevel_(buffer);
                if (level_res != ReturnCode::OK)
                {
                    return level_res;
                }
                format_index++;
                break;
            }
            case LogInfo::SpecialSymbols::SOURCE_PROCESS:
            {
                ReturnCode source_res = TranslateSource_(buffer);
                if (source_res != ReturnCode::OK)
                {
                    return source_res;
                }
                format_index++;
                break;
            }
            default:
                return ReturnCode::ERROR;
            }
        }
        else
        {
            if (burst_length == 0)
            {
                burst_start = format_string + format_index;
            }
            burst_length++;
            format_index++;
        }
    }
    ReturnCode burst_res = BurstWrite_(buffer, burst_start, burst_length);
    if (burst_res != ReturnCode::OK)
    {
        return burst_res;
    }
    return ReturnCode::OK;
}

template <typename... ArgTypes>
template <concepts::stream::ByteWritableStreamConcept DestType>
ReturnCode Log<ArgTypes...>::TranslateMessage_(DestType &buffer,
                                               ArgTypes... other) const
{
    return message.ToBytes(buffer, other...);
}

template <typename... ArgTypes>
template <concepts::stream::ByteWritableStreamConcept DestType>
ReturnCode Log<ArgTypes...>::TranslateLevel_(DestType &buffer) const
{
    const char *level_str;
    int str_length;
    switch (level)
    {
    case LogLevel::DEBUG:
        level_str = LogInfo::DEBUG_STR;
        str_length = sizeof(LogInfo::DEBUG_STR) - 1;
        break;
    case LogLevel::INFO:
        level_str = LogInfo::INFO_STR;
        str_length = sizeof(LogInfo::INFO_STR) - 1;
        break;
    case LogLevel::WARNING:
        level_str = LogInfo::WARNING_STR;
        str_length = sizeof(LogInfo::WARNING_STR) - 1;
        break;
    case LogLevel::ERROR:
        level_str = LogInfo::ERROR_STR;
        str_length = sizeof(LogInfo::ERROR_STR) - 1;
        break;
    case LogLevel::CRITICAL:
        level_str = LogInfo::CRITICAL_STR;
        str_length = sizeof(LogInfo::CRITICAL_STR) - 1;
        break;
    default:
        return ReturnCode::ERROR;
    };

    int res = write(buffer, level_str, str_length);
    if (res == -1)
    {
        return ReturnCode::OVERFLOW;
    }
    if (res != str_length)
    {
        return ReturnCode::ERROR;
    }
    return ReturnCode::OK;
}

template <typename... ArgTypes>
template <concepts::stream::ByteWritableStreamConcept DestType>
ReturnCode Log<ArgTypes...>::TranslateSource_(DestType &buffer) const
{
    int source_length = process_name->GetLength();

    int res = write(buffer, process_name, source_length);
    if (res == -1)
    {
        return ReturnCode::OVERFLOW;
    }
    if (res != source_length)
    {
        return ReturnCode::ERROR;
    }
    return ReturnCode::OK;
}

template <typename... ArgTypes>
template <concepts::stream::ByteWritableStreamConcept DestType>
ReturnCode Log<ArgTypes...>::BurstWrite_(DestType &buffer,
                                         const char *burst_start,
                                         int &burst_length) const
{
    if (burst_length != 0)
    {
        int res = write(buffer, burst_start, burst_length);
        if (res == -1)
        {
            return ReturnCode::OVERFLOW;
        }
        if (res != burst_length)
        {
            return ReturnCode::ERROR;
        }
        burst_length = 0;
    }
    return ReturnCode::OK;
}

} // namespace hydrolib::logger

#endif
