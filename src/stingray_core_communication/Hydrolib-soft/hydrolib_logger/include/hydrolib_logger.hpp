#ifndef HYDROLIB_LOGGER_H_
#define HYDROLIB_LOGGER_H_

#include "hydrolib_cstring.hpp"
#include "hydrolib_formatable_string.hpp"
#include "hydrolib_log.hpp"

#define LOG(logger_, level, message, ...)                                      \
    hydrolib::logger::LoggingCase(logger_ __VA_OPT__(,) __VA_ARGS__)                        \
        .WriteLog(level, message __VA_OPT__(,) __VA_ARGS__)

namespace hydrolib::logger
{
template <typename T, typename... Ts>
concept LogDistributorConcept =
    requires(T distributor, unsigned source_id, Log<Ts...> &log, Ts... params) {
        distributor.Notify(source_id, log, params...);
    };

template <LogDistributorConcept Distributor>
class Logger
{
public:
    constexpr Logger(const char *name, unsigned id,
                     const Distributor &distributor)
        : name_(name), id_(id), distributor_(distributor)
    {
    }

public:
    template <typename... Ts>
    void WriteLog(LogLevel level,
                  strings::StaticFormatableString<Ts...> message, Ts... params)
    {
        Log<Ts...> log{
            .message = message, .level = level, .process_name = &name_};

        distributor_.Notify(id_, log, params...);
    }

private:
    const strings::CString<LogInfo::MAX_NAME_LENGTH> name_;
    const unsigned id_;

    const Distributor &distributor_;
};

template <LogDistributorConcept Distributor, typename... Ts>
class LoggingCase
{
public:
    constexpr LoggingCase(Logger<Distributor> &logger,
                          [[maybe_unused]] Ts... params)
        : logger_(logger)
    {
    }

public:
    void WriteLog(LogLevel level,
                  strings::StaticFormatableString<Ts...> message, Ts... params)
    {
        logger_.WriteLog(level, message, params...);
    }

public:
    Logger<Distributor> &logger_;
};
} // namespace hydrolib::logger

#endif
