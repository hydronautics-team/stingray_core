#ifndef HYDROLIB_FORMATABLE_STRTING_H_
#define HYDROLIB_FORMATABLE_STRTING_H_

#include <cassert>
#include <concepts>
#include <cstring>

#include "hydrolib_fixed_point.hpp"
#include "hydrolib_return_codes.hpp"
#include "hydrolib_stream_concepts.hpp"

namespace hydrolib::strings
{
template <typename T>
concept StringConsept = requires(T string) {
    { static_cast<const char *>(string) };
    { string.GetLength() } -> std::convertible_to<std::size_t>;
};

template <typename... ArgTypes>
class StaticFormatableString
{
public:
    static constexpr unsigned MAX_PARAMETERS_COUNT = 10;

public:
    constexpr StaticFormatableString()
        : string_(nullptr), length_(0), param_count_(0) {};
    consteval StaticFormatableString(const char *string);

public:
    [[deprecated]]
    const char *GetString() const;
    unsigned GetLength() const;

    template <concepts::stream::ByteWritableStreamConcept DestType>
    ReturnCode ToBytes(DestType &buffer, ArgTypes... params) const;

    constexpr operator const char *() const;

private:
    template <concepts::stream::ByteWritableStreamConcept DestType,
              typename... Ts>
    ReturnCode ToBytes_(DestType &buffer, unsigned next_param_index,
                        int translated_length, int param, Ts... others) const;

    template <concepts::stream::ByteWritableStreamConcept DestType,
              StringConsept String, typename... Ts>
    ReturnCode ToBytes_(DestType &buffer, unsigned next_param_index,
                        unsigned translated_length, String param, Ts...) const;

    template <concepts::stream::ByteWritableStreamConcept DestType,
              typename... Ts>
    ReturnCode ToBytes_(DestType &buffer, unsigned next_param_index,
                        unsigned translated_length, math::FixedPointBase param,
                        Ts...) const;

    template <concepts::stream::ByteWritableStreamConcept DestType>
    ReturnCode ToBytes_(DestType &buffer, unsigned next_param_index,
                        int translated_length) const;

private:
    template <concepts::stream::ByteWritableStreamConcept DestType>
    static ReturnCode WriteIntegerToBuffer_(DestType &buffer, int param);

    static consteval unsigned CountLength_(const char *string);
    static consteval unsigned CountParametres_(const char *string);

private:
    const char *const string_;
    const int length_;
    const int param_count_;
    unsigned short param_pos_diffs_[MAX_PARAMETERS_COUNT];
};

inline void Error(const char *);

inline void Error(const char *)
{
    int a = 0;
    [[maybe_unused]] int b = 1 / a;
};

template <typename... ArgTypes>
consteval StaticFormatableString<ArgTypes...>::StaticFormatableString(
    const char *string)
    : string_(string),
      length_(CountLength_(string)),
      param_count_(CountParametres_(string))
{
    if (sizeof...(ArgTypes) != CountParametres_(string))
    {
        Error("Not enough arguments for inserting to formatable string");
    }
    unsigned param_number = 0;
    unsigned last_param = 0;
    for (int i = 0; i < length_; i++)
    {
        if (string_[i] == '{')
        {
            param_pos_diffs_[param_number] = i - last_param;
            param_number++;
        }
        else if (string_[i] == '}')
        {
            last_param = i + 1;
        }
    }
    for (unsigned i = param_number; i < MAX_PARAMETERS_COUNT; i++)
    {
        param_pos_diffs_[i] = 0;
    }
}

template <typename... ArgTypes>
template <concepts::stream::ByteWritableStreamConcept DestType>
ReturnCode
StaticFormatableString<ArgTypes...>::ToBytes(DestType &buffer,
                                             ArgTypes... params) const
{
    return ToBytes_(buffer, 0, 0, params...);
}

template <typename... ArgTypes>
constexpr StaticFormatableString<ArgTypes...>::operator const char *() const
{
    return string_;
}

template <typename... ArgTypes>
template <concepts::stream::ByteWritableStreamConcept DestType, typename... Ts>
ReturnCode StaticFormatableString<ArgTypes...>::ToBytes_(
    DestType &buffer, unsigned next_param_index, int translated_length,
    int param, Ts... others) const
{
    if (translated_length >= length_)
    {
        return ReturnCode::OK;
    }

    int write_res = write(buffer, string_ + translated_length,
                          param_pos_diffs_[next_param_index]);
    if (write_res == -1)
    {
        return ReturnCode::ERROR;
    }
    if (write_res != param_pos_diffs_[next_param_index])
    {
        return ReturnCode::OVERFLOW;
    }
    translated_length += param_pos_diffs_[next_param_index] + 2;
    next_param_index++;

    auto write_integer_result = WriteIntegerToBuffer_(buffer, param);
    if (write_integer_result != ReturnCode::OK)
    {
        return write_integer_result;
    }

    return ToBytes_(buffer, next_param_index, translated_length, others...);
}

template <typename... ArgTypes>
template <concepts::stream::ByteWritableStreamConcept DestType,
          StringConsept String, typename... Ts>
ReturnCode StaticFormatableString<ArgTypes...>::ToBytes_(
    DestType &buffer, unsigned next_param_index, unsigned translated_length,
    String param, Ts... others) const
{
    if (translated_length >= length_)
    {
        return ReturnCode::OK;
    }

    int write_res = write(buffer, string_ + translated_length,
                          param_pos_diffs_[next_param_index]);
    if (write_res == -1)
    {
        return ReturnCode::ERROR;
    }
    if (write_res != param_pos_diffs_[next_param_index])
    {
        return ReturnCode::OVERFLOW;
    }
    translated_length += param_pos_diffs_[next_param_index] + 2;
    next_param_index++;

    write_res =
        write(buffer, static_cast<const char *>(param), param.GetLength());
    if (write_res == -1)
    {
        return ReturnCode::ERROR;
    }
    if (write_res != param.GetLength())
    {
        return ReturnCode::OVERFLOW;
    }

    return ToBytes_(buffer, next_param_index, translated_length, others...);
}

template <typename... ArgTypes>
template <concepts::stream::ByteWritableStreamConcept DestType, typename... Ts>
ReturnCode StaticFormatableString<ArgTypes...>::ToBytes_(
    DestType &buffer, unsigned next_param_index, unsigned translated_length,
    math::FixedPointBase param, Ts... others) const
{
    if (translated_length >= length_)
    {
        return ReturnCode::OK;
    }

    int write_res = write(buffer, string_ + translated_length,
                          param_pos_diffs_[next_param_index]);
    if (write_res == -1)
    {
        return ReturnCode::ERROR;
    }
    if (write_res != param_pos_diffs_[next_param_index])
    {
        return ReturnCode::OVERFLOW;
    }
    translated_length += param_pos_diffs_[next_param_index] + 2;
    next_param_index++;

    if (param < 0)
    {
        constexpr char point = '-';
        auto point_res = write(buffer, &point, 1);
        if (point_res == -1)
        {
            return ReturnCode::ERROR;
        }
        if (point_res != 1)
        {
            return ReturnCode::OVERFLOW;
        }
    }

    auto int_res = WriteIntegerToBuffer_(buffer, param.GetAbsIntPart());
    if (int_res != ReturnCode::OK)
    {
        return int_res;
    }
    constexpr char point = '.';
    auto point_res = write(buffer, &point, 1);
    if (point_res == -1)
    {
        return ReturnCode::ERROR;
    }
    if (point_res != 1)
    {
        return ReturnCode::OVERFLOW;
    }
    int fractional = (param.GetAbsFractionPart() * 1000) >>
                     math::FixedPointBase::GetFractionBits();
    int nulls_counter = 1000 / 10;
    while (nulls_counter > fractional)
    {
        nulls_counter /= 10;
        constexpr char null_char = '0';
        auto null_res = write(buffer, &null_char, 1);
        if (null_res == -1)
        {
            return ReturnCode::ERROR;
        }
        if (null_res != 1)
        {
            return ReturnCode::OVERFLOW;
        }
    }
    auto frac_res = WriteIntegerToBuffer_(buffer, fractional);
    if (frac_res != ReturnCode::OK)
    {
        return frac_res;
    }
    return ToBytes_(buffer, next_param_index, translated_length, others...);
}

template <typename... ArgTypes>
template <concepts::stream::ByteWritableStreamConcept DestType>
ReturnCode StaticFormatableString<ArgTypes...>::ToBytes_(
    DestType &buffer, unsigned next_param_index, int translated_length) const
{
    (void)next_param_index;
    int write_res =
        write(buffer, string_ + translated_length, length_ - translated_length);
    if (write_res == -1)
    {
        return ReturnCode::ERROR;
    }
    if (write_res != length_ - translated_length)
    {
        return ReturnCode::OVERFLOW;
    }
    return ReturnCode::OK;
}

template <typename... ArgTypes>
const char *StaticFormatableString<ArgTypes...>::GetString() const
{
    return string_;
}

template <typename... ArgTypes>
unsigned StaticFormatableString<ArgTypes...>::GetLength() const
{
    return length_;
}

template <typename... ArgTypes>
consteval unsigned
StaticFormatableString<ArgTypes...>::CountLength_(const char *string)
{
    unsigned length = 0;
    while (string[length] != '\0')
    {
        length++;
    }
    return length;
}

template <typename... ArgTypes>
template <concepts::stream::ByteWritableStreamConcept DestType>
ReturnCode
StaticFormatableString<ArgTypes...>::WriteIntegerToBuffer_(DestType &buffer,
                                                           int param)
{
    if (param == 0)
    {
        auto write_res = write(buffer, "0", 1);
        if (write_res == -1)
        {
            return ReturnCode::ERROR;
        }
        if (write_res != 1)
        {
            return ReturnCode::OVERFLOW;
        }
    }
    else
    {
        if (param < 0)
        {
            auto write_res = write(buffer, "-", 1);
            if (write_res == -1)
            {
                return ReturnCode::ERROR;
            }
            if (write_res != 1)
            {
                return ReturnCode::OVERFLOW;
            }
            param = -param;
        }

        unsigned digit = 1;
        while (digit <= static_cast<unsigned>(param))
        {
            digit *= 10;
        }
        digit /= 10;

        char symbol;

        while (digit != 0)
        {
            symbol = param / digit + '0';
            param %= digit;
            auto write_res = write(buffer, &symbol, 1);
            if (write_res == -1)
            {
                return ReturnCode::ERROR;
            }
            if (write_res != 1)
            {
                return ReturnCode::OVERFLOW;
            }
            digit /= 10;
        }
    }
    return ReturnCode::OK;
}

template <typename... ArgTypes>
consteval unsigned
StaticFormatableString<ArgTypes...>::CountParametres_(const char *string)
{
    unsigned param_number = 0;
    bool open_flag = false;
    unsigned length = 0;
    while (string[length] != '\0')
    {
        if (string[length] == '{')
        {
            assert(!open_flag); // TODO think about it
            open_flag = true;
            param_number++;
        }
        else if (string[length] == '}')
        {
            open_flag = false;
        }
        length++;
    }
    return param_number;
}
} // namespace hydrolib::strings

#endif
