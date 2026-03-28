#pragma once

#include <algorithm>
#include <cstring>

namespace hydrolib::strings
{
template <unsigned CAPACITY>
class CString
{
public:
    constexpr CString(const char *str);
    constexpr CString() = default;
    // constexpr CString(const CString& other);

public:
    [[deprecated]]
    constexpr char *GetString();
    [[deprecated]]
    constexpr const char *GetConstString() const;
    constexpr unsigned GetLength() const;

    int Write(const void *data, unsigned length);

    constexpr operator const char *() const;
    constexpr char &operator[](int index);

private:
    char string_[CAPACITY] = {};
    unsigned length_ = 0;
};

template <unsigned CAPACITY>
int write(CString<CAPACITY> &str, const void *data, unsigned length);

template <unsigned CAPACITY>
constexpr CString<CAPACITY>::CString(const char *str)
    : length_(std::strlen(str))
{
    std::copy(str, str + length_, string_);
}

// template <unsigned CAPACITY>
// constexpr CString<CAPACITY>::CString(const CString& other):
// length_(other.length_)
// {
//     std::copy(other.string_, other.string_+length_, string_);
// }

template <unsigned CAPACITY>
constexpr char *CString<CAPACITY>::GetString()
{
    return string_;
}

template <unsigned CAPACITY>
constexpr const char *CString<CAPACITY>::GetConstString() const
{
    return string_;
}

template <unsigned CAPACITY>
constexpr unsigned CString<CAPACITY>::GetLength() const
{
    return length_;
}

template <unsigned CAPACITY>
int CString<CAPACITY>::Write(const void *data, unsigned length)
{
    unsigned writing_length = length;
    if (CAPACITY - 1 - length_ < length) [[unlikely]]
    {
        writing_length = CAPACITY - 1 - length_;
    }
    memcpy(string_ + length_, data, writing_length);
    length_ += writing_length;
    return writing_length;
}

template <unsigned CAPACITY>
constexpr CString<CAPACITY>::operator const char *() const
{
    return const_cast<const char *>(string_);
}

template <unsigned CAPACITY>
constexpr char &CString<CAPACITY>::operator[](int index)
{
    return string_[index];
}

template <unsigned CAPACITY>
int write(CString<CAPACITY> &str, const void *data, unsigned length)
{
    return str.Write(data, length);
}

} // namespace hydrolib::strings
