#include "hydrolib_cstring.hpp"
#include "hydrolib_fixed_point.hpp"
#include "hydrolib_formatable_string.hpp"

#include <cstring>
#include <gtest/gtest.h>
#include <iostream>

using namespace hydrolib::strings;
using namespace hydrolib::math;
using namespace std;

class BytesReceiver
{
public:
    BytesReceiver() : current_length(0) {}

public:
    void Push(const void *source, std::size_t length)
    {
        memcpy(buffer + current_length, source, length);
        current_length += length;
    }

public:
    uint8_t buffer[100];
    unsigned current_length;
};

int write(BytesReceiver &stream, const void *source, unsigned length)
{
    stream.Push(source, length);
    return length;
}

TEST(TestHydrolibStrings, TestFormatableString)
{
    constexpr StaticFormatableString<int, int, int, CString<4>, FixedPointBase>
        s("Inserting values: {}, {}, {}, {}, {} End");
    BytesReceiver stream;
    s.ToBytes(stream, 1, 20, -33, CString<4>("haha"), 1.5_fp);
    stream.buffer[stream.current_length] = '\0';

    EXPECT_EQ(0, strcmp(reinterpret_cast<char *>(stream.buffer),
                        "Inserting values: 1, 20, -33, haha, 1.500 End"));
}

// TEST(TestHydrolibStrings, Test)
// {
//     constexpr StaticFormatableString<FixedPointBase> s("{}");
//     BytesReceiver stream;
//     FixedPointBase a = 3.0;
//     while (a >= -3)
//     {
//         s.ToBytes(stream, a);
//         stream.buffer[stream.current_length] = '\0';
//         cout << stream.buffer << std::endl;
//         stream.current_length = 0;
//         a -= 0.00005;
//     }
// }
