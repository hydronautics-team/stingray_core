#pragma once

namespace hydrolib::filter
{
template <typename Number, double PERIOD_MS, double TAU_MS>
class IIR
{
public:
    constexpr IIR(Number last_u = 0, Number last_x = 0);

public:
    Number Process(Number u);

private:
    Number last_u_;
    Number last_x_;
};

template <typename Number, double PERIOD_MS, double TAU_MS>
constexpr IIR<Number, PERIOD_MS, TAU_MS>::IIR(Number last_u, Number last_x)
    : last_u_(last_u), last_x_(last_x)
{
}

template <typename Number, double PERIOD_MS, double TAU_MS>
inline Number IIR<Number, PERIOD_MS, TAU_MS>::Process(Number u)
{
    last_x_ =
        Number(PERIOD_MS / (2 * TAU_MS + PERIOD_MS)) * (u + last_u_) -
        Number((PERIOD_MS - 2 * TAU_MS) / (2 * TAU_MS + PERIOD_MS)) * last_x_;
    last_u_ = u;
    return last_x_;
}

} // namespace hydrolib::filter