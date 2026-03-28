#ifndef HYDROLIB_PID_H_
#define HYDROLIB_PID_H_

namespace hydrolib::controlling
{
template <unsigned FREQ_HZ>
class PID
{
public:
    PID(unsigned last_output = 0, unsigned last_input = 0)
        : p_(0),
          i_(0),
          divide_shift_(0),
          current_input_(0),
          last_input_(last_input),
          current_output_undivided_(last_output)
    {
    }

public:
    void SetP(unsigned p) { p_ = p; }

    void SetI(unsigned i) { i_ = i; }

    void SetDivideShift(unsigned divide_shift) { divide_shift_ = divide_shift; }

    int Process(int input)
    {
        last_input_ = current_input_;
        current_input_ = input;

        int delta = static_cast<int>(p_) * (current_input_ - last_input_) +
                    static_cast<int>(i_) * (current_input_ + last_input_) /
                        static_cast<int>(FREQ_HZ) / 2;

        current_output_undivided_ += delta;
        return current_output_undivided_ >> divide_shift_;
    }

private:
    unsigned p_;
    unsigned i_;
    unsigned divide_shift_;

    int current_input_;
    int last_input_;

    int current_output_undivided_;
};
} // namespace hydrolib::controlling

#endif