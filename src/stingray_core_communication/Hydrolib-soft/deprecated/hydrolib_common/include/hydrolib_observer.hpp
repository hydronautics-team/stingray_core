#ifndef HYDROLIB_OBSERVER_H_
#define HYDROLIB_OBSERVER_H_

namespace hydrolib::observer
{
    template <typename T>
    concept Observer = requires(T observer, const void *data, const void *source) {
        observer.Notify(source, data);
    };  // TODO Try to make template

    void NotifyAll(const void *source, const void *data)
    {
        (void)source;
        (void)data;
    }

    template <Observer T0, Observer... Ts>
    void NotifyAll(const void *source, const void *data, T0 &observer, Ts &...others)
    {
        observer.Notify(source, data);
        NotifyAll(source, data, others...);
    }
}

#endif
