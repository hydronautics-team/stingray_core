#ifndef HYDROLIB_FUNC_CONCEPTS_H_
#define HYDROLIB_FUNC_CONCEPTS_H_

#include <concepts>

namespace hydrolib::concepts::func
{
template <typename T, typename ReturnType, typename... ArgTypes>
concept FuncConcept = requires(T func, ArgTypes... args) {
    { func(args...) } -> std::same_as<ReturnType>;
};

template <typename ReturnType, typename... ArgTypes>
ReturnType DummyFunc([[maybe_unused]] ArgTypes... args)
{
}

} // namespace hydrolib::concepts::func

#endif
