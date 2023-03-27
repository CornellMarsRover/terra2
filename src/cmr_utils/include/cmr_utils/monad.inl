/// @file Inline definitions for monad.hpp
#pragma once
#include "cmr_utils/monad.hpp"

namespace monad
{
template <typename T, typename Func>
constexpr auto bind(T&& monad, Func f) noexcept(noexcept(f(monad.value())))
    -> std::enable_if_t<is_monad_like_v<T>,
                        std::remove_reference_t<decltype(f(monad.value()))>>
{
    if (monad.has_value()) {
        return f(std::forward<decltype(monad.value())>(monad.value()));
    } else {
        return std::remove_reference_t<decltype(f(monad.value()))>{};
    }
}

template <template <typename> class T, typename U, typename Func>
constexpr auto map(const T<U>& monad, Func f) noexcept(noexcept(f(monad.value())))
    -> std::enable_if_t<is_monad_like_v<T<U>>,
                        T<std::remove_reference_t<decltype(f(monad.value()))>>>
{
    if (monad.has_value()) {
        return T<std::remove_reference_t<decltype(f(monad.value()))>>{
            f(monad.value())};
    } else {
        return {};
    }
}

template <typename T, typename U>
constexpr auto value_or(T&& monad, U&& default_val) noexcept
    -> std::enable_if_t<is_monad_like_v<T>, U>
{
    if (monad.has_value()) {
        return monad.value();
    } else {
        return default_val;
    }
}

template <typename T, typename Func>
constexpr auto value_or_else(T&& monad, Func f) noexcept(noexcept(f()))
    -> std::enable_if_t<is_monad_like_v<T>,
                        std::remove_reference_t<decltype(monad.value())>>
{
    if (monad.has_value()) {
        return monad.value();
    } else {
        return f();
    }
}

template <typename T, typename U>
constexpr U value_or(const std::pair<bool, T>& pair, U&& default_val) noexcept
{
    return pair.first ? pair.second : default_val;
}

template <typename T, typename Func>
constexpr T value_or_else(const std::pair<bool, T>& pair,
                          Func f) noexcept(noexcept(f()))
{
    return pair.first ? pair.second : f();
}
}  // namespace monad