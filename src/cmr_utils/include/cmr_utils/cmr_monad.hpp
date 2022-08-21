#pragma once
#include <type_traits>
#include <utility>
namespace monad
{
template <typename T, typename = void>
struct IsMonadLike : std::false_type {
};

template <typename T>
struct IsMonadLike<
    T,
    std::void_t<
        std::enable_if_t<std::is_same_v<decltype(std::declval<T>().has_value()), bool>>,
        decltype(std::declval<T>().value()), decltype(T(std::declval<T>().value()))>>
    : std::true_type {
};

template <typename T>
constexpr bool IsMonadLikeV = IsMonadLike<T>::value;

template <typename T, typename Func>
constexpr auto bind(T&& monad, Func f) noexcept
    -> std::enable_if_t<IsMonadLikeV<T>,
                        std::remove_reference_t<decltype(f(monad.value()))>>
{
    if (monad.has_value()) {
        return f(std::forward<decltype(monad.value())>(monad.value()));
    } else {
        return std::remove_reference_t<decltype(f(monad.value()))>{};
    }
}

template <template <typename> class T, typename U, typename Func>
constexpr auto map(const T<U>& monad, Func f) noexcept
    -> std::enable_if_t<IsMonadLikeV<T<U>>,
                        T<std::remove_reference_t<decltype(f(monad.value()))>>>
{
    if (monad.has_value()) {
        return T<std::remove_reference_t<decltype(f(monad.value()))>>{f(monad.value())};
    } else {
        return {};
    }
}

template <template <typename, typename...> class T, typename Func, typename U,
          typename... Rest>
constexpr auto map(T<U, Rest...>&& monad, Func f) noexcept -> std::enable_if_t<
    IsMonadLikeV<T<U>>,
    T<std::remove_reference_t<decltype(f(std::move(monad).value()))>, Rest...>>
{
    using RetType = std::remove_reference_t<decltype(f(std::move(monad).value()))>;
    if (monad.has_value()) {
        return T<RetType, Rest...>{f(std::move(monad).value())};
    } else {
        return {};
    }
}

template <typename T, typename U>
constexpr auto value_or(T&& monad, U&& default_val) noexcept
    -> std::enable_if_t<IsMonadLikeV<T>, U>
{
    if (monad.has_value()) {
        return monad.value();
    } else {
        return default_val;
    }
}

template <typename T, typename Func>
constexpr auto value_or_else(T&& monad, Func f) noexcept
    -> std::enable_if_t<IsMonadLikeV<T>, std::remove_reference_t<decltype(monad.value())>>
{
    if (monad.has_value()) {
        return monad.value();
    } else {
        return f();
    }
}
}  // namespace monad