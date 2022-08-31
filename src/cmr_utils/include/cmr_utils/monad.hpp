#pragma once
#include <type_traits>
#include <utility>
/**
 * @brief Namespace of helper functions for monads.
 *
 * [Read more here](https://cs3110.github.io/textbook/chapters/ds/monads.html)
 *
 * I use the term `monad` here to mean a type that wraps another value. They key
 * features for general a monad are that it can be constructed from a value it wraps,
 * it can return the value it wraps, and it can transform itself into a new monad
 * type. This namespace requires that monads may *not necessarily* contain a value
 * and thus should have a default constructor and a way to determine if they contain
 * a value or not.
 *
 * In this sense, I use the term monad slightly loosely, to mean some class that
 * wraps a type and can be augmented to be a monad. Specifically, I am targetting
 * `std::optional`to add the monad functions available to it in C++23.
 *
 *
 */
namespace monad
{

/**
 * SFINAE helpers for determining if a class is "monad like"
 * I say something is "monad like" if it has a `has_value()`, `value()`, default
 * constructor, and constructor that takes the value the monad wraps.
 * @{
 */
template <typename T, typename = void>
struct IsMonadLike : std::false_type {
};

template <typename T>
struct IsMonadLike<T,
                   std::void_t<std::enable_if_t<std::is_same_v<
                                   decltype(std::declval<T>().has_value()), bool>>,
                               decltype(std::declval<T>().value()),
                               decltype(T(std::declval<T>().value()))>>
    : std::true_type {
};

template <typename T>
constexpr bool is_monad_like_v = IsMonadLike<T>::value;
/** @} */

/**
 * @brief Converts a monad of one type to a monad of another type.
 * This function is equivalent to `bind` in traditional functional programming
 * monad literature.
 *
 * Has the same exception guaruntee as `f`
 *
 * @tparam T Monad type
 * @tparam Func a function which takes the value of a monad and returns a new monad
 * @param monad the input monad
 * @param f the monad transformation function
 * @return the monad converted by `f`, or an empty monad of the return type of `f`
 */
template <typename T, typename Func>
constexpr auto bind(T&& monad, Func f) noexcept(noexcept(f(monad.value())))
    -> std::enable_if_t<is_monad_like_v<T>,
                        std::remove_reference_t<decltype(f(monad.value()))>>;

/**
 * @brief Converts a monad of one type to a monad of another.
 * This is equivalent to `map` in Rust and `transform` in C++23 `std::optional`
 *
 * Has the same exception guaruntee as `f`
 *
 * @tparam T monad wrapper type
 * @tparam U monad value type
 * @tparam Func a function which takes `U` and returns `V` for any `V` and `U`
 * @param monad the input monad
 * @param f function which takes the value and returns a different value. Not
 * executed if `monad` does not contain a value
 * @return A monad of the returned value of `f` or an empty monad of the same type
 */
template <template <typename> class T, typename U, typename Func>
constexpr auto map(const T<U>& monad, Func f) noexcept(noexcept(f(monad.value())))
    -> std::enable_if_t<is_monad_like_v<T<U>>,
                        T<std::remove_reference_t<decltype(f(monad.value()))>>>;

/**
 * @brief Gets the value contained within the monad or the default if it is empty
 *
 * @tparam T monad type
 * @tparam U default type
 * @param monad
 * @param default_val
 * @return the value within `monad` or `default_val`
 */
template <typename T, typename U>
constexpr auto value_or(T&& monad, U&& default_val) noexcept
    -> std::enable_if_t<is_monad_like_v<T>, U>;

/**
 * @brief Gets the value contained within the monad or the value returned by the
 * function if it is empty
 *
 * Has the same exception guaruntees as `f`
 *
 * @tparam T monad type
 * @tparam Func a function which takes nothing and returns a value of the wrapped
 *  monad value
 * @param monad
 * @param f
 * @return
 */
template <typename T, typename Func>
constexpr auto value_or_else(T&& monad, Func f) noexcept(noexcept(f()))
    -> std::enable_if_t<is_monad_like_v<T>,
                        std::remove_reference_t<decltype(monad.value())>>;
}  // namespace monad

#include "cmr_utils/monad.inl"