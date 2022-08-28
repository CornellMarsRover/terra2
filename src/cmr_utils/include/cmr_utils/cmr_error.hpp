#pragma once
#include <type_traits>
#include <utility>

#include "rclcpp/rclcpp.hpp"

/** @file Contains general error handling, debug, and assertion utilities for the
 * code base
 */

/** Gets the default ROS logger. Called if logging outside a node and `get_logger`
 * is not shadowed */
inline auto get_logger() { return rclcpp::get_logger("cmr_log"); }

/** An assert handler is the function that is called when an assert fails
 * or an `CMR_INVALID` is triggered.
 *
 * The default assert handler will trap a debugger if one is attached, otherwise it
 * will terminate the program
 */
using assert_handler_t = void (*)();

/**
 * @brief Set the assert handler callback.
 *
 * This function doesn't have to be called since there is a default handler. It is
 * here only for hooking asserts for unit tests.
 *
 * @param handler the new assert handler
 * @return the old assert handler
 */
assert_handler_t set_assert_handler(assert_handler_t handler) noexcept;

/** Gets the assert handler object */
assert_handler_t get_assert_handler() noexcept;

/**
 * @def CMR_LOG
 *
 * Logs a message of the specified level.
 * Uses the node logger if called from a node, otherwise uses the default logger
 * @param LEVEL the level of the message, such as WARN, INFO, DEBUG, FATAL
 * @param ... A format string followed by format arguments, like `printf`
 */
// NOLINTNEXTLINE
#define CMR_LOG(LEVEL, ...) RCLCPP_##LEVEL(get_logger(), __VA_ARGS__)

/**
 * @def CMR_ASSERT
 *
 * Asserts a condition is true. Optionally pass a format string to log.
 * If the condition is false, creates a `FATAL` log and calls the assert handler
 * which by default traps the debugger or terminates the program
 * @param CONDITION condition to assert
 */
// NOLINTNEXTLINE
#define CMR_ASSERT(CONDITION)                                                \
    if (!(CONDITION)) {                                                      \
        CMR_LOG(FATAL, "ASSERT: '%s' FAILED in %s:%d", #CONDITION, __FILE__, \
                __LINE__);                                                   \
        get_assert_handler()();                                              \
    }

/**
 * @def CMR_ASSERT_MSG
 * Like `CMR_ASSERT` but also supplied a format string followed by format arguments,
 * just like `printf`
 *
 * ### Example
 * - `CMR_ASSERT_MSG(x == 10, "%s was %d instead of 10", "x", x)`
 * - `CMR_ASSERT(x == 10);`
 *
 */
// NOLINTNEXTLINE
#define CMR_ASSERT_MSG(CONDITION, ...) \
    CMR_ASSERT(CONDITION);             \
    CMR_LOG(FATA, __VA_ARGS__);

/**
 * @brief Helper function for `CMR_INVALID` macro.
 * This function will log debug info, call the assert handler, then terminate the
 * program
 *
 * @param file file that the invalid state occurred at
 * @param line line number of invalid statement
 * @param msg debug message to log
 * @see CMR_INVALID
 */
[[noreturn]] inline void cmr_invalid(const char* file, unsigned line,
                                     const char* msg);

/**
 * @def CMR_INVALID
 * Asserts that a statement should not be reached
 */
// NOLINTNEXTLINE
#define CMR_INVALID(MSG) cmr_invalid(__FILE__, __LINE__, MSG)

/** @defgroup DEBUG_MACROS
 * Debug versions of `CMR_LOG` and `CMR_ASSERT`
 * Only enabled in debug builds and should be used VERY sparingly
 * @{
 */
#ifndef NDEBUG
// NOLINTNEXTLINE
#define CMR_ASSSERT_D(CONDITION, ...) CMR_ASSERT(CONDITION, __VA_ARGS__)
// NOLINTNEXTLINE
#define CMR_LOG_D(LEVEL, ...) CMR_LOG(LEVEL, __VA_ARGS__)
#else
// NOLINTNEXTLINE
#define CMR_ASSERT_D(CONDITION, ...)
// NOLINTNEXTLINE
#define CMR_LOG_D(LEVEL, ...)
#endif
/** @} */

/**
 * @brief Base class for all custom exceptions for our code
 *
 * You should use distinct exception classes if you expect a user would want to
 * handle the errors differently.
 *
 * If a user would want different debug information, than you should use a single
 * exception class that is constructed from an error code which returns different
 * messages based on that error code in `what()`
 *
 * Exceptions should be used for external errors that can occur even when our code
 * is bugfree, and should be thrown only if there is a reasonable thing that the
 * user can do to handler the exception.
 *
 * ## Example
 * ```C++
 * enum class ECEIOErrorCode
 * {
 *      Malformed, InvalidData, Timeout
 * };
 *
 *
 * class ECEIOException : public CmrException
 * {
 *      ECEIOErrorCode m_error;
 *    public:
 *      ECEIOException(ECEIOErrorCode code) : m_error(code) {}
 *
 *      const char* what() const noexcept
 *      {
 *         switch(m_error) {
 *              case ECEIOErrorCode::Malformed:
 *                  return "Malformed packet";
 *              // ...
 *          }
 *      }
 * }
 * ```
 *
 * Notice how in this example, a client can't really do any different error
 * handling for each case besides retry. However someone debugging an issue
 * may want this information. Further note how even if our code is
 * bug free, this exception could occur. The exception is an external problem.
 * This is a good usage of an exception
 *
 */
class CmrException : public std::exception
{
};

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

#include "cmr_utils/cmr_error.inl"
