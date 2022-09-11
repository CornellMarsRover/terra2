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
    CMR_LOG(FATAL, __VA_ARGS__);

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
#define CMR_ASSSERT_D(CONDITION) CMR_ASSERT(CONDITION)
// NOLINTNEXTLINE
#define CMR_ASSSERT_MSG_D(CONDITION, ...) CMR_ASSERT_MSG(CONDITION, __VA_ARGS__)
// NOLINTNEXTLINE
#define CMR_LOG_D(LEVEL, ...) CMR_LOG(LEVEL, __VA_ARGS__)
#else
// NOLINTNEXTLINE
#define CMR_ASSERT_D(CONDITION)
// NOLINTNEXTLINE
#define CMR_ASSERT_MSG_D(CONDITION, ...)
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

[[noreturn]] inline void cmr_invalid(const char* file, unsigned line,
                                     const char* msg)
{
    CMR_LOG(FATAL, "CMR_INVALID triggered at %s:%u", file, line);
    CMR_LOG(FATAL, "%s", msg);
    get_assert_handler()();  // the assert handler could throw an exception
    std::terminate();
    // To use cmr_invalid to end control paths (ie. need not explicitly return from
    // a function after calling it), we must make sure we actually abort
    // Returning from a [[noreturn]] function is undefined behavior
}
