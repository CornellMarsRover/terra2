#pragma once
#include <csignal>
#include <optional>
#include <string_view>
#include <variant>

#include "rclcpp/rclcpp.hpp"

/// Gets the default logger. Called if logging outside a node and `get_logger`
/// is not shadowed
inline auto get_logger() { return rclcpp::get_logger("cmr_log"); }

/**
 * Logs a message of the specified level.
 * Uses the node logger if called from a node, otherwise uses the default logger
 * @param LEVEL the level of the message, such as WARN, INFO, DEBUG, FATAL
 */
// NOLINTNEXTLINE
#define CMR_LOG(LEVEL, ...) RCLCPP_##LEVEL(get_logger(), __VA_ARGS__)

/**
 * Asserts a condition is true.
 * If the condition is false, creates a `FATAL` log and signals to trap the debugger
 * if one is attached, otherwise aborts the program.
 */
// NOLINTNEXTLINE
#define CMR_ASSERT(CONDITION, ...)                                           \
    if (!(CONDITION)) {                                                      \
        CMR_LOG(FATAL, "ASSERT: '%s' FAILED in %s:%d", #CONDITION, __FILE__, \
                __LINE__);                                                   \
        CMR_LOG(FATAL, __VA_ARGS__);                                         \
        raise(SIGTRAP);                                                      \
    }

/// Helper function for macro. Aborts the program with the specified error
/// information
[[noreturn]] inline void cmr_invalid(const char* file, unsigned line,
                                     const char* msg) noexcept
{
    CMR_LOG(FATAL, "CMR_INVALID triggered at %s:%u", file, line);
    CMR_LOG(FATAL, "%s", msg);
    raise(SIGTRAP);
    std::terminate();
}

/**
 * Asserts that a statement should not be reached
 */
// NOLINTNEXTLINE
#define CMR_INVALID(MSG) cmr_invalid(__FILE__, __LINE__, MSG)

/// @{
/// Debug versions of `CMR_LOG` and `CMR_ASSERT`
/// Only enabled in debug builds and should be used VERY sparingly
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
/// @}

enum class ErrorCode {
    /// An error ocurred but we give the user no further information
    OpaqueError
};

/// Base class for all errors
class Error
{
  public:
    virtual ~Error() = default;

    /// Gets a description of the error. Can be empty
    virtual std::string_view what() const noexcept = 0;

    /// Gets an error code for the error.
    virtual ErrorCode code() const noexcept = 0;
};

/// An error type with only an error code and its description
class BasicError : public Error
{
    ErrorCode m_code;

  public:
    explicit BasicError(ErrorCode code) : m_code(code) {}

    ErrorCode code() const noexcept override { return m_code; }
    std::string_view what() const noexcept override
    {
        switch (m_code) {
            case ErrorCode::OpaqueError:
                return {"No information available"};
            default:
                CMR_INVALID("Unimplemented error code");
        }
    }
};

/// A result type that holds either a value, or an error
/// @tparam T the type of the value
/// @tparam E the type of the error
template <typename T, typename E = std::unique_ptr<Error>>
class Result
{
    std::variant<T, E> m_data;

  public:
    Result() : m_data(T{}) {}
    Result(T&& t) : m_data(std::move(t)) {}  // NOLINT(google-explicit-constructor)
    Result(E&& e) : m_data(std::move(e)) {}  // NOLINT(google-explicit-constructor)
    Result(const T& t) : m_data(t) {}        // NOLINT(google-explicit-constructor)

    template <typename EArg>
    // NOLINTNEXTLINE(google-explicit-constructor)
    Result(std::unique_ptr<EArg>&& e) : m_data(E(e.release()))
    {
    }

    /// `true` if the result holds the error type.
    inline bool has_error() const noexcept
    {
        return std::holds_alternative<E>(m_data);
    }

    /// `true` if the result holds the value type.
    inline bool has_value() const noexcept
    {
        return std::holds_alternative<T>(m_data);
    }

    /// Gets an optional to the value
    std::optional<T> maybe_get_val() const noexcept
    {
        if (std::holds_alternative<T>(m_data)) {
            return {std::get<T>(m_data)};
        } else {
            return {};
        }
    }

    /// Gets an optional to the error type
    std::optional<T> maybe_get_err() const noexcept
    {
        if (std::holds_alternative<E>(m_data)) {
            return {std::get<E>(m_data)};
        } else {
            return {};
        }
    }

    const T& value() const& noexcept
    {
        CMR_ASSERT(has_value(), "Called value() on an error Result");
        return std::get<T>(m_data);
    }
    T& value() & noexcept
    {
        CMR_ASSERT(has_value(), "Called value() on an error Result");
        return std::get<T>(m_data);
    }
    T value() && noexcept
    {
        CMR_ASSERT(has_value(), "Called value() on an error Result");
        return std::move(std::get<T>(m_data));
    }
};

template <typename T, typename E>
inline Result<T> make_opaque_result(E&& error) noexcept
{
    return {std::make_unique<E>(std::forward<E>(error))};
}
