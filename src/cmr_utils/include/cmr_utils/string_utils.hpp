#pragma once

#include <sstream>
#include <string>

#include "cmr_debug.hpp"

namespace cmr
{
/**
 * Format a string using `std::snprintf`.
 * Formatting uses the same format codes and semantics as `printf`
 *
 * @param format format string
 * @param args format arguments
 * @return the formatted string
 * @throw `std::runtime_error` if the format was invalid
 */
template <typename... Args>
std::string string_format(const std::string& format, Args&&... args)
{
    // Source: https://stackoverflow.com/a/26221725/3212994 (explanation commented
    // here too) the goal of this approach is to create a char buffer of the final
    // string's size and use std::snprintf to write to it, before finally converting
    // it to a std:string.

    // first, we need to know what the final size actually is.
    // this requires a call to std::snprintf, where we don't actually write the
    // result anywhere but just use the size it returns. this also gives us a chance
    // to do some error handling which is the advantage of this longer-winded
    // approach over std::sprintf.
    const int size_s =
        std::snprintf(nullptr, 0, format.c_str(), std::forward<Args>(args)...) +
        1;  // add extra space for terminator '\0'
    CMR_ASSERT_MSG(
        size_s > 0,
        "Invalid format or mismatched arguments passed to `string_format`");
    const auto size = static_cast<size_t>(size_s);

    // now that we know the size, and we know that the format is valid, we finally
    // create our char buffer and use std::snprintf to write it
    std::string buf(size, '\0');  // Use string for NRVO
    std::snprintf(buf.data(), size, format.c_str(), std::forward<Args>(args)...);
    buf.resize(size - 1);
    // We don't want the terminator '\0' inside
    return buf;
}

/**
 * @brief Constructs a string by concatenating together the arguments via streams
 *
 * ## Example
 * ```C++
 * const auto age = 10;
 * build_string("Hello World!", " ", "I am ", age, "years old today!",
 *              " That's", std::hex, age, " in hexadecimal\n");
 *
 * // Results in:
 * // "Hello World! I am 10 years old today! That's 0x0A in hexadecimal
 * "
 * ```
 *
 * @tparam Args anything that can be printed to a stream
 * @param args
 * @return formatted string
 */
template <typename... Args>
auto build_string(Args&&... args)
{
    std::stringstream ss;
    (ss << ... << std::forward<Args>(args));
    return ss.str();
}

}  // namespace cmr
