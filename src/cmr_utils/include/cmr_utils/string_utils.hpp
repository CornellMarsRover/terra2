#pragma once

#include <memory>
#include <string>
#include <stdexcept>

namespace cmr
{
/**
 * Format a string using <code>std::snprintf</code>.
 *
 * @param format format string
 * @param args format arguments
 * @return the formatted string
 * @throw std::runtime_error if the format was invalid
 */
template<typename ... Args>
std::string string_format(const std::string & format, Args ... args)
{
  // Source: https://stackoverflow.com/a/26221725/3212994 (explanation commented here too)
  // the goal of this approach is to create a char buffer of the final string's size and
  // use std::snprintf to write to it, before finally converting it to a std:string.

  // first, we need to know what the final size actually is.
  // this requires a call to std::snprintf, where we don't actually write the result anywhere
  // but just use the size it returns. this also gives us a chance to do some error handling
  // which is the advantage of this longer-winded approach over std::sprintf.
  int size_s =
    std::snprintf(nullptr, 0, format.c_str(), args ...) +
    1;         // add extra space for terminator '\0'
  if (size_s <= 0) {throw std::runtime_error("string format failed");}
  auto size = static_cast<size_t>( size_s );

  // now that we know the size, and we know that the format is valid, we finally create our
  // char buffer and use std::snprintf to write it
  std::unique_ptr<char[]> buf(new char[size]);
  std::snprintf(buf.get(), size, format.c_str(), args ...);
  return {buf.get(), buf.get() + size - 1};   // We don't want the terminator '\0' inside
}

/**
 * Check whether a string ends with the given substring.
 * @param value the string to check
 * @param ending the ending
 * @return true if the string ends in the ending, false otherwise
 */
inline bool ends_with(std::string const & value, std::string const & ending)
{
  if (ending.size() > value.size()) {return false;}
  return std::equal(ending.rbegin(), ending.rend(), value.rbegin());
}
}
