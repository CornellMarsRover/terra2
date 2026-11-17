// Minimal TOML reader for the flat cmr_rovernet config files
// (config/*.toml: root keys + simple [tables] of scalar key = value pairs).
// Supports: comments, [table] / [dotted.table] headers, bare / quoted keys,
// basic + literal strings, integers (with '_' / 0x / 0o / 0b), floats (incl.
// inf/nan), booleans.  Arrays and inline tables are kept as raw text
// (TomlValue::Type::kOther), which is enough since no port reads them.
// Anything else (multi-line strings, [[array tables]]) is a parse error.
#pragma once

#include <cstdint>
#include <map>
#include <stdexcept>
#include <string>

namespace cmr_rovernet_cpp {

struct TomlValue
{
  enum class Type { kString, kInteger, kFloat, kBool, kOther };
  Type type = Type::kOther;
  std::string str;      // string value, or raw text for kOther
  int64_t integer = 0;
  double floating = 0.0;
  bool boolean = false;

  // Python-style conversions: float(v), int(v), bool(v), str(v).
  double as_double() const;
  int64_t as_int() const;
  bool as_bool() const;
  std::string as_string() const;
};

using TomlTable = std::map<std::string, TomlValue>;

struct TomlDocument
{
  TomlTable root;                            // keys before the first header
  std::map<std::string, TomlTable> tables;   // "[name]" -> keys
};

class TomlParseError : public std::runtime_error
{
public:
  using std::runtime_error::runtime_error;
};

/// Throws TomlParseError on syntax errors and std::runtime_error if the file
/// cannot be opened.
TomlDocument parse_toml_file(const std::string & path);
TomlDocument parse_toml_string(const std::string & text);

}  // namespace cmr_rovernet_cpp
