#include "cmr_rovernet_cpp/toml_lite.hpp"

#include <cctype>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>

namespace cmr_rovernet_cpp {
namespace {

std::string trim(const std::string & s)
{
  const auto b = s.find_first_not_of(" \t\r\n");
  if (b == std::string::npos) {
    return "";
  }
  const auto e = s.find_last_not_of(" \t\r\n");
  return s.substr(b, e - b + 1);
}

// Removes a trailing '#' comment that is not inside a string.
std::string strip_comment(const std::string & line)
{
  char quote = 0;
  for (std::size_t i = 0; i < line.size(); ++i) {
    const char c = line[i];
    if (quote) {
      if (quote == '"' && c == '\\') {
        ++i;
      } else if (c == quote) {
        quote = 0;
      }
    } else if (c == '"' || c == '\'') {
      quote = c;
    } else if (c == '#') {
      return line.substr(0, i);
    }
  }
  return line;
}

[[noreturn]] void fail(int line_no, const std::string & what)
{
  std::ostringstream o;
  o << "line " << line_no << ": " << what;
  throw TomlParseError(o.str());
}

std::string parse_basic_string(const std::string & v, int line_no)
{
  // v starts with '"' and must end with the matching '"'.
  std::string out;
  std::size_t i = 1;
  for (; i < v.size(); ++i) {
    const char c = v[i];
    if (c == '"') {
      break;
    }
    if (c == '\\') {
      if (++i >= v.size()) {
        fail(line_no, "unterminated escape");
      }
      switch (v[i]) {
        case 'n': out += '\n'; break;
        case 't': out += '\t'; break;
        case 'r': out += '\r'; break;
        case 'b': out += '\b'; break;
        case 'f': out += '\f'; break;
        case '"': out += '"'; break;
        case '\\': out += '\\'; break;
        default: fail(line_no, "unsupported string escape");
      }
    } else {
      out += c;
    }
  }
  if (i >= v.size() || trim(v.substr(i + 1)) != "") {
    fail(line_no, "malformed string");
  }
  return out;
}

std::string parse_key(const std::string & raw, int line_no)
{
  const std::string k = trim(raw);
  if (k.empty()) {
    fail(line_no, "empty key");
  }
  if (k.front() == '"') {
    return parse_basic_string(k, line_no);
  }
  if (k.front() == '\'') {
    if (k.size() < 2 || k.back() != '\'') {
      fail(line_no, "malformed key");
    }
    return k.substr(1, k.size() - 2);
  }
  for (char c : k) {
    if (!(std::isalnum(static_cast<unsigned char>(c)) || c == '_' || c == '-' || c == '.')) {
      fail(line_no, "invalid bare key '" + k + "'");
    }
  }
  return k;
}

bool parse_number(const std::string & text, TomlValue & out)
{
  std::string s;
  for (char c : text) {
    if (c != '_') {
      s += c;
    }
  }
  if (s.empty()) {
    return false;
  }
  std::string body = s;
  bool neg = false;
  if (body[0] == '+' || body[0] == '-') {
    neg = body[0] == '-';
    body = body.substr(1);
  }
  if (body == "inf" || body == "nan") {
    out.type = TomlValue::Type::kFloat;
    out.floating = body == "inf" ? std::numeric_limits<double>::infinity() :
      std::numeric_limits<double>::quiet_NaN();
    if (neg) {
      out.floating = -out.floating;
    }
    return true;
  }
  try {
    std::size_t pos = 0;
    if (body.size() > 2 && body[0] == '0' && (body[1] == 'x' || body[1] == 'o' || body[1] == 'b')) {
      const int base = body[1] == 'x' ? 16 : (body[1] == 'o' ? 8 : 2);
      const int64_t v = std::stoll(body.substr(2), &pos, base);
      if (pos != body.size() - 2) {
        return false;
      }
      out.type = TomlValue::Type::kInteger;
      out.integer = neg ? -v : v;
      return true;
    }
    const bool is_float = body.find_first_of(".eE") != std::string::npos;
    if (!std::isdigit(static_cast<unsigned char>(body[0]))) {
      return false;
    }
    if (is_float) {
      const double v = std::stod(s, &pos);
      if (pos != s.size()) {
        return false;
      }
      out.type = TomlValue::Type::kFloat;
      out.floating = v;
    } else {
      const int64_t v = std::stoll(s, &pos, 10);
      if (pos != s.size()) {
        return false;
      }
      out.type = TomlValue::Type::kInteger;
      out.integer = v;
    }
    return true;
  } catch (const std::exception &) {
    return false;
  }
}

TomlValue parse_value(const std::string & raw, int line_no)
{
  const std::string v = trim(raw);
  TomlValue out;
  if (v.empty()) {
    fail(line_no, "missing value");
  }
  if (v.rfind("\"\"\"", 0) == 0 || v.rfind("'''", 0) == 0) {
    fail(line_no, "multi-line strings are not supported");
  }
  if (v.front() == '"') {
    out.type = TomlValue::Type::kString;
    out.str = parse_basic_string(v, line_no);
    return out;
  }
  if (v.front() == '\'') {
    const auto end = v.find('\'', 1);
    if (end == std::string::npos || trim(v.substr(end + 1)) != "") {
      fail(line_no, "malformed literal string");
    }
    out.type = TomlValue::Type::kString;
    out.str = v.substr(1, end - 1);
    return out;
  }
  if (v == "true" || v == "false") {
    out.type = TomlValue::Type::kBool;
    out.boolean = v == "true";
    return out;
  }
  if (v.front() == '[' || v.front() == '{') {
    out.type = TomlValue::Type::kOther;
    out.str = v;
    return out;
  }
  if (!parse_number(v, out)) {
    fail(line_no, "unsupported value '" + v + "'");
  }
  return out;
}

int bracket_balance(const std::string & s)
{
  int depth = 0;
  char quote = 0;
  for (std::size_t i = 0; i < s.size(); ++i) {
    const char c = s[i];
    if (quote) {
      if (quote == '"' && c == '\\') {
        ++i;
      } else if (c == quote) {
        quote = 0;
      }
    } else if (c == '"' || c == '\'') {
      quote = c;
    } else if (c == '[' || c == '{') {
      ++depth;
    } else if (c == ']' || c == '}') {
      --depth;
    }
  }
  return depth;
}

}  // namespace

double TomlValue::as_double() const
{
  switch (type) {
    case Type::kFloat: return floating;
    case Type::kInteger: return static_cast<double>(integer);
    case Type::kBool: return boolean ? 1.0 : 0.0;
    case Type::kString: {
        std::size_t pos = 0;
        const std::string t = trim(str);
        const double d = std::stod(t, &pos);
        if (pos != t.size()) {
          throw std::invalid_argument("could not convert string to float: '" + str + "'");
        }
        return d;
      }
    default: break;
  }
  throw std::invalid_argument("cannot convert TOML value '" + str + "' to float");
}

int64_t TomlValue::as_int() const
{
  switch (type) {
    case Type::kInteger: return integer;
    case Type::kFloat: return static_cast<int64_t>(std::trunc(floating));
    case Type::kBool: return boolean ? 1 : 0;
    case Type::kString: {
        std::size_t pos = 0;
        const std::string t = trim(str);
        const int64_t i = std::stoll(t, &pos, 10);
        if (pos != t.size()) {
          throw std::invalid_argument("invalid literal for int(): '" + str + "'");
        }
        return i;
      }
    default: break;
  }
  throw std::invalid_argument("cannot convert TOML value '" + str + "' to int");
}

bool TomlValue::as_bool() const
{
  // Python truthiness.
  switch (type) {
    case Type::kBool: return boolean;
    case Type::kInteger: return integer != 0;
    case Type::kFloat: return floating != 0.0;
    case Type::kString: return !str.empty();
    default: return !(str == "[]" || str == "{}");
  }
}

std::string TomlValue::as_string() const
{
  switch (type) {
    case Type::kString: return str;
    case Type::kInteger: return std::to_string(integer);
    case Type::kFloat: {
        std::ostringstream o;
        o << floating;
        return o.str();
      }
    case Type::kBool: return boolean ? "True" : "False";
    default: return str;
  }
}

TomlDocument parse_toml_string(const std::string & text)
{
  TomlDocument doc;
  TomlTable * current = &doc.root;
  std::istringstream in(text);
  std::string line;
  int line_no = 0;
  while (std::getline(in, line)) {
    ++line_no;
    std::string l = trim(strip_comment(line));
    if (l.empty()) {
      continue;
    }
    if (l.front() == '[') {
      if (l.rfind("[[", 0) == 0) {
        fail(line_no, "array tables are not supported");
      }
      if (l.back() != ']') {
        fail(line_no, "malformed table header");
      }
      const std::string name = parse_key(l.substr(1, l.size() - 2), line_no);
      current = &doc.tables[name];
      continue;
    }
    const auto eq = l.find('=');
    if (eq == std::string::npos) {
      fail(line_no, "expected key = value");
    }
    const std::string key = parse_key(l.substr(0, eq), line_no);
    std::string value = l.substr(eq + 1);
    // Multi-line arrays / inline tables: keep reading until balanced.
    while (bracket_balance(value) > 0 && std::getline(in, line)) {
      ++line_no;
      value += " " + trim(strip_comment(line));
    }
    if (bracket_balance(value) != 0) {
      fail(line_no, "unbalanced brackets");
    }
    (*current)[key] = parse_value(value, line_no);
  }
  return doc;
}

TomlDocument parse_toml_file(const std::string & path)
{
  std::ifstream f(path);
  if (!f.is_open()) {
    throw std::runtime_error("[Errno 2] No such file or directory: '" + path + "'");
  }
  std::stringstream ss;
  ss << f.rdbuf();
  return parse_toml_string(ss.str());
}

}  // namespace cmr_rovernet_cpp
