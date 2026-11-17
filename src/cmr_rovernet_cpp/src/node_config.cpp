#include "cmr_rovernet_cpp/node_config.hpp"

#include <sys/stat.h>

#include <cmath>
#include <cstdlib>
#include <sstream>

namespace cmr_rovernet_cpp {
namespace {

std::string expand_user(const std::string & path)
{
  if (path.empty() || path[0] != '~') {
    return path;
  }
  if (path.size() > 1 && path[1] != '/') {
    return path;  // ~user form: left as-is
  }
  const char * home = std::getenv("HOME");
  if (!home) {
    return path;
  }
  return std::string(home) + path.substr(1);
}

bool path_exists(const std::string & path)
{
  struct stat st {};
  return ::stat(path.c_str(), &st) == 0;
}

}  // namespace

TomlTable load_node_config(rclcpp::Node & node, const ConfigLoadOptions & opts)
{
  const std::string raw = param_to_string(node.get_parameter("config_path").get_parameter_value());
  if (raw.empty()) {
    return {};
  }
  const std::string path = opts.expand_user ? expand_user(raw) : raw;
  if (opts.check_exists && !path_exists(path)) {
    RCLCPP_WARN(node.get_logger(), "config_path does not exist: %s", path.c_str());
    return {};
  }

  TomlDocument doc;
  try {
    doc = parse_toml_file(path);
  } catch (const std::exception & exc) {
    RCLCPP_WARN(node.get_logger(), "Failed to load config_path %s: %s", path.c_str(), exc.what());
    return {};
  }

  if (doc.root.count("node")) {
    RCLCPP_WARN(node.get_logger(), "config_path %s has no valid [node] table", path.c_str());
    return {};
  }
  const auto it = doc.tables.find("node");
  if (it == doc.tables.end()) {
    return {};
  }
  if (opts.log_loaded) {
    RCLCPP_INFO(node.get_logger(), "Loaded node config from %s", path.c_str());
  }
  return it->second;
}

double param_to_double(const rclcpp::ParameterValue & v)
{
  switch (v.get_type()) {
    case rclcpp::ParameterType::PARAMETER_DOUBLE: return v.get<double>();
    case rclcpp::ParameterType::PARAMETER_INTEGER: return static_cast<double>(v.get<int64_t>());
    case rclcpp::ParameterType::PARAMETER_BOOL: return v.get<bool>() ? 1.0 : 0.0;
    case rclcpp::ParameterType::PARAMETER_STRING: return std::stod(v.get<std::string>());
    default: break;
  }
  throw std::invalid_argument("parameter is not convertible to float");
}

int64_t param_to_int(const rclcpp::ParameterValue & v)
{
  switch (v.get_type()) {
    case rclcpp::ParameterType::PARAMETER_INTEGER: return v.get<int64_t>();
    case rclcpp::ParameterType::PARAMETER_DOUBLE:
      return static_cast<int64_t>(std::trunc(v.get<double>()));
    case rclcpp::ParameterType::PARAMETER_BOOL: return v.get<bool>() ? 1 : 0;
    case rclcpp::ParameterType::PARAMETER_STRING: return std::stoll(v.get<std::string>());
    default: break;
  }
  throw std::invalid_argument("parameter is not convertible to int");
}

bool param_to_bool(const rclcpp::ParameterValue & v)
{
  switch (v.get_type()) {
    case rclcpp::ParameterType::PARAMETER_BOOL: return v.get<bool>();
    case rclcpp::ParameterType::PARAMETER_INTEGER: return v.get<int64_t>() != 0;
    case rclcpp::ParameterType::PARAMETER_DOUBLE: return v.get<double>() != 0.0;
    case rclcpp::ParameterType::PARAMETER_STRING: return !v.get<std::string>().empty();
    case rclcpp::ParameterType::PARAMETER_NOT_SET: return false;
    default: return true;
  }
}

std::string param_to_string(const rclcpp::ParameterValue & v)
{
  switch (v.get_type()) {
    case rclcpp::ParameterType::PARAMETER_STRING: return v.get<std::string>();
    case rclcpp::ParameterType::PARAMETER_NOT_SET: return "None";
    case rclcpp::ParameterType::PARAMETER_BOOL: return v.get<bool>() ? "True" : "False";
    case rclcpp::ParameterType::PARAMETER_INTEGER: return std::to_string(v.get<int64_t>());
    case rclcpp::ParameterType::PARAMETER_DOUBLE: {
        std::ostringstream o;
        o << v.get<double>();
        return o.str();
      }
    default: return rclcpp::to_string(v);
  }
}

double setting_double(rclcpp::Node & node, const std::string & name, const TomlTable & config)
{
  const auto it = config.find(name);
  if (it != config.end()) {
    return it->second.as_double();
  }
  return param_to_double(node.get_parameter(name).get_parameter_value());
}

int64_t setting_int(rclcpp::Node & node, const std::string & name, const TomlTable & config)
{
  const auto it = config.find(name);
  if (it != config.end()) {
    return it->second.as_int();
  }
  return param_to_int(node.get_parameter(name).get_parameter_value());
}

bool setting_bool(rclcpp::Node & node, const std::string & name, const TomlTable & config)
{
  const auto it = config.find(name);
  if (it != config.end()) {
    return it->second.as_bool();
  }
  return param_to_bool(node.get_parameter(name).get_parameter_value());
}

std::string setting_string(
  rclcpp::Node & node, const std::string & name, const TomlTable & config)
{
  const auto it = config.find(name);
  if (it != config.end()) {
    return it->second.as_string();
  }
  return param_to_string(node.get_parameter(name).get_parameter_value());
}

}  // namespace cmr_rovernet_cpp
