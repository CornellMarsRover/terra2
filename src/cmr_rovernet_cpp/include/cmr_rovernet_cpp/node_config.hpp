// Helpers mirroring the Python nodes' `_load_node_config` / `_setting`
// pattern: values in the [node] table of the TOML file named by the
// `config_path` parameter override the ROS parameters of the same name.
#pragma once

#include <cstdint>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "cmr_rovernet_cpp/toml_lite.hpp"

namespace cmr_rovernet_cpp {

struct ConfigLoadOptions
{
  bool expand_user = true;   // Path(config_path).expanduser()
  bool check_exists = true;  // warn "config_path does not exist" if missing
  bool log_loaded = true;    // info "Loaded node config from ..."
};

/// Reads parameter `config_path` (must already be declared) and returns its
/// [node] table, or an empty table with a warning on any problem.
TomlTable load_node_config(rclcpp::Node & node, const ConfigLoadOptions & opts = {});

/// Declare a parameter with dynamic typing so int/double/bool/string
/// overrides are all accepted and converted Python-style on read.
template<typename T>
void declare_lenient(rclcpp::Node & node, const std::string & name, const T & default_value)
{
  rcl_interfaces::msg::ParameterDescriptor desc;
  desc.dynamic_typing = true;
  node.declare_parameter(name, rclcpp::ParameterValue(default_value), desc);
}

// `_setting(name, config)` followed by float()/int()/bool()/str().
double setting_double(rclcpp::Node & node, const std::string & name, const TomlTable & config);
int64_t setting_int(rclcpp::Node & node, const std::string & name, const TomlTable & config);
bool setting_bool(rclcpp::Node & node, const std::string & name, const TomlTable & config);
std::string setting_string(
  rclcpp::Node & node, const std::string & name, const TomlTable & config);

// Python-style conversions of a ROS parameter value.
double param_to_double(const rclcpp::ParameterValue & v);
int64_t param_to_int(const rclcpp::ParameterValue & v);
bool param_to_bool(const rclcpp::ParameterValue & v);
std::string param_to_string(const rclcpp::ParameterValue & v);

}  // namespace cmr_rovernet_cpp
