// Standalone-executable main for the cmr_zed_cpp nodes.
//
// Unlike the rclcpp_components-generated main, construction failures (the
// ZED nodes throw when the camera / tracking / mapping cannot be enabled) are
// caught, logged FATAL, and turned into a clean shutdown with exit code 1
// instead of escaping to std::terminate().
#pragma once

#include <exception>
#include <memory>
#include <type_traits>

#include <rclcpp/rclcpp.hpp>

namespace cmr_zed_cpp
{

template<typename NodeT, typename ExecutorT = rclcpp::executors::SingleThreadedExecutor>
int run_standalone(int argc, char ** argv, const char * exe_name, size_t num_threads = 0)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<NodeT> node;
  try {
    node = std::make_shared<NodeT>(rclcpp::NodeOptions());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger(exe_name), "Failed to start: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  auto exec_ptr = [num_threads]() {
      if constexpr (std::is_same_v<ExecutorT, rclcpp::executors::MultiThreadedExecutor>) {
        return std::make_unique<ExecutorT>(rclcpp::ExecutorOptions(), num_threads);
      } else {
        (void)num_threads;
        return std::make_unique<ExecutorT>();
      }
    }();
  ExecutorT & exec = *exec_ptr;
  exec.add_node(node);
  exec.spin();
  exec.remove_node(node);
  node.reset();  // closes the ZED before the context goes away
  rclcpp::shutdown();
  return 0;
}

}  // namespace cmr_zed_cpp
