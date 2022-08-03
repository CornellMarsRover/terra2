#pragma once

#include <string>
#include <chrono>
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

namespace cmr
{
/**
 * Spins up a service client owned by a temporary node and calls the service with the given request.
 * Returns the response if successful, or nullptr otherwise.
 */
template<typename SrvT>
std::shared_ptr<typename SrvT::Response> sendRequest(
  const std::string & service_name,
  const std::shared_ptr<typename SrvT::Request> request)
{
  // we need to make a new node for each client we create, so let's make one with a unique name by
  // using the current time.
  auto node =
    rclcpp::Node::make_shared(
    "service_client_" +
    std::to_string(std::chrono::system_clock::now().time_since_epoch().count()));
  auto client = node->create_client<SrvT>(service_name);

  while (!client->wait_for_service(3s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(
        rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return nullptr;
    }
    RCLCPP_INFO(node->get_logger(), "service not available, waiting again...");
  }

  auto future = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node, future) != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "failed request to %s", service_name.c_str());
    return nullptr;
  }
  return future.get();
}
}
