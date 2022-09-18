#pragma once

#include <chrono>
#include <string>

#include "rclcpp/rclcpp.hpp"

namespace cmr
{
/**
 * Spins up a service client owned by a temporary node and calls the service with the
 * given request. Returns the response if successful, or empty optional otherwise.
 *
 * **DATA RACE WARNING:** This function blocks until the service response is received
 * and **YIELDS TO ROS! THIS IS VERY IMPORTANT AND POSSIBLY DANGEROUS, DO NOT CALL
 * WITHIN A CRITICAL SECTION OR ANY AREA OF CODE THAT SHOULD NOT BE INTERUPTED!!!**
 *
 * **DEADLOCK WARNING:** This function creates a new node for the client and spins
 * it, this means that **THIS FUNCTION BLOCKS THE ENTIRE THREAD, PREVENTING ANY OTHER
 * ROS CALLBACKS FROM BEING INVOKED.** Thus any service dispatched from this function
 * **CANNOT RELY ON A SERVICE PROVIDED FROM THE SAME THREAD**
 *
 * This is quite possibly **THE MOST DANGEROUS** FUNCTION IN THE ENTIRE
 * CODEBASE, use with EXTREME CAUTION
 *
 * If you can use it, prefer to use an asynchronous service directly, rather than
 * using this function to block a response
 *
 * @tparam ServiceT The type of the service
 * @param service_name The name of the service
 * @param request The request to send
 *
 * @return The response from the service, or empty optional if the service call
 * failed
 */
template <typename SrvT>
std::optional<std::shared_ptr<typename SrvT::Response>> send_request(
    const std::string& service_name,
    const std::shared_ptr<typename SrvT::Request> request)
{
    // we need to make a new node for each client we create, so let's make one with a
    // unique name by using the current time.
    auto node = rclcpp::Node::make_shared(
        "service_client_" +
        std::to_string(std::chrono::system_clock::now().time_since_epoch().count()));
    auto client = node->create_client<SrvT>(service_name);

    using namespace std::chrono_literals;
    while (!client->wait_for_service(3s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                         "Interrupted while waiting for the service. Exiting.");
            return {};
        }
        RCLCPP_INFO(node->get_logger(), "service %s not available, waiting again...",
                    service_name.c_str());
    }

    auto future = client->async_send_request(request);
    RCLCPP_INFO(node->get_logger(), "Request sent to %s", service_name.c_str());
    if (rclcpp::spin_until_future_complete(node, future) !=
        rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(node->get_logger(), "failed request to %s",
                     service_name.c_str());
        return {};
    }
    return future.get();
}
}  // namespace cmr
