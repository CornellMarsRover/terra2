#pragma once

#include <chrono>
#include <string>

#include "rclcpp/rclcpp.hpp"

namespace cmr
{

/**
 * @defgroup ServiceUtils
 * Utility functions for interacting with ROS services by using a temporary node.
 * @{
 */

/**
 * @brief Helper function for `send_request`
 * Spins the node until the future is ready
 *
 * @tparam FutureT
 * @param node the node that sent the service request
 * @param future the future of the async service call
 * @param service_name the name of the service, for logging purposes
 * @return an optional containing the response if successful, or empty optional
 */
template <typename FutureT>
static auto wait_for_future(std::shared_ptr<rclcpp::Node> node, FutureT&& future,
                            const std::string& service_name,
                            std::chrono::seconds timeout = std::chrono::seconds(5))
{
    using namespace std::chrono_literals;
    RCLCPP_INFO(node->get_logger(), "Request sent to %s", service_name.c_str());
    const auto start = std::chrono::system_clock::now();
    while (rclcpp::ok() && std::chrono::system_clock::now() - start < timeout) {
        const auto status = rclcpp::spin_until_future_complete(node, future, 1s);
        if (status == rclcpp::FutureReturnCode::SUCCESS) {
            // RCLCPP_INFO(node->get_logger(), "Response received from %s",
            //             service_name.c_str());
            return std::make_optional(future.get());
        } else if (status == rclcpp::FutureReturnCode::INTERRUPTED) {
            RCLCPP_ERROR(node->get_logger(),
                         "Interrupted while waiting for service %s. "
                         "Exiting.",
                         service_name.c_str());
            return std::optional<decltype(future.get())>{};
        } else {
            RCLCPP_INFO(node->get_logger(), "Service %s timed out, waiting again...",
                        service_name.c_str());
        }
    }
    RCLCPP_INFO(node->get_logger(), "Service %s timed out.", service_name.c_str());
    return std::optional<decltype(future.get())>{};
}
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
    const std::shared_ptr<typename SrvT::Request> request,
    std::chrono::seconds timeout = std::chrono::seconds(5))
{
    // we need to make a new node for each client we create, so let's make one with a
    // unique name by using the current time.
    auto node = rclcpp::Node::make_shared(
        "service_client_" +
        std::to_string(std::chrono::system_clock::now().time_since_epoch().count()));
    auto client = node->create_client<SrvT>(service_name);

    using namespace std::chrono_literals;
    const auto start = std::chrono::system_clock::now();
    bool wait_success = false;
    while (!(wait_success = client->wait_for_service(1s)) &&
           std::chrono::system_clock::now() - start < timeout) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                         "Interrupted while waiting for the service. Exiting.");
            return {};
        }
        RCLCPP_INFO(node->get_logger(), "Service %s not available, waiting again...",
                    service_name.c_str());
    }
    if (wait_success) {
        auto future = client->async_send_request(request);
        return wait_for_future(node, std::move(future), service_name);
    } else {
        RCLCPP_ERROR(node->get_logger(), "Service %s timed out.",
                     service_name.c_str());
        return {};
    }
}

/** @} */
}  // namespace cmr
