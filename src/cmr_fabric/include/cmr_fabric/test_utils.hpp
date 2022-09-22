#pragma once
#include <optional>
#include <sstream>

#include "cmr_fabric/fabric_node.hpp"
#include "cmr_utils/thread_wrapper.hpp"

/**
 * @brief Configuration parameters for a node
 */
struct NodeConfig {
    const char* node_name;
    const char* node_namespace;
};

/**
 * @brief Create a thread for a node, passing it `config` to its constructor
 *
 * @param end_test an atomic flag that is signalled when the test is over. Signaling
 * this flag will stop spinning the node.
 * @param config the node configuration, passed to the node constructor
 * @tparam NodeT the type of the node to create
 * @return auto
 */
template <typename NodeT>
auto create_test_thread(const std::atomic<bool>& end_test,
                        const cmr::fabric::FabricNodeConfig& config)
{
    JThread test_thread([&end_test, config = std::make_optional(config)]() {
        auto test_node = std::make_shared<NodeT>(config);
        while (!end_test) {
            rclcpp::spin_some(test_node->get_node_base_interface());
        }
    });
    return test_thread;
}

/**
 * @brief Creates a FabricNode config by formatting a toml string
 *
 * @tparam Formats
 * @param node_config node name and namespace
 * @param config the configuration string, with format specifiers such as `%d`
 * @param formats optional arguments to format into `config`. Node that all strings
 * should be first converted to `const char*` C strings
 *
 * ## Example
 *
 * ```C++
 * const auto node_config = NodeConfig{"test_node", "test_ns"};
 * const auto config_str = R"(
 * dependencies = [%s]
 *
 * [fault_handling]
 * restart_attempts = 2
 * restart_delay = 2
 * )";
 *
 * const auto formatted_config = create_config(node_config, config_str,
 * "\"test_dep\"");
 *
 * // formatted_config.toml_config now contains a string which is:
 * // dependencies = ["test_dep"]
 * //
 * // [fault_handling]
 * // restart_attempts = 2
 * // restart_delay = 2
 *
 *
 * ```
 */
template <typename... Formats>
auto create_config(NodeConfig node_config, const char* const config,
                   Formats&&... formats)
{
    std::array<char, 2048> config_buffer = {};
#pragma GCC diagnostic push
#pragma clang diagnostic push
#pragma GCC diagnostic ignored "-Wformat-security"
#pragma clang diagnostic ignored "-Wformat-security"
    std::snprintf(config_buffer.data(), sizeof(config_buffer), config,
                  std::forward<Formats>(formats)...);
#pragma GCC diagnostic pop
#pragma clang diagnostic pop
    return cmr::fabric::FabricNodeConfig{node_config.node_name,
                                         node_config.node_namespace,
                                         {std::string(config_buffer.data())}};
}

/**
 * @brief Creates a list of strings by inserting string between quotes, and
 * separating them with commas
 *
 * ## Example
 *
 * ```C++
 * std::stringstream ss;
 * const auto name = "test";
 * add_to_stream(ss, 10, "hello", name);
 * ss.str() == "\"10\",\"hello\",\"test\",";
 * ```
 *
 * @tparam Args
 * @param stream the stream to output the result to
 * @param name the first name to add to the list
 * @param rest the rest of the names to add to the list. Must be strings
 */
template <typename... Args>
void add_to_toml_list(std::stringstream& stream, const std::string& name,
                      Args&&... rest)
{
    stream << '"' << name << '"' << ',';
    if constexpr (sizeof...(rest) > 0) {
        add_to_toml_list(stream, std::forward<Args>(rest)...);
    }
}