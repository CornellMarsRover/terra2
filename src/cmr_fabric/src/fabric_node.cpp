#include "cmr_fabric/fabric_node.hpp"

#include <chrono>

#include "cmr_msgs/srv/acquire_dependency.hpp"
#include "cmr_msgs/srv/release_dependency.hpp"
#include "cmr_utils/cmr_debug.hpp"
#include "cmr_utils/monad.hpp"
#include "cmr_utils/services.hpp"
#include "lifecycle_msgs/msg/state.hpp"

using namespace std::chrono_literals;
using lifecycle_msgs::msg::Transition;

constexpr auto param_config_path = "config_path";
constexpr auto param_composition_namespace = "composition_ns";
constexpr auto param_max_restarts = "restart_attempts";
constexpr auto param_restart_delay = "restart_delay";
constexpr auto param_num_attempted_restarts = "num_restarts";
constexpr auto param_config_data = "config_data";

namespace cmr::fabric
{

FabricNode::FabricNode(const std::optional<FabricNodeConfig>& config)
    : rclcpp_lifecycle::LifecycleNode(monad::value_or_else(
          monad::map(config, [](const auto& c) { return c.node_name; }), []() {
              return "fabric_untitled_" +
                     std::to_string(std::chrono::system_clock::now()
                                        .time_since_epoch()
                                        .count());
          }))
{
    m_dependency_manager = std::make_unique<DependencyHandler>(*this);
    declare_parameter(
        param_config_path,
        monad::bind(config, [](const auto& c) {
            if (std::holds_alternative<std::filesystem::path>(c.toml_config)) {
                return std::make_optional(
                    std::get<std::filesystem::path>(c.toml_config).string());
            } else {
                return std::optional<std::string>{};
            }
        }).value_or(""));
    declare_parameter(param_composition_namespace,
                      monad::map(config, [](const auto& c) {
                          return c.composition_namespace;
                      }).value_or(""));
    declare_parameter(
        param_config_data,
        monad::bind(config, [](const auto& c) {
            if (std::holds_alternative<std::string>(c.toml_config)) {
                return std::make_optional(std::get<std::string>(c.toml_config));
            } else {
                return std::optional<std::string>{};
            }
        }).value_or(""));
    declare_parameter(param_max_restarts, 2);
    declare_parameter(param_restart_delay, 0);
    declare_parameter(param_num_attempted_restarts, 0);
}

// Basic guarantee. This is ok because if this fails, entire node is reset anyway
bool set_config_params(FabricNode& node, const toml::Result& toml)
{
    const auto table = toml.table;

    if (!table) {
        RCLCPP_ERROR(node.get_logger(), "Failed to parse config file: %s",
                     toml.errmsg.c_str());
        return false;
    }

    const auto fault_handling_settings = table->getTable("fault_handling");
    const auto [ok, max_restart_attempts] =
        fault_handling_settings->getInt(param_max_restarts);
    if (!ok) {
        RCLCPP_ERROR(node.get_logger(),
                     "Failed to parse fault_handling.restart_attempts: %s",
                     toml.errmsg.c_str());
        return false;
    }
    node.set_parameter(rclcpp::Parameter(param_max_restarts, max_restart_attempts));
    CMR_LOG(INFO, "Set max restarts to %zd", max_restart_attempts);
    node.set_parameter({param_num_attempted_restarts, 0});

    const auto [delay_ok, restart_delay] =
        fault_handling_settings->getInt(param_restart_delay);
    if (!delay_ok) {
        RCLCPP_ERROR(node.get_logger(),
                     "Failed to parse fault_handling.restart_delay: %s",
                     toml.errmsg.c_str());
        return false;
    }
    node.set_parameter(rclcpp::Parameter(param_restart_delay, restart_delay));
    return true;
}

/**
 * @brief Parses the toml config file if one is specified, otherwise parses the
 * config data
 *
 * @param toml_path path to toml config file
 * @param toml_data string of toml config file data
 * @return toml::Result or std::nullopt
 */
inline auto parse_toml(const std::string& toml_path, const std::string& toml_data)
{
    if (toml_path.empty() && toml_data.empty()) {
        CMR_LOG(ERROR, "No config_path parameter specified");
        return std::optional<toml::Result>{};
    } else if (!toml_path.empty()) {
        return std::make_optional(toml::parseFile(toml_path));
    } else {
        return std::make_optional(toml::parse(toml_data));
    }
}

rclcpp_lifecycle::LifecycleNode::CallbackReturn FabricNode::on_configure(
    const rclcpp_lifecycle::State&)
{
    CMR_LOG(INFO, "Configuring fabric node");
    m_composition_namespace = get_parameter("composition_ns").as_string();
    m_recover_fault_client = this->create_client<cmr_msgs::srv::RecoverFault>(
        "/" + m_composition_namespace + "/recover_fault");
    CMR_LOG(INFO, "Created recover fault client targetting /%s/recover_fault",
            m_composition_namespace.c_str());

    const auto toml = parse_toml(get_parameter(param_config_path).as_string(),
                                 get_parameter(param_config_data).as_string());
    if (!toml) {
        return rclcpp_lifecycle::LifecycleNode::CallbackReturn::ERROR;
    }

    CMR_LOG(INFO, "About to set config params");
    if (!set_config_params(*this, *toml)) {
        // error logging done in set_config_params
        return rclcpp_lifecycle::LifecycleNode::CallbackReturn::ERROR;
    }

    const auto config = toml->table;

    const auto dependencies = config->getArray("dependencies");
    if (dependencies) {
        const auto deps = dependencies->getStringVector();
        m_dependency_manager->set_dependencies(deps->begin(), deps->end());
    }

    return configure(config)
               ? rclcpp_lifecycle::LifecycleNode::CallbackReturn::SUCCESS
               : rclcpp_lifecycle::LifecycleNode::CallbackReturn::ERROR;
}

rclcpp_lifecycle::LifecycleNode::CallbackReturn FabricNode::on_activate(
    const rclcpp_lifecycle::State&)
{
    if (!m_dependency_manager->acquire_all_dependencies()) {
        return rclcpp_lifecycle::LifecycleNode::CallbackReturn::ERROR;
    }
    return activate() ? rclcpp_lifecycle::LifecycleNode::CallbackReturn::SUCCESS
                      : rclcpp_lifecycle::LifecycleNode::CallbackReturn::ERROR;
}

rclcpp_lifecycle::LifecycleNode::CallbackReturn FabricNode::on_deactivate(
    const rclcpp_lifecycle::State&)
{
    if (!m_dependency_manager->release_all_dependencies() ||
        !m_dependency_manager->notify_deactivate(
            m_processing_fault ? DeactivationReason::Error
                               : DeactivationReason::Manual,
            static_cast<int32_t>(get_parameter(param_restart_delay).as_int()))) {
        return rclcpp_lifecycle::LifecycleNode::CallbackReturn::ERROR;
    }
    return deactivate() ? rclcpp_lifecycle::LifecycleNode::CallbackReturn::SUCCESS
                        : rclcpp_lifecycle::LifecycleNode::CallbackReturn::ERROR;
}

rclcpp_lifecycle::LifecycleNode::CallbackReturn FabricNode::on_cleanup(
    const rclcpp_lifecycle::State&)
{
    return cleanup() ? rclcpp_lifecycle::LifecycleNode::CallbackReturn::SUCCESS
                     : rclcpp_lifecycle::LifecycleNode::CallbackReturn::ERROR;
}

rclcpp_lifecycle::LifecycleNode::CallbackReturn FabricNode::on_shutdown(
    const rclcpp_lifecycle::State&)
{
    // We treat shutdown and cleanup the same way; just invoke cleanup()
    return cleanup() ? rclcpp_lifecycle::LifecycleNode::CallbackReturn::SUCCESS
                     : rclcpp_lifecycle::LifecycleNode::CallbackReturn::ERROR;
}

// Performs the necessary cleanup for `node` during the `ErrorProcessing` state
bool FabricNode::cleanup_on_error(const rclcpp_lifecycle::State& current_state)
{
    m_processing_fault = true;
    ALWAYS(this) { m_processing_fault = false; };
    using namespace lifecycle_msgs::msg;  // NOLINT(google-build-using-namespace)
    constexpr auto success =
        rclcpp_lifecycle::LifecycleNode::CallbackReturn::SUCCESS;
    constexpr auto error = rclcpp_lifecycle::LifecycleNode::CallbackReturn::ERROR;
    switch (current_state.id()) {
        case State::PRIMARY_STATE_UNCONFIGURED:
            return true;
        case State::PRIMARY_STATE_ACTIVE: {
            const auto deactive_code = on_deactivate(current_state);
            if (deactive_code == error) {
                return false;
            }
            return on_cleanup(current_state) == success;
        }
        case State::TRANSITION_STATE_ACTIVATING:
            return m_dependency_manager->release_all_dependencies() &&
                   m_dependency_manager->notify_deactivate(
                       DeactivationReason::Error,
                       static_cast<int32_t>(
                           get_parameter(param_restart_delay).as_int())) &&
                   on_cleanup(current_state) == success;
        case State::TRANSITION_STATE_DEACTIVATING:
        case State::PRIMARY_STATE_INACTIVE:
            return on_cleanup(current_state) == success;
        default:
            CMR_LOG(ERROR, "Reached on_error from state %d", current_state.id());
            return false;
    }
}

rclcpp_lifecycle::LifecycleNode::CallbackReturn FabricNode::on_error(
    const rclcpp_lifecycle::State& current_state)
{
    const auto cleanup_success = cleanup_on_error(current_state);
    if (cleanup_success) {
        // returning SUCCESS will tell ROS2 to move the node into the Unconfigured
        // state.
        // returning ERROR will tell ROS2 to move the node into the Finalized state
        // and prep for destruction
        return schedule_restart()
                   ? rclcpp_lifecycle::LifecycleNode::CallbackReturn::SUCCESS
                   : rclcpp_lifecycle::LifecycleNode::CallbackReturn::ERROR;
    }
    return rclcpp_lifecycle::LifecycleNode::CallbackReturn::ERROR;
}

bool FabricNode::schedule_restart()
{
    const int64_t num_restarts =
        get_parameter(param_num_attempted_restarts).as_int();
    CMR_LOG(INFO, "Scheduling restart");
    CMR_LOG(INFO, "num_restarts: %zd", num_restarts);
    const int64_t max_restart_attempts = get_parameter(param_max_restarts).as_int();
    CMR_LOG(INFO, "max_restart_attempts: %zd", max_restart_attempts);
    if (num_restarts >= max_restart_attempts) {
        // reset the counter in case the user wants to try and enable this again
        // later
        set_parameter(rclcpp::Parameter(param_num_attempted_restarts, 0));
        CMR_LOG(ERROR,
                "Node will not attempt to restart because it has restarted "
                "the maximum amount of times.");
        return false;
    }

    // make sure the fault handler is available before doing anything else
    while (!m_recover_fault_client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            CMR_LOG(ERROR,
                    "Interrupted while waiting for the fault handler. Not "
                    "scheduling restart.");
            return false;
        }
        CMR_LOG(INFO, "Fault handler not available, waiting again...");
    }

    set_parameter(rclcpp::Parameter("num_restarts", num_restarts + 1));

    // Best effort to send the restart request; don't block and wait for the
    // response though though because if it fails, this is emblematic of a more
    // serious problem.
    auto req = std::make_shared<cmr_msgs::srv::RecoverFault::Request>();
    req->node_name = get_name();
    req->restart_delay =
        static_cast<int32_t>(get_parameter(param_restart_delay).as_int());
    m_recover_fault_client->async_send_request(req);
    // do we want to wait for the response?
    return true;
}

void FabricNode::error_transition()
{
    m_processing_fault = true;
    ALWAYS(this) { m_processing_fault = false; };
    CMR_LOG(INFO, "Transitioning to ErrorProcessing");
    switch (get_current_state().id()) {
        case lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED:
            break;
        case lifecycle_msgs::msg::State::TRANSITION_STATE_ACTIVATING:
        case lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE:
            trigger_transition(Transition::TRANSITION_DEACTIVATE);
            [[fallthrough]];
        case lifecycle_msgs::msg::State::TRANSITION_STATE_DEACTIVATING:
        case lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE:
        case lifecycle_msgs::msg::State::TRANSITION_STATE_CONFIGURING:
            trigger_transition(Transition::TRANSITION_CLEANUP);
            break;
        default:
            CMR_LOG(FATAL, "Failed to make error transition, shutting down");
            trigger_transition(Transition::TRANSITION_DESTROY);
            return;
    }
    schedule_restart();
}
}  // namespace cmr::fabric
