#pragma once

#include "cmr_msgs/srv/recover_fault.hpp"
#include "cmr_utils/tomlcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace cmr::fabric {

class FabricNode : public rclcpp_lifecycle::LifecycleNode {
   public:
    FabricNode(std::string name) : rclcpp_lifecycle::LifecycleNode(name) {
        declare_parameter("config_path", "");
        declare_parameter("restart_attempts", 0);
        declare_parameter("restart_delay", 0);
        declare_parameter("num_restarts", 0);

        recover_fault_client =
            this->create_client<cmr_msgs::srv::RecoverFault>("/fabric/recover_fault");
    }

    virtual ~FabricNode() = default;

    virtual bool onConfigure(std::shared_ptr<toml::Table>) { return true; }

    virtual bool onActivate() { return true; }

    virtual bool onDeactivate() { return true; }

    virtual bool onShutdown() { return true; }

    /**
     * @brief Returns the vector of dependencies defined by this node. There is no
     * guarantee that everything in this list is a valid dependency (i.e. that they each
     * exist as a node).
     *
     * @return std::vector<std::string> list of dependency names
     */
    std::vector<std::string> get_dependencies() { return dependencies; }

    rclcpp_lifecycle::LifecycleNode::CallbackReturn on_configure(
        const rclcpp_lifecycle::State &) override;

    rclcpp_lifecycle::LifecycleNode::CallbackReturn on_activate(
        const rclcpp_lifecycle::State &) override;

    rclcpp_lifecycle::LifecycleNode::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State &) override;

    rclcpp_lifecycle::LifecycleNode::CallbackReturn on_cleanup(
        const rclcpp_lifecycle::State &) override;

    rclcpp_lifecycle::LifecycleNode::CallbackReturn on_shutdown(
        const rclcpp_lifecycle::State &) override;

    rclcpp_lifecycle::LifecycleNode::CallbackReturn on_error(
        const rclcpp_lifecycle::State &) override;

   private:
    std::shared_ptr<rclcpp::Client<cmr_msgs::srv::RecoverFault>> recover_fault_client;
    std::vector<std::string> dependencies;
    void scheduleRestart();
};

}  // namespace cmr::fabric
