#include "cmr_fabric/lifecycle_manager.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<cmr::fabric::LifecycleManager>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}