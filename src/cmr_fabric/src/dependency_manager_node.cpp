#include "cmr_fabric/dependency_manager.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    const auto node = std::make_shared<cmr::fabric::DependencyManager>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}