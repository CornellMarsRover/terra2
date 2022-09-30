#include "cmr_fabric/fault_handler.hpp"
#include "cmr_utils/cmr_debug.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    const auto node = std::make_shared<cmr::fabric::FaultHandler>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}