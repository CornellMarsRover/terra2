#include "cmr_demo/gps_fake.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<cmr::GpsFake>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}