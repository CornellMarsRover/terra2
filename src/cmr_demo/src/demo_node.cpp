#include <cstdio>
#include "cmr_fabric/fabric_node.hpp"
#include "rclcpp/rclcpp.hpp"

class DemoNode : public cmr::fabric::FabricNode
{
public:
  DemoNode()
  : cmr::fabric::FabricNode::FabricNode("demo") {}

  bool onConfigure(std::shared_ptr<toml::Table> config) override
  {
    printf("Configuring...\n");
    printf(
      "Restart attempts: %ld\n",
      config->getTable("fault_handling")->getInt("restart_attempts").second);
    return true;
  }

  bool onActivate() override
  {
    printf("Activating...\n");
    throw std::invalid_argument("Activation failed");
    return true;
  }

  bool onDeactivate() override
  {
    printf("Deactivating...\n");
    return true;
  }

  bool onShutdown() override
  {
    printf("Shutting down...\n");
    return true;
  }

};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  printf("The demo_node node.\n");

  auto node = std::make_shared<DemoNode>();

  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
