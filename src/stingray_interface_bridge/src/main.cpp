#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "stingray_interface_bridge/bridge_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<StingrayInterfaceBridge>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
