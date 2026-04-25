#include "lights_device/lights_device.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto lights_node = std::make_shared<stingray_core::lights_device::LightsControl>();
    lights_node->spin();
    rclcpp::shutdown();
    return 0;
}
