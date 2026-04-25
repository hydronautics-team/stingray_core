#include <display_device/display_device_node.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto display_node =
        std::make_shared<stingray_core::display_device::DisplayDevice>();
    display_node->spin();
    rclcpp::shutdown();
    return 0;
}