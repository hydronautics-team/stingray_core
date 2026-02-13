#include <servo_device/servo_device.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto servo_node =
        std::make_shared<stingray_core::servo_device::ServoDevice>();
    servo_node->spin();
    rclcpp::shutdown();
    return 0;
}
