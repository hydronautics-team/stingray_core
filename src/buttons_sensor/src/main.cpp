#include <buttons_sensor/buttons_sensor.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto sensor_node =
        std::make_shared<stingray_core::buttons_sensor::ButtonsSensor>();
    sensor_node->spin();
    rclcpp::shutdown();
    return 0;
}
