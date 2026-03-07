#include <battery_sensor/battery_sensor.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto battery_node =
        std::make_shared<stingray_core::battery_sensor::BatterySensor>();
    battery_node->spin();
    rclcpp::shutdown();
    return 0;
}