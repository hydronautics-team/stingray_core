#include <pressure_sensor/pressure_sensor.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto sensor_node =
        std::make_shared<stingray_core::pressure_sensor::PressureSensor>();
    sensor_node->spin();
    rclcpp::shutdown();
    return 0;
}