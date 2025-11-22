#include <pressure_sensor/pressure_sensor_node.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto sensor_node =
        std::make_shared<stingray_core::pressure_sensor::PressureSensorNode>();
    rclcpp::spin(sensor_node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}