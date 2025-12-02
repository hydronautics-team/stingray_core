#include <battery_sensor/battery_sensor_node.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto battery_node =
        std::make_shared<stingray_core::battery_sensor::BatterySensorNode>();
    rclcpp::spin(battery_node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}