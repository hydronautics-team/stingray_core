#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#define DEFAULT_VOLTAGE_SCALE 1.0
#define DEFAULT_CURRENT_SCALE 0.0
#define DEFAULT_FILTER_WINDOW 0.0
#define DEFAULT_LOW_BATTERY_THRESHOLD 13.6

namespace stingray_core::battery_sensor {

class BatterySensorNode {
   public:
    explicit BatterySensorNode(
        rclcpp::NodeOptions options = rclcpp::NodeOptions());

    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
    get_node_base_interface() {
        return node_->get_node_base_interface();
    }

    rclcpp::Logger get_logger() const { return node_->get_logger(); }

   private:
    void data_raw_callback(
        const std_msgs::msg::Float64MultiArray::ConstSharedPtr& msg);
    void process_battery_data(const double voltage_raw_1,
                              const double voltage_raw_2);

    double voltage_scale_;
    double current_scale_;
    double filter_window_;
    double low_battery_threshold_;

    std::shared_ptr<rclcpp::Node> node_;

    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr data_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr
        low_voltage_pub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr
        data_raw_sub_;
};

}  // namespace stingray_core::battery_sensor