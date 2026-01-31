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

struct BatterySensorConfig {
    PressureSensorConfig(const rclcpp::Node::SharedPtr& node)
    : voltage_scale(node->declare_parameter<double>("voltage_scale", DEFAULT_VOLTAGE_SCALE)),
    current_scale(node->declare_parameter<double>("current_scale", DEFAULT_CURRENT_SCALE)),
    filter_window(node->declare_parameter<double>("filter_window", DEFAULT_FILTER_WINDOW)),
    low_battery_threshold(node->declare_parameter<double>("low_battery_threshold", DEFAULT_LOW_BATTERY_THRESHOLD))
    {}
    const double voltage_scale;
    const double current_scale;
    const double filter_window;
    const double low_battery_threshold;
}

class BatterySensor {
   public:
    explicit BatterySensor(
        rclcpp::NodeOptions options = rclcpp::NodeOptions());

    void spin() { 
        rclcpp::spin(node_);
    }

    rclcpp::Logger get_logger() const { return node_->get_logger(); }

   private:
    void data_raw_callback(
        const std_msgs::msg::Float64MultiArray::ConstSharedPtr& msg);
    void process_battery_data(const double voltage_raw_1,
                              const double voltage_raw_2);

    rclcpp::Node::SharedPtr node_;
    BatterySensorConfig config_;

    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr data_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr
        low_voltage_pub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr
        data_raw_sub_;
};

}  // namespace stingray_core::battery_sensor