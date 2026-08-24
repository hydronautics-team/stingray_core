#pragma once

#include <algorithm>
#include <cstdint>
#include <memory>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"

#define DEFAULT_VOLTAGE_SCALE 1.0
#define DEFAULT_CURRENT_SCALE 0.0
#define DEFAULT_FILTER_WINDOW 0.0
#define DEFAULT_LOW_BATTERY_THRESHOLD 13.6
namespace stingray_core::battery_sensor
{

struct BatterySensorConfig
{
    BatterySensorConfig(const rclcpp::Node::SharedPtr &node)
        : voltage_scale(node->declare_parameter<double>("voltage_scale", DEFAULT_VOLTAGE_SCALE)),
          current_scale(node->declare_parameter<double>("current_scale", DEFAULT_CURRENT_SCALE)),
          filter_window(node->declare_parameter<double>("filter_window", DEFAULT_FILTER_WINDOW)),
          low_battery_threshold(node->declare_parameter<double>("low_battery_threshold", DEFAULT_LOW_BATTERY_THRESHOLD))
    {
        for (int i = 0; i < 9; ++i)
        {
            voltage_points[i] = node->declare_parameter<double>("battery_voltage_" + std::to_string(i), 0.0);

            percentage_points[i] = node->declare_parameter<double>("battery_percentage_" + std::to_string(i), 0.0);
        }
    }

    const double voltage_scale;
    const double current_scale;
    const double filter_window;
    const double low_battery_threshold;

    std::array<double, 9> voltage_points{};
    std::array<double, 9> percentage_points{};
};

class BatterySensor
{
public:
    explicit BatterySensor(rclcpp::NodeOptions options = rclcpp::NodeOptions());
    void spin() { rclcpp::spin(node_); }
    rclcpp::Logger get_logger() const { return node_->get_logger(); }

private:
    void data_raw_callback(const std_msgs::msg::UInt8MultiArray::ConstSharedPtr &msg);
    void process_battery_data(const uint16_t battery_raw_1, const uint16_t battery_raw_2);
    void publish_diagnostics(double voltage_1, double voltage_2);
    double calculate_percentage(double voltage) const;

    rclcpp::Node::SharedPtr node_;
    BatterySensorConfig config_;

    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr data_pub_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr data_raw_sub_;
};

} // namespace stingray_core::battery_sensor
