#include <battery_sensor/battery_sensor.hpp>

#include <algorithm>
#include <cstdint>
#include <utility>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"

namespace stingray_core::battery_sensor
{

BatterySensor::BatterySensor(rclcpp::NodeOptions options)
    : node_(rclcpp::Node::make_shared("battery_sensor", std::move(options))),
      config_(node_),
      data_pub_(node_->create_publisher<sensor_msgs::msg::BatteryState>("/power/battery/state", rclcpp::SensorDataQoS())),
      diagnostics_pub_(node_->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10)),
      data_raw_sub_(node_->create_subscription<std_msgs::msg::UInt8MultiArray>(
          "/power/battery/raw", rclcpp::SensorDataQoS(),
          [this](const std_msgs::msg::UInt8MultiArray::ConstSharedPtr &msg) { this->data_raw_callback(msg); }))
{

    RCLCPP_INFO(node_->get_logger(), "Battery sensor node initialized.");
    RCLCPP_INFO(node_->get_logger(), "voltage_scale: %f", config_.voltage_scale);
    RCLCPP_INFO(node_->get_logger(), "current_scale: %f", config_.current_scale);
    RCLCPP_INFO(node_->get_logger(), "filter_window: %f", config_.filter_window);
    RCLCPP_INFO(node_->get_logger(), "low_battery_threshold: %f", config_.low_battery_threshold);
}

void BatterySensor::data_raw_callback(const std_msgs::msg::UInt8MultiArray::ConstSharedPtr &msg)
{
    if (msg->data.size() < 4)
    {
        RCLCPP_WARN(node_->get_logger(), "Invalid battery packet size: %zu, expected at least 4 bytes", msg->data.size());

        return;
    }
    /*
     * Packet format:
     *
     * byte 0 -> Battery 1 LOW
     * byte 1 -> Battery 1 HIGH
     * byte 2 -> Battery 2 LOW
     * byte 3 -> Battery 2 HIGH
     *
     * uint16 = LOW | (HIGH << 8)
     */

    const uint16_t battery_raw_1 = static_cast<uint16_t>(msg->data[0]) | (static_cast<uint16_t>(msg->data[1]) << 8);

    const uint16_t battery_raw_2 = static_cast<uint16_t>(msg->data[2]) | (static_cast<uint16_t>(msg->data[3]) << 8);

    RCLCPP_DEBUG(node_->get_logger(), "Raw battery values: %u, %u", battery_raw_1, battery_raw_2);

    process_battery_data(battery_raw_1, battery_raw_2);
}

void BatterySensor::process_battery_data(const uint16_t battery_raw_1, const uint16_t battery_raw_2)
{
    const double voltage_1 = static_cast<double>(battery_raw_1) * config_.voltage_scale;

    const double voltage_2 = static_cast<double>(battery_raw_2) * config_.voltage_scale;

    const double percentage_1 = calculate_percentage(voltage_1);
    const double percentage_2 = calculate_percentage(voltage_2);

    const auto now = node_->now();

    sensor_msgs::msg::BatteryState battery1_msg;
    battery1_msg.header.stamp = now;
    battery1_msg.header.frame_id = "battery_system";
    battery1_msg.location = "1";
    battery1_msg.voltage = voltage_1;
    battery1_msg.percentage = percentage_1;
    battery1_msg.present = true;

    data_pub_->publish(battery1_msg);

    sensor_msgs::msg::BatteryState battery2_msg;
    battery2_msg.header.stamp = now;
    battery2_msg.header.frame_id = "battery_system";
    battery2_msg.location = "2";
    battery2_msg.voltage = voltage_2;
    battery2_msg.percentage = percentage_2;
    battery2_msg.present = true;

    data_pub_->publish(battery2_msg);

    publish_diagnostics(voltage_1, voltage_2);

    RCLCPP_INFO(node_->get_logger(), "Battery 1: %.2f V (%.1f%%), Battery 2: %.2f V (%.1f%%)", voltage_1, percentage_1 * 100.0,
                voltage_2, percentage_2 * 100.0);
}

void BatterySensor::publish_diagnostics(const double voltage_1, const double voltage_2)
{
    diagnostic_msgs::msg::DiagnosticArray diagnostic_array;
    diagnostic_array.header.stamp = node_->now();

    // ============================================================
    // Battery 1
    // ============================================================
    diagnostic_msgs::msg::DiagnosticStatus battery1_status;
    battery1_status.name = "Battery 1";
    battery1_status.hardware_id = "battery_1";
    if (voltage_1 < config_.low_battery_threshold)
    {
        battery1_status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;

        battery1_status.message = "Low battery voltage";
    }
    else
    {
        battery1_status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;

        battery1_status.message = "Battery voltage OK";
    }
    diagnostic_msgs::msg::KeyValue battery1_voltage;
    battery1_voltage.key = "Voltage";
    battery1_voltage.value = std::to_string(voltage_1) + " V";
    diagnostic_msgs::msg::KeyValue battery1_percentage;
    battery1_percentage.key = "Percentage";
    battery1_percentage.value = std::to_string(calculate_percentage(voltage_1)) + " %";
    battery1_status.values.push_back(battery1_voltage);
    battery1_status.values.push_back(battery1_percentage);
    diagnostic_array.status.push_back(battery1_status);

    // ============================================================
    // Battery 2
    // ============================================================
    diagnostic_msgs::msg::DiagnosticStatus battery2_status;
    battery2_status.name = "Battery 2";
    battery2_status.hardware_id = "battery_2";
    if (voltage_2 < config_.low_battery_threshold)
    {
        battery2_status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;

        battery2_status.message = "Low battery voltage";
    }
    else
    {
        battery2_status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;

        battery2_status.message = "Battery voltage OK";
    }
    diagnostic_msgs::msg::KeyValue battery2_voltage;
    battery2_voltage.key = "Voltage";
    battery2_voltage.value = std::to_string(voltage_2) + " V";
    diagnostic_msgs::msg::KeyValue battery2_percentage;
    battery2_percentage.key = "Percentage";
    battery2_percentage.value = std::to_string(calculate_percentage(voltage_2)) + " %";
    battery2_status.values.push_back(battery2_voltage);
    battery2_status.values.push_back(battery2_percentage);
    diagnostic_array.status.push_back(battery2_status);
    diagnostics_pub_->publish(std::move(diagnostic_array));
}

double BatterySensor::calculate_percentage(const double voltage) const
{
    const auto &voltages = config_.voltage_points;
    const auto &percentages = config_.percentage_points;

    if (voltage <= voltages.front())
        return 0.0;

    if (voltage >= voltages.back())
        return 100.0;

    for (std::size_t i = 1; i < voltages.size(); ++i)
    {
        if (voltage <= voltages[i])
        {
            const double v1 = voltages[i - 1];
            const double v2 = voltages[i];

            const double p1 = percentages[i - 1];
            const double p2 = percentages[i];

            const double percentage = p1 + (voltage - v1) * (p2 - p1) / (v2 - v1);

            return percentage;
        }
    }

    return 0.0;
}

} // namespace stingray_core::battery_sensor
