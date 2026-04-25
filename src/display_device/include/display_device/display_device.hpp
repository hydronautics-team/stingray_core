#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include "display_device/msg/display_status.hpp"

#define DEFAULT_PUBLISH_RATE_HZ 60
#define DEFAULT_MAX_LINE_LENGTH 15
#define DEFAULT_DEDUPE true
#define DEFAULT_DISPLAY_FIELDS 13.6

namespace stingray_core::display_device {

struct DisplayDeviceConfig{
    DisplayDeviceConfig(const rclcpp::Node::SharedPtr& node)
    : publish_rate_hz(node->declare_parameter<unsigned>("publish_rate_hz", DEFAULT_PUBLISH_RATE_HZ)),
      max_line_length(node->declare_parameter<unsigned>("max_line_length", DEFAULT_MAX_LINE_LENGTH)),
      dedupe(node->declare_parameter<bool>("dedupe", DEFAULT_DEDUPE)),
      display_fields(node->declare_parameter<double>("display_fields", DEFAULT_DISPLAY_FIELDS))
    {}
    unsigned publish_rate_hz;
    unsigned max_line_length;
    bool dedupe;
    const double display_fields; // FIXME choose the field type
};

class DisplayDevice {
   private:
   struct SystemStatus {
        unsigned battery1_percentage = 0;
        unsigned battery2_percentage = 0;
        unsigned battery1_emergency = 0;
        unsigned battery2_emergency = 0;
        std::string operation_mode = "UNKNOWN";
        double temperature = 0.0;
        double pressure = 0.0;
        double depth = 0.0;
        std::string warnings;
        std::string errors;
        
        bool update = false;
        
        bool operator!=(const SystemStatus& other) const {
            return battery1_percentage != other.battery1_percentage ||
                   battery2_percentage != other.battery2_percentage ||
                   battery1_emergency != other.battery1_emergency ||
                   battery2_emergency != other.battery2_emergency ||
                   operation_mode != other.operation_mode ||
                   std::abs(temperature - other.temperature) > 0.01 ||
                   std::abs(pressure - other.pressure) > 0.001 ||
                   std::abs(depth - other.depth) > 0.001 ||
                   warnings != other.warnings ||
                   errors != other.errors;
        }
    };

   public:
    explicit DisplayDevice(
        rclcpp::NodeOptions options = rclcpp::NodeOptions());

    void spin() { 
        rclcpp::spin(node_);
    }

    rclcpp::Logger get_logger() const { return node_->get_logger(); }

   private:
    void init_subscriptions();
    void battery_status_callback(
        const sensor_msgs::msg::BatteryState::ConstSharedPtr& msg);
    void low_voltage_callback(const std_msgs::msg::Int32MultiArray::ConstSharedPtr& msg);

    display_device::msg::DisplayStatus format_display_message(const SystemStatus& status);
    std::string serialize_display_message(const display_device::msg::DisplayStatus& msg);
    std::string truncate_string(const std::string& str, size_t max_length);

    bool should_publish(const display_device::msg::DisplayStatus& msg);
    void update_display();

    SystemStatus current_status_;
    SystemStatus last_published_status_;
    std::string last_serialized_status_;

    rclcpp::Node::SharedPtr node_;
    DisplayDeviceConfig config_;

    rclcpp::Publisher<display_device::msg::DisplayStatus>::SharedPtr display_status_pub_;

    rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_status_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr low_voltage_sub_;

    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace stingray_core::panel_device
