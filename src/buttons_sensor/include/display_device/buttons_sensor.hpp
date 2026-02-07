#pragma once

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include <std_msgs/msg/string.hpp> 
#include "vector"
#include "string"
#include "unordered_map"

namespace stingray_core::buttons_sensor {

static const std::unordered_map<int, std::string> DEFAULT_BUTTONS_MAP = {
    {1, "MISSION_SELECT"},
    {2, "MISSION_CONFIRM"},
    {3, "MISSION_ABORT"},
    {4, "CALIBRATION_START"},
    {5, "SENSOR_TEST"}
};

struct ButtonsSensorConfig{
    ButtonsSensorConfig(const rclcpp::Node::SharedPtr& node)
    : buttons_map(node->declare_parameter<std::unordered_map<int, std::string>>("buttons_map", DEFAULT_BUTTONS_MAP))
    {}

    std::unordered_map<int, std::string> buttons_map;
};

class ButtonsSensor {

public:
    explicit DisplayDevice(
        rclcpp::NodeOptions options = rclcpp::NodeOptions());

    void spin() { 
        rclcpp::spin(node_);
    }

    rclcpp::Logger get_logger() const { return node_->get_logger(); }
    
private:
    rclcpp::Node::SharedPtr node_;
    ButtonsSensorConfig config_;

    std::unordered_map<int32_t, std::string> buttons_map_;
    
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mission_events_pub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr buttons_sub_;
    
    void buttons_callback(const std_msgs::msg::Int32::ConstSharedPtr msg);
};

} // namespace stingray_core::buttons_sensor
