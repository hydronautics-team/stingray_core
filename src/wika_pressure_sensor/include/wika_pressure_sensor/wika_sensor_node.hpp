#pragma once

#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/float64.hpp"
#include <std_msgs/msg/string.hpp>

namespace stingray_core::wika_pressure_sensor {

class WikaPressureSensorNode : public rclcpp::Node
{
public:
    WikaPressureSensorNode();

private:
    void data_raw_callback_(const std_msgs::msg::String::SharedPtr msg);
    
    double parse_wika_data_(const std::string& raw_data);
    void publish_depth_(double depth);

    double depth_coefficient_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr depth_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr data_raw_sub_;
};

}