#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>

#define DEFAULT_DUMP_PARAM 1.0

namespace stingray_core::pressure_sensor {

struct PressureSensorConfig{
    PressureSensorConfig(const rclcpp::Node::SharedPtr& node)
    : dump_param(node->declare_parameter<double>("dump_param", DEFAULT_DUMP_PARAM)),
      data_topic(node->declare_parameter<std::string>("data_topic", "/data_raw"))
    {}
    const double  dump_param;
    const std::string data_topic;
};

class PressureSensor {
   public:
    explicit PressureSensor(
        rclcpp::NodeOptions options = rclcpp::NodeOptions());

    void spin() { 
        rclcpp::spin(node_);
    }

    rclcpp::Logger get_logger() const { return node_->get_logger(); }

   private:
    void data_raw_callback(const std_msgs::msg::String::ConstSharedPtr& msg);
    void publish_depth(const double depth);

    rclcpp::Node::SharedPtr node_;
    PressureSensorConfig config_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr depth_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr data_raw_sub_;
};

}  // namespace stingray_core::pressure_sensor
