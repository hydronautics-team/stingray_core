#pragma once

#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/u_int8.hpp"

namespace stingray_core::servo_device {

class ServoDevice {
   public:
    explicit ServoDevice(
        rclcpp::NodeOptions options = rclcpp::NodeOptions());

    void spin() { 
        rclcpp::spin(node_);
    }

    rclcpp::Logger get_logger() const { return node_->get_logger(); }

   private:
    void data_raw_callback(const std_msgs::msg::UInt8::ConstSharedPtr& msg);
    void publish_servo_control(const uint8_t control_signal);

    rclcpp::Node::SharedPtr node_;

    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr servo_control_pub_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr data_raw_sub_;
};

}  // namespace stingray_core::servo_device
