#pragma once

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"

namespace stingray_core::lights_device
{

class LightsControl
{
public:
    explicit LightsControl(rclcpp::NodeOptions options = rclcpp::NodeOptions());

    void spin() { rclcpp::spin(node_); }

    rclcpp::Logger get_logger() const { return node_->get_logger(); }

private:
    void data_raw_callback(const std_msgs::msg::Int32::ConstSharedPtr &msg);
    void publish_cmd(uint8_t command);

    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr lights_pub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr mode_sub_;
};

} // namespace stingray_core::lights_device
