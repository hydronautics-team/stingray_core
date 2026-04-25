#pragma once

#include "chrono"
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

    void start_blinking(int period_ms, uint8_t brightness);
    void stop_blinking();
    void blink_timer_callback();

    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr lights_pub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr mode_sub_;

    rclcpp::TimerBase::SharedPtr blink_timer_n_times_;
    bool is_blinking_ = false;
    bool blink_state_ = false;
    uint8_t blink_brightness_ = 255;
    int blink_period_ms_ = 500;
};

} // namespace stingray_core::lights_device
