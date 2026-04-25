#pragma once

#include "chrono"
#include "memory"
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
    void mode_callback(const std_msgs::msg::Int32::ConstSharedPtr &msg);
    void brightness_callback(const std_msgs::msg::Int32::ConstSharedPtr &msg);
    void publish_cmd(uint8_t command);

    void start_blinking(int period_ms);
    void stop_blinking();
    void blink_timer_callback();

    void start_gradient(int step_ms = 10);
    void stop_gradient();
    void gradient_timer_callback();

    void lights_on();
    void lights_off();

    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr lights_pub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr mode_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr brightness_sub_;

    rclcpp::TimerBase::SharedPtr blink_timer_;
    rclcpp::TimerBase::SharedPtr gradient_timer_;

    bool is_lights_turn_on_ = false;

    uint8_t brightness_ = 255;
    bool is_blinking_ = false;
    int lights_state_ = 0; // 0 - off, 1 - on, 2 - blink slow, 3 - blink fast, 4 - gradient
    bool blink_state_ = false;
    int blink_period_ms_ = 500;

    bool is_gradient_ = false;
    uint8_t current_brightness_ = 0;
    int gradient_direction_ = 1; // 1 = увеличение, -1 = уменьшение
    int gradient_step_ms_ = 10;  // шаг изменения яркости в мс
};

} // namespace stingray_core::lights_device
