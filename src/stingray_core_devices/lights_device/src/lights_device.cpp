#include "lights_device/lights_device.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"

namespace stingray_core::lights_device
{

LightsControl::LightsControl(rclcpp::NodeOptions options)
    : node_(rclcpp::Node::make_shared("lights_device", std::move(options))),
      lights_pub_(node_->create_publisher<std_msgs::msg::UInt8MultiArray>("/lights/brightness", 10)),
      mode_sub_(node_->create_subscription<std_msgs::msg::Int32>(
          "/lights/mode", 10, [this](const std_msgs::msg::Int32::ConstSharedPtr &msg) { this->data_raw_callback(msg); }))
{
    RCLCPP_INFO(node_->get_logger(), "Lights device node initialized");
}

void LightsControl::data_raw_callback(const std_msgs::msg::Int32::ConstSharedPtr &msg)
{
    switch (msg->data)
    {
    case 0: // OFF
        lights_off();
        break;
    case 1: // ON
        lights_on();
        break;
    case 2: // BLINK SLOW (1 Hz)
        start_blinking(1000, 255);
        break;
    case 3: // BLINK FAST (5 Hz)
        start_blinking(200, 255);
        break;
    default:
        RCLCPP_WARN(node_->get_logger(), "Unknown command! %d", msg->data);
    }
}

void LightsControl::start_blinking(int period_ms, uint8_t brightness)
{
    stop_blinking();
    is_blinking_ = true;
    blink_brightness_ = brightness;
    blink_period_ms_ = period_ms;
    blink_state_ = true;

    blink_timer_ = node_->create_wall_timer(std::chrono::milliseconds(period_ms / 2), // пол периода для каждого состояния
                                            [this]() { this->blink_timer_callback(); });

    publish_cmd(blink_brightness_);
    RCLCPP_INFO(node_->get_logger(), "Started blinking with period %d ms, brightness %d", period_ms, brightness);
}

void LightsControl::stop_blinking()
{
    if (blink_timer_)
    {
        blink_timer_->cancel();
        blink_timer_.reset();
    }
    is_blinking_ = false;
    blink_state_ = false;
}

void LightsControl::blink_timer_callback()
{
    if (!is_blinking_)
        return;

    blink_state_ = !blink_state_;
    if (blink_state_)
    {
        publish_cmd(blink_brightness_);
    }
    else
    {
        publish_cmd(0);
    }
}

void LightsControl::publish_cmd(uint8_t command)
{
    auto command_msg = std_msgs::msg::UInt8MultiArray();
    command_msg.data = {command, command};
    RCLCPP_DEBUG(node_->get_logger(), "Published command lights: %u", command);
    lights_pub_->publish(command_msg);
}

void LightsControl::lights_off()
{
    stop_blinking();
    publish_cmd(0);
}

void LightsControl::lights_on()
{
    stop_blinking();
    publish_cmd(255);
}

} // namespace stingray_core::lights_device
