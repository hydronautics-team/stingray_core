#include "lights_device/lights_device.hpp"
#include "algorithm"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"

namespace stingray_core::lights_device
{

LightsControl::LightsControl(rclcpp::NodeOptions options)
    : node_(rclcpp::Node::make_shared("lights_device", std::move(options))),
      lights_pub_(node_->create_publisher<std_msgs::msg::UInt8MultiArray>("/lights/cmd", 10)),
      mode_sub_(node_->create_subscription<std_msgs::msg::Int32>(
          "/lights/mode", 10, [this](const std_msgs::msg::Int32::ConstSharedPtr &msg) { this->mode_callback(msg); })),
      brightness_sub_(node_->create_subscription<std_msgs::msg::Int32>(
          "/lights/brightness", 10, [this](const std_msgs::msg::Int32::ConstSharedPtr &msg) { this->brightness_callback(msg); }))
{
    RCLCPP_INFO(node_->get_logger(), "Lights device node initialized");
}

void LightsControl::mode_callback(const std_msgs::msg::Int32::ConstSharedPtr &msg)
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
        start_blinking(1000);
        break;
    case 3: // BLINK FAST (5 Hz)
        start_blinking(200);
        break;
    case 4:
        start_gradient(10); // меняем яркость каждые 10 мс
        break;
    case 5:
        start_gradient(30); // меняем яркость каждые 30 мс
        break;
    default:
        RCLCPP_WARN(node_->get_logger(), "Unknown command! %d", msg->data);
    }
}

void LightsControl::brightness_callback(const std_msgs::msg::Int32::ConstSharedPtr &msg)
{
    brightness_ = static_cast<uint8_t>(std::max(0, std::min(255, msg->data)));
}

void LightsControl::start_blinking(int period_ms)
{
    if (!is_blinking_) {
        stop_gradient();
        stop_blinking();

        is_blinking_ = true;
        blink_period_ms_ = period_ms;
        blink_state_ = true;

        blink_timer_ = node_->create_wall_timer(std::chrono::milliseconds(period_ms / 2), [this]() { this->blink_timer_callback(); });

        publish_cmd(brightness_);
        RCLCPP_INFO(node_->get_logger(), "Started blinking with period %d ms, brightness %d", blink_period_ms_, brightness_);
    }
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
        publish_cmd(brightness_);
    }
    else
    {
        publish_cmd(0);
    }
}

void LightsControl::start_gradient(int step_ms)
{
    if (!is_gradient_)
    {
        stop_blinking();
        stop_gradient();

        is_gradient_ = true;
        gradient_step_ms_ = step_ms;
        current_brightness_ = 0;
        gradient_direction_ = 1;

        publish_cmd(current_brightness_);

        gradient_timer_ =
            node_->create_wall_timer(std::chrono::milliseconds(gradient_step_ms_), [this]() { this->gradient_timer_callback(); });

        RCLCPP_INFO(node_->get_logger(), "Started gradient with step %d ms", step_ms);
    }
}

void LightsControl::stop_gradient()
{
    if (gradient_timer_)
    {
        gradient_timer_->cancel();
        gradient_timer_.reset();
    }
    is_gradient_ = false;
}

void LightsControl::gradient_timer_callback()
{
    if (!is_gradient_)
        return;

    current_brightness_ += gradient_direction_;

    if (current_brightness_ >= brightness_)
    {
        current_brightness_ = brightness_;
        gradient_direction_ = -1;
    }
    else if (current_brightness_ <= 0)
    {
        current_brightness_ = 0;
        gradient_direction_ = 1;
    }

    publish_cmd(current_brightness_);
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
    is_lights_turn_on_ = false;
    stop_gradient();
    stop_blinking();
    publish_cmd(0);
}

void LightsControl::lights_on()
{
    is_lights_turn_on_ = true;
    stop_gradient();
    stop_blinking();
    publish_cmd(brightness_);
}

} // namespace stingray_core::lights_device
