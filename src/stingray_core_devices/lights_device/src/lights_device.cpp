#include "lights_device/lights_device.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"

namespace stingray_core::lights_device
{

LightsControl::LightsControl(rclcpp::NodeOptions options)
    : node_(rclcpp::Node::make_shared("lights_device", std::move(options))),
      lights_pub_(node_->create_publisher<std_msgs::msg::UInt8MultiArray>("/lights/brightness", rclcpp::SensorDataQoS())),
      mode_sub_(node_->create_subscription<std_msgs::msg::String>("/lights/set_mode", rclcpp::SensorDataQoS(),
                                                                  [this](const std_msgs::msg::String::ConstSharedPtr &msg)
                                                                  { this->data_raw_callback(msg); }))
{
    RCLCPP_INFO(node_->get_logger(), "Lights device node initialized");
}

void LightsControl::data_raw_callback(const std_msgs::msg::String::ConstSharedPtr &msg)
{
    if (msg->data == "ON")
    {
        publish_cmd(255);
    }
    else if (msg->data == "OFF")
    {
        publish_cmd(0);
    }
    else
    {
        RCLCPP_WARN(node_->get_logger(), "Unknown command! %s", msg->data.c_str());
    }
}

void LightsControl::publish_cmd(uint8_t command)
{
    auto command_msg = std_msgs::msg::UInt8MultiArray();
    command_msg.data = {command, command};
    RCLCPP_DEBUG(node_->get_logger(), "Published command lights: %u", command);
    lights_pub_->publish(command_msg);
}

} // namespace stingray_core::lights_device
