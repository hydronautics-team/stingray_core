#include "buttons_sensor/buttons_sensor.hpp"

namespace stingray_core::buttons_sensor {

ButtonsSensor::ButtonsSensor(const rclcpp::NodeOptions& options)
    : node_(rclcpp::Node::make_shared("buttons_sensor", std::move(options))),
      config_(node_),
      events_pub_(node_->create_publisher<std_msgs::msg::String>(
          "events", rclcpp::SensorDataQoS())),
      buttons_sub_(node_->create_subscription<std_msgs::msg::Int32>(
          "display/buttons", rclcpp::SensorDataQoS(), [this](
            const std_msgs::msg::Int32::ConstSharedPtr& msg) {
            this->buttons_callback(msg); })) {
    
    RCLCPP_INFO(node_->get_logger(), "Buttons sensor initialized");
}

void ButtonsSensorNode::buttons_callback(const std_msgs::msg::Int32::ConstSharedPtr& msg)
{
    int32_t button_id = msg->data;
    
    RCLCPP_INFO(node_->get_logger(), "Received button press: %d", button_id);
    
    auto it = buttons_map_.find(button_id);
    
    if (it != buttons_map_.end()) {
        std::string action = it->second;
        RCLCPP_INFO(node_->get_logger(), "Key ID %d -> Action: %s", button_id, action.c_str());
        
        int32_t event_id = action_to_event(action);
        
        auto action_msg = std_msgs::msg::String::build().data(action);
        RCLCPP_INFO(node_->get_logger(), "Published event: %s", action.c_str());
        events_pub_->publish(std::move(action_msg));
    } else {
        RCLCPP_WARN(node_->get_logger(), "Unknown key ID: %d", button_id);
    }
}

} // namespace stingray_core::buttons_sensor
