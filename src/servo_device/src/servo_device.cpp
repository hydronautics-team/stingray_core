#include <servo_device/servo_device.hpp>
#include <rclcpp/rclcpp.hpp>

namespace stingray_core::servo_device {

ServoDevice::ServoDevice(rclcpp::NodeOptions options)
    : node_(rclcpp::Node::make_shared("servo_device",
                                      std::move(options))),
      servo_control_pub_(node_->create_publisher<std_msgs::msg::UInt8>("cmd", rclcpp::SensorDataQoS())),
      data_raw_sub_(node_->create_subscription<std_msgs::msg::UInt8>(
          "state", rclcpp::SensorDataQoS(),
          [this](const std_msgs::msg::UInt8::ConstSharedPtr msg) {
              this->data_raw_callback(msg);
          })) {

    RCLCPP_INFO(node_->get_logger(), "Servo device node is initialized");
}

void ServoDevice::data_raw_callback(
    const std_msgs::msg::UInt8::ConstSharedPtr& msg) {
    publish_servo_control(static_cast<uint8_t>(msg->data));
}

void ServoDevice::publish_servo_control(const uint8_t control_signal) {
    auto control_signal_msg = std_msgs::msg::UInt8();
    control_signal_msg.data = control_signal;
    RCLCPP_DEBUG(node_->get_logger(), "Published control_signal: %d", static_cast<int>(control_signal));
    servo_control_pub_->publish(std::move(control_signal_msg));
}

}  // namespace stingray_core::servo_device
