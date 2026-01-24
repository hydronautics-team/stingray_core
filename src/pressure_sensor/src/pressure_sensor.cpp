#include <pressure_sensor/pressure_sensor.hpp>
#include <rclcpp/rclcpp.hpp>

namespace stingray_core::pressure_sensor {

PressureSensor::PressureSensor(rclcpp::NodeOptions options)
    : node_(rclcpp::Node::make_shared("pressure_sensor",
                                      std::move(options))),
      config_(node_),
      depth_pub_(node_->create_publisher<std_msgs::msg::Float64>("depth", rclcpp::SensorDataQoS())),
      data_raw_sub_(node_->create_subscription<std_msgs::msg::String>(
          "/stingray_core/depth_link_node/data_raw", rclcpp::SensorDataQoS(),
          [this](const std_msgs::msg::String::SharedPtr& msg) {
              this->data_raw_callback(msg)
          })) {

    RCLCPP_INFO(node_->get_logger(), "Pressure sensor node initialized");
    RCLCPP_INFO(node_->get_logger(), "dump_param: %.3f", config_.dump_param);
}

void PressureSensor::data_raw_callback(
    const std_msgs::msg::String::ConstSharedPtr& msg) {
    const double depth = std::stod(msg->data) * config_.dump_param;
    publish_depth(depth);
}

void PressureSensor::publish_depth(const double depth) {
    auto depth_msg = std_msgs::msg::Float64::build().data(depth);
    RCLCPP_DEBUG(node_->get_logger(), "Published depth: %.3f m", depth);
    depth_pub_->publish(std::move(depth_msg));
}

}  // namespace stingray_core::pressure_sensor