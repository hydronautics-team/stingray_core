#include <pressure_sensor/pressure_sensor.hpp>

namespace stingray_core::sensors {

PressureSensor::PressureSensor(rclcpp::NodeOptions options)
    : node_(rclcpp::Node::make_shared(
          "pressure_sensor",
          std::move(options))),
      config_(node_),
      depth_pub_(
          node_->create_publisher<geometry_msgs::msg::PointStamped>(
              "depth",
              rclcpp::SensorDataQoS())),
      data_raw_sub_(
          node_->create_subscription<std_msgs::msg::Float64>(
              config_.data_topic,
              rclcpp::SensorDataQoS(),
              [this](
                  const std_msgs::msg::Float64::ConstSharedPtr& msg) {
                  this->data_raw_callback(msg);
              }))
{
    RCLCPP_INFO(
        node_->get_logger(),
        "Pressure sensor node initialized");

    RCLCPP_INFO(
        node_->get_logger(),
        "dump_param: %.3f",
        config_.dump_param);

    RCLCPP_INFO(
        node_->get_logger(),
        "data_topic: %s",
        config_.data_topic.c_str());

    RCLCPP_INFO(
        node_->get_logger(),
        "frame_id: %s",
        config_.frame_id.c_str());
}

void PressureSensor::data_raw_callback(
    const std_msgs::msg::Float64::ConstSharedPtr& msg)
{
    const double depth =
        msg->data * config_.dump_param / 10000.0;

    publish_depth(
        depth,
        node_->now());
}

void PressureSensor::publish_depth(
    double depth,
    const rclcpp::Time& stamp)
{
    auto depth_msg =
        geometry_msgs::msg::PointStamped();

    depth_msg.header.stamp = stamp;
    depth_msg.header.frame_id = config_.frame_id;

    depth_msg.point.x = 0.0;
    depth_msg.point.y = 0.0;
    depth_msg.point.z = depth;

    RCLCPP_DEBUG(
        node_->get_logger(),
        "Published depth: %.3f m",
        depth);

    depth_pub_->publish(std::move(depth_msg));
}

}  // namespace stingray_core::sensors
