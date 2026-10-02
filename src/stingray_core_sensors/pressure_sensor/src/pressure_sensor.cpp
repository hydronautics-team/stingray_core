#include <diagnostic_updater/diagnostic_updater.hpp>
#include <exception>
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
              })),
      diagnostic_updater_(node_)
{
    diagnostic_updater_.setHardwareID("pressure_sensor");
    diagnostic_updater_.add("Pressure Sensor Diagnostics", this,
                            &PressureSensor::update_diagnostics);

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

    last_msg_time_ = node_->now();
    current_depth = depth;
    has_received_msg_ = true;
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

void PressureSensor::update_diagnostics(
    diagnostic_updater::DiagnosticStatusWrapper& stat)
{
    if (!has_received_msg_)
        stat.summary(
            diagnostic_msgs::msg::DiagnosticStatus::WARN,
            "Waiting for first message");
    else if ((node_->now() - last_msg_time_).seconds() > 1.0)
        stat.summary(
            diagnostic_msgs::msg::DiagnosticStatus::ERROR,
            "Sensor timeout");
    else
        stat.summary(
            diagnostic_msgs::msg::DiagnosticStatus::OK,
            "Sensor OK");

    stat.add("Current depth", current_depth);
    stat.add("Errors count", error_msgs_count);

    if (has_received_msg_)
        stat.add(
            "Time since last message",
            (node_->now() - last_msg_time_).seconds());
}

}  // namespace stingray_core::sensors