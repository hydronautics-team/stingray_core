#pragma once

#include <geometry_msgs/msg/point_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

#include <string>

#define DEFAULT_DUMP_PARAM 1.0

namespace stingray_core::sensors {

struct PressureSensorConfig {
    PressureSensorConfig(const rclcpp::Node::SharedPtr& node)
        : dump_param(
              node->declare_parameter<double>(
                  "dump_param",
                  DEFAULT_DUMP_PARAM)),
          data_topic(
              node->declare_parameter<std::string>(
                  "data_topic",
                  "/data_raw")),
          frame_id(
              node->declare_parameter<std::string>(
                  "frame_id",
                  "pressure_link"))
    {
    }

    const double dump_param;
    const std::string data_topic;
    const std::string frame_id;
};

class PressureSensor {
public:
    explicit PressureSensor(
        rclcpp::NodeOptions options = rclcpp::NodeOptions());

    void spin()
    {
        rclcpp::spin(node_);
    }

    rclcpp::Logger get_logger() const
    {
        return node_->get_logger();
    }

private:
    void data_raw_callback(
        const std_msgs::msg::Float64::ConstSharedPtr& msg);

    void publish_depth(
        double depth,
        const rclcpp::Time& stamp);

    rclcpp::Node::SharedPtr node_;
    PressureSensorConfig config_;

    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr depth_pub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr data_raw_sub_;
};

}  // namespace stingray_core::sensors
