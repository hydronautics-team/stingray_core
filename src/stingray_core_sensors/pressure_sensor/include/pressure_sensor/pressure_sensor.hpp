#pragma once

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>

#define DEFAULT_DUMP_PARAM 1.0

namespace stingray_core::pressure_sensor
{

struct PressureSensorConfig
{
    PressureSensorConfig(const rclcpp::Node::SharedPtr &node)
        : dump_param(node->declare_parameter<double>("dump_param", DEFAULT_DUMP_PARAM)),
          data_topic(node->declare_parameter<std::string>("data_topic", "/data_raw"))
    {
    }
    const double dump_param;
    const std::string data_topic;
};

class PressureSensor
{
public:
    explicit PressureSensor(rclcpp::NodeOptions options = rclcpp::NodeOptions());

    void spin() { rclcpp::spin(node_); }

    rclcpp::Logger get_logger() const { return node_->get_logger(); }

private:
    void data_raw_callback(const std_msgs::msg::String::ConstSharedPtr &msg);
    void publish_depth(double depth);
    void update_diagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);

    rclcpp::Node::SharedPtr node_;
    PressureSensorConfig config_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr depth_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr data_raw_sub_;

    diagnostic_updater::Updater diagnostic_updater_;
    rclcpp::Time last_msg_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    double current_depth = 0.0;
    uint32_t error_msgs_count = 0;
    bool has_received_msg_ = false;
};

} // namespace stingray_core::pressure_sensor
