#pragma once

#include "rclcpp/rclcpp.hpp"

#include "chrono"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace stingray_core::localization
{

struct LocalizationConfig
{
    LocalizationConfig(const rclcpp::Node::SharedPtr &node)
        : imu_topic(node->declare_parameter<std::string>("imu_topic", "/core/sensors/imu")),
          dvl_topic(node->declare_parameter<std::string>("dvl_topic", "/core/sensors/dvl")),
          ps_topic(node->declare_parameter<std::string>("ps_topic", "/core/sensors/heave")),
          odometry_topic(
              node->declare_parameter<std::string>("odometry_topic", "/core/state/odometry")),
          odom_frame(node->declare_parameter<std::string>("odom_frame", "odom")),
          base_frame(node->declare_parameter<std::string>("base_frame", "base_link")),
          update_rate(node->declare_parameter<double>("update_rate", 50.0))
    {
    }
    std::string imu_topic;
    std::string dvl_topic;
    std::string ps_topic;
    std::string odometry_topic;
    std::string odom_frame;
    std::string base_frame;
    double update_rate;
};

class Localization
{
public:
    explicit Localization(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    void spin() { rclcpp::spin(node_); }

private:
    void setup_subscribers();
    void setup_publishers();
    void setup_timer();

    void imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr &msg);
    void dvl_callback(const nav_msgs::msg::Odometry::ConstSharedPtr &msg);
    void ps_callback();

    void update_estimate();

    void publish_odometry();

    rclcpp::Logger get_logger() const { return node_->get_logger(); }

    rclcpp::Node::SharedPtr node_;

    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr dvl_sub_;

    // Publishers
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    // Configuration
    LocalizationConfig config_;
};
} // namespace stingray_core::localization
