#include "stingray_core_localization/localization.hpp"

#include "rclcpp/rclcpp.hpp"

namespace stingray_core::localization
{

Localization::Localization(const rclcpp::NodeOptions &options)
    : node_(rclcpp::Node::make_shared("localization", options)), config_(node_)
{
    RCLCPP_INFO(get_logger(), "Localization node started");

    RCLCPP_INFO(get_logger(), "IMU topic: %s", config_.imu_topic.c_str());
    RCLCPP_INFO(get_logger(), "DVL topic: %s", config_.dvl_topic.c_str());
    RCLCPP_INFO(get_logger(), "Odometry topic: %s", config_.odometry_topic.c_str());
    RCLCPP_INFO(get_logger(), "Update rate: %.1f Hz", config_.update_rate);

    setup_subscribers();
    setup_publishers();
    setup_timer();
}

void Localization::setup_subscribers()
{
    imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
        config_.imu_topic, rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::Imu::ConstSharedPtr &msg) { this->imu_callback(msg); });

    dvl_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        config_.dvl_topic, rclcpp::SensorDataQoS(),
        [this](const nav_msgs::msg::Odometry::ConstSharedPtr &msg) { this->dvl_callback(msg); });
}

void Localization::setup_publishers()
{
    odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>(config_.odometry_topic, 10);
}

void Localization::setup_timer()
{
    auto timer_callback = [this]() { this->update_estimate(); };
    timer_ = node_->create_wall_timer(std::chrono::duration<double>(1.0 / config_.update_rate),
                                      timer_callback);
}

void Localization::imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr &msg) {}

void Localization::dvl_callback(const nav_msgs::msg::Odometry::ConstSharedPtr &msg) {}

void Localization::ps_callback() {}

void Localization::update_estimate() {}

void Localization::publish_odometry()
{
    // Create an odometry message
    auto odom_msg = nav_msgs::msg::Odometry();

    odom_msg.header.stamp = node_->now();
    odom_msg.header.frame_id = config_.odom_frame;
    odom_msg.child_frame_id = config_.base_frame;

    // Publish the odometry message
    odom_pub_->publish(odom_msg);
}

} // namespace stingray_core::localization
