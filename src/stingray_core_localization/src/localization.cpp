#include "stingray_core_localization/localization.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>

namespace stingray_core::localization
{

namespace
{
constexpr double kDegreesToRadians = 3.14159265358979323846 / 180.0;
}

LocalizationConfig::LocalizationConfig(rclcpp::Node *node)
{
    imu_orientation_topic =
        node->declare_parameter<std::string>("imu_orientation_topic", "/vectornav/raw/common");
    imu_topic = node->declare_parameter<std::string>("imu_topic", "/vectornav/imu");
    dvl_topic = node->declare_parameter<std::string>("dvl_topic", "/dvl/data");
    pressure_topic = node->declare_parameter<std::string>("pressure_topic",
                                                          "/stingray_core/pressure_sensor/depth");
    zero_yaw_topic = node->declare_parameter<std::string>("zero_yaw_topic", "/imu/zero_yaw");
    odometry_topic = node->declare_parameter<std::string>("odometry_topic", "/core/state/odometry");
    acceleration_topic = node->declare_parameter<std::string>("acceleration_topic",
                                                              "/core/state/linear_acceleration");
    orientation_topic =
        node->declare_parameter<std::string>("orientation_topic", "/core/state/orientation");

    odom_frame = node->declare_parameter<std::string>("odom_frame", "odom");
    base_frame = node->declare_parameter<std::string>("base_frame", "base_link");

    update_rate = node->declare_parameter<double>("update_rate", 100.0);
    use_dvl_velocity = node->declare_parameter<bool>("use_dvl_velocity", false);
    dvl_velocity_alpha = node->declare_parameter<double>("dvl_velocity_alpha", 0.2);
    dvl_timeout_sec = node->declare_parameter<double>("dvl_timeout_sec", 0.5);
}

Localization::Localization(const rclcpp::NodeOptions &options)
    : Node("stingray_core_localization", options), config_(this)
{
    setup_subscribers();
    setup_publishers();
    setup_timer();

    RCLCPP_INFO(get_logger(), "Localization node started at %.1f Hz", config_.update_rate);
}

void Localization::setup_subscribers()
{
    const auto qos_sensor = rclcpp::SensorDataQoS();

    const auto qos_event = rclcpp::QoS(1).reliable();

    imu_orientation_sub_ = create_subscription<vectornav_msgs::msg::CommonGroup>(
        config_.imu_orientation_topic, qos_sensor,
        std::bind(&Localization::imu_orientation_callback, this, std::placeholders::_1));

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        config_.imu_topic, qos_sensor,
        std::bind(&Localization::imu_callback, this, std::placeholders::_1));

    dvl_sub_ = create_subscription<dvl_msgs::msg::DVL>(
        config_.dvl_topic, qos_sensor,
        std::bind(&Localization::dvl_callback, this, std::placeholders::_1));

    pressure_sub_ = create_subscription<std_msgs::msg::Float64>(
        config_.pressure_topic, qos_sensor,
        std::bind(&Localization::pressure_callback, this, std::placeholders::_1));

    zero_yaw_sub_ = create_subscription<std_msgs::msg::Bool>(
        config_.zero_yaw_topic, qos_event,
        std::bind(&Localization::zero_yaw_callback, this, std::placeholders::_1));
}

void Localization::setup_publishers()
{
    const auto qos_state = rclcpp::QoS(10).reliable();
    odometry_pub_ = create_publisher<nav_msgs::msg::Odometry>(config_.odometry_topic, qos_state);
    acceleration_pub_ =
        create_publisher<geometry_msgs::msg::Vector3>(config_.acceleration_topic, qos_state);
    orientation_pub_ =
        create_publisher<geometry_msgs::msg::Vector3>(config_.orientation_topic, qos_state);
}

void Localization::setup_timer()
{
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / config_.update_rate));

    timer_ = create_wall_timer(period, std::bind(&Localization::update_estimate, this));
}

void Localization::imu_orientation_callback(
    const vectornav_msgs::msg::CommonGroup::ConstSharedPtr &msg)
{
    const auto &ypr = msg->yawpitchroll;

    imu_yaw_raw_ = static_cast<double>(ypr.x);

    yaw_ = normalize_angle_deg(imu_yaw_raw_ - yaw_zero_offset_);

    pitch_ = static_cast<double>(ypr.y);
    roll_ = static_cast<double>(ypr.z);
}

void Localization::imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr &msg)
{
    angular_velocity_x_ = msg->angular_velocity.x;
    angular_velocity_y_ = msg->angular_velocity.y;
    angular_velocity_z_ = msg->angular_velocity.z;

    acceleration_x_ = msg->linear_acceleration.x;
    acceleration_y_ = msg->linear_acceleration.y;
    acceleration_z_ = msg->linear_acceleration.z;
}

void Localization::dvl_callback(const dvl_msgs::msg::DVL::ConstSharedPtr &msg)
{
    dvl_velocity_x_ = static_cast<double>(msg->velocity.x);

    dvl_velocity_y_ = static_cast<double>(msg->velocity.y);

    dvl_velocity_z_ = static_cast<double>(msg->velocity.z);

    dvl_velocity_valid_ = msg->velocity_valid;
    dvl_last_time_ = now();
}

void Localization::pressure_callback(const std_msgs::msg::Float64::ConstSharedPtr &msg)
{
    depth_ = msg->data;

    // ROS convention used by this package:
    // +Z is upward, while depth is positive downward.
    z_ = -depth_;
}

void Localization::zero_yaw_callback(const std_msgs::msg::Bool::ConstSharedPtr &msg)
{
    if (!msg->data)
    {
        return;
    }

    yaw_zero_offset_ = imu_yaw_raw_;

    RCLCPP_INFO(get_logger(), "Yaw zeroed at %.2f deg", yaw_zero_offset_);
}

bool Localization::is_dvl_fresh() const
{
    if (dvl_last_time_.nanoseconds() == 0)
    {
        return false;
    }

    const double age = (now() - dvl_last_time_).seconds();

    return age >= 0.0 && age <= config_.dvl_timeout_sec;
}

double Localization::normalize_angle_deg(double angle_deg)
{
    return angle_deg - 360.0 * std::floor((angle_deg + 180.0) / 360.0);
}

void Localization::update_estimate()
{
    const auto current_time = now();

    double dt = 1.0 / config_.update_rate;

    if (!first_update_)
    {
        dt = (current_time - last_update_time_).seconds();
    }

    last_update_time_ = current_time;
    first_update_ = false;

    // Keep the same safety limits as the previous control node.
    dt = std::clamp(dt, 1e-3, 0.05);

    // Same estimation logic that previously lived in
    // stingray_core_control.
    velocity_imu_x_ += acceleration_x_ * dt;
    velocity_imu_y_ += acceleration_y_ * dt;
    velocity_imu_z_ += acceleration_z_ * dt;

    const bool use_dvl = config_.use_dvl_velocity && dvl_velocity_valid_ && is_dvl_fresh();

    const double alpha = std::clamp(config_.dvl_velocity_alpha, 0.0, 1.0);

    if (use_dvl)
    {
        velocity_x_ = alpha * velocity_imu_x_ + (1.0 - alpha) * dvl_velocity_x_;

        velocity_y_ = alpha * velocity_imu_y_ + (1.0 - alpha) * dvl_velocity_y_;

        velocity_z_ = alpha * velocity_imu_z_ + (1.0 - alpha) * dvl_velocity_z_;
    }
    else
    {
        velocity_x_ = velocity_imu_x_;
        velocity_y_ = velocity_imu_y_;
        velocity_z_ = velocity_imu_z_;
    }

    // Pressure sensor directly determines depth.
    z_ = -depth_;

    publish_odometry();
    publish_acceleration();
    publish_orientation();
}

void Localization::publish_odometry()
{
    nav_msgs::msg::Odometry msg;

    msg.header.stamp = now();
    msg.header.frame_id = config_.odom_frame;
    msg.child_frame_id = config_.base_frame;

    msg.pose.pose.position.x = x_;
    msg.pose.pose.position.y = y_;
    msg.pose.pose.position.z = z_;

    tf2::Quaternion orientation;
    orientation.setRPY(roll_ * kDegreesToRadians, pitch_ * kDegreesToRadians,
                       yaw_ * kDegreesToRadians);

    msg.pose.pose.orientation.x = orientation.x();
    msg.pose.pose.orientation.y = orientation.y();
    msg.pose.pose.orientation.z = orientation.z();
    msg.pose.pose.orientation.w = orientation.w();

    msg.twist.twist.linear.x = velocity_x_;
    msg.twist.twist.linear.y = velocity_y_;
    msg.twist.twist.linear.z = velocity_z_;

    msg.twist.twist.angular.x = angular_velocity_x_;
    msg.twist.twist.angular.y = angular_velocity_y_;
    msg.twist.twist.angular.z = angular_velocity_z_;

    // The current estimator does not estimate x/y position.
    msg.pose.covariance[0] = 999.0;
    msg.pose.covariance[7] = 999.0;
    msg.pose.covariance[14] = 0.05;

    // Roll/pitch/yaw are currently taken directly from VectorNav.
    msg.pose.covariance[21] = 0.01;
    msg.pose.covariance[28] = 0.01;
    msg.pose.covariance[35] = 0.02;

    const bool use_dvl = config_.use_dvl_velocity && dvl_velocity_valid_ && is_dvl_fresh();

    const double velocity_covariance = use_dvl ? 0.01 : 0.1;

    msg.twist.covariance[0] = velocity_covariance;
    msg.twist.covariance[7] = velocity_covariance;
    msg.twist.covariance[14] = velocity_covariance;

    msg.twist.covariance[21] = 0.01;
    msg.twist.covariance[28] = 0.01;
    msg.twist.covariance[35] = 0.01;

    odometry_pub_->publish(msg);
}

void Localization::publish_acceleration()
{
    geometry_msgs::msg::Vector3 msg;

    msg.x = acceleration_x_;
    msg.y = acceleration_y_;
    msg.z = acceleration_z_;

    acceleration_pub_->publish(msg);
}

void Localization::publish_orientation()
{
    geometry_msgs::msg::Vector3 msg;

    // VectorNav yawpitchroll:
    // x = yaw
    // y = pitch
    // z = roll
    //
    // Public localization state:
    // x = roll
    // y = pitch
    // z = yaw
    //
    // All values are in degrees.

    msg.x = roll_;
    msg.y = pitch_;
    msg.z = yaw_;

    orientation_pub_->publish(msg);
}

} // namespace stingray_core::localization
