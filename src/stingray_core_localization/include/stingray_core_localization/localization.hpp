#pragma once

#include <memory>
#include <string>

#include <dvl_msgs/msg/dvl.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <vectornav_msgs/msg/common_group.hpp>

namespace stingray_core::localization
{

struct LocalizationConfig
{
    explicit LocalizationConfig(rclcpp::Node *node);

    std::string imu_orientation_topic;
    std::string imu_topic;
    std::string dvl_topic;
    std::string pressure_topic;
    std::string zero_yaw_topic;

    std::string odometry_topic;
    std::string acceleration_topic;
    std::string orientation_topic;

    std::string odom_frame;
    std::string base_frame;

    double update_rate;
    bool use_dvl_velocity;
    double dvl_velocity_alpha;
    double dvl_timeout_sec;
};

class Localization final : public rclcpp::Node
{
public:
    explicit Localization(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
    void setup_subscribers();
    void setup_publishers();
    void setup_timer();

    void imu_orientation_callback(const vectornav_msgs::msg::CommonGroup::ConstSharedPtr &msg);

    void imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr &msg);

    void dvl_callback(const dvl_msgs::msg::DVL::ConstSharedPtr &msg);

    void pressure_callback(const std_msgs::msg::Float64::ConstSharedPtr &msg);

    void zero_yaw_callback(const std_msgs::msg::Bool::ConstSharedPtr &msg);

    void update_estimate();
    void publish_odometry();
    void publish_acceleration();
    void publish_orientation();

    [[nodiscard]] bool is_dvl_fresh() const;
    [[nodiscard]] static double normalize_angle_deg(double angle_deg);

    LocalizationConfig config_;

    rclcpp::Subscription<vectornav_msgs::msg::CommonGroup>::SharedPtr imu_orientation_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<dvl_msgs::msg::DVL>::SharedPtr dvl_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr pressure_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr zero_yaw_sub_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr acceleration_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr orientation_pub_;

    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Time last_update_time_;
    rclcpp::Time dvl_last_time_{0LL, RCL_ROS_TIME};
    bool first_update_{true};

    // Position/depth state.
    double x_{0.0};
    double y_{0.0};
    double z_{0.0};
    double depth_{0.0};

    // Orientation in degrees, matching the existing control node.
    double roll_{0.0};
    double pitch_{0.0};
    double yaw_{0.0};

    double imu_yaw_raw_{0.0};
    double yaw_zero_offset_{0.0};

    // IMU angular velocity.
    double angular_velocity_x_{0.0};
    double angular_velocity_y_{0.0};
    double angular_velocity_z_{0.0};

    // IMU linear acceleration.
    double acceleration_x_{0.0};
    double acceleration_y_{0.0};
    double acceleration_z_{0.0};

    // Velocity estimated by integrating IMU acceleration.
    double velocity_imu_x_{0.0};
    double velocity_imu_y_{0.0};
    double velocity_imu_z_{0.0};

    // DVL velocity.
    double dvl_velocity_x_{0.0};
    double dvl_velocity_y_{0.0};
    double dvl_velocity_z_{0.0};
    bool dvl_velocity_valid_{false};

    // Final velocity estimate published in odometry.
    double velocity_x_{0.0};
    double velocity_y_{0.0};
    double velocity_z_{0.0};
};

} // namespace stingray_core::localization
