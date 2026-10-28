#pragma once

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/bool.hpp>

namespace stingray_core::localization
{

class ImuAdapter : public rclcpp::Node
{
public:
    ImuAdapter();

private:
    void imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr &msg);
    void zeroYawCallback(const std_msgs::msg::Bool::ConstSharedPtr &msg);

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr zero_yaw_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

    std::string input_topic_;
    std::string output_topic_;
    std::string frame_id_;
    double current_yaw_{0.0};
    double yaw_offset_{0.0};
    bool have_orientation_{false};
};

} // namespace stingray_core::localization
