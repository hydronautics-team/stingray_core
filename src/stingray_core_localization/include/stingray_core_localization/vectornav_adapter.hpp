#pragma once

#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/imu.hpp"
#include "vectornav_msgs/msg/common_group.hpp"

namespace stingray_core::localization
{

class VectornavAdapter : public rclcpp::Node
{
public:
    VectornavAdapter();

private:
    void imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr &msg);
    void commonCallback(const vectornav_msgs::msg::CommonGroup::ConstSharedPtr &msg);

    void publishImu();

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<vectornav_msgs::msg::CommonGroup>::SharedPtr common_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

    sensor_msgs::msg::Imu latest_imu_;

    double yaw_;
    double pitch_;
    double roll_;

    bool have_imu_;
    bool have_ypr_;

    std::string frame_id_;
};

} // namespace stingray_core::localization
