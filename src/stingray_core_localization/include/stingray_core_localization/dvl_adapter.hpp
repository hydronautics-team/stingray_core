#pragma once

#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "dvl_msgs/msg/dvl.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float32.hpp"

namespace stingray_core::localization
{

class DvlAdapter : public rclcpp::Node
{
public:
    DvlAdapter();

private:
    void dvlCallback(const dvl_msgs::msg::DVL::ConstSharedPtr &msg);

    rclcpp::Subscription<dvl_msgs::msg::DVL>::SharedPtr dvl_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr dvl_odom_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr altitude_pub_;

    std::string output_frame_;
    std::string input_topic_;
    std::string odometry_topic_;
    std::string altitude_topic_;
};

} // namespace stingray_core::localization
