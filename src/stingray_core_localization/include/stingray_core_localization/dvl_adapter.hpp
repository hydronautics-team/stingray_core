#pragma once

#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "dvl_msgs/msg/dvl.hpp"
#include "nav_msgs/msg/odometry.hpp"

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

    std::string output_frame_;
};

} // namespace stingray_core::localization
