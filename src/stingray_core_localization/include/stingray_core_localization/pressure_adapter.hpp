#pragma once

#include <string>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

namespace stingray_core::localization
{

class PressureAdapter : public rclcpp::Node
{
public:
    PressureAdapter();

private:
    void pressureCallback(const geometry_msgs::msg::PointStamped::ConstSharedPtr &msg);

    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr pressure_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr measurement_pub_;

    double depth_variance_;
    std::string output_frame_;
};

} // namespace stingray_core::localization
