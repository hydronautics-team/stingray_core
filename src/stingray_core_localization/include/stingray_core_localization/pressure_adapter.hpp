#pragma once

#include <string>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

namespace stingray_core::localization
{

class PressureAdapter : public rclcpp::Node
{
public:
    PressureAdapter();

private:
    void pressureCallback(const geometry_msgs::msg::PointStamped::ConstSharedPtr &msg);
    void ms5837Callback(const std_msgs::msg::Float32::ConstSharedPtr &msg);
    void publishDepth(double depth, const rclcpp::Time &stamp);

    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr pressure_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr ms5837_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr measurement_pub_;

    double depth_variance_;
    double depth_scale_;
    std::string output_frame_;
    std::string input_topic_;
    std::string input_type_;
};

} // namespace stingray_core::localization
