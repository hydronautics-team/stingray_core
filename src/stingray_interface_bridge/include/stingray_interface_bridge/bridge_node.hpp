#pragma once

#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "stingray_core_interfaces/srv/set_stabilization.hpp"
#include "stingray_core_interfaces/srv/set_twist.hpp"

class StingrayInterfaceBridge : public rclcpp::Node
{
public:
  explicit StingrayInterfaceBridge(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void handle_set_twist(
    const std::shared_ptr<stingray_core_interfaces::srv::SetTwist::Request> request,
    std::shared_ptr<stingray_core_interfaces::srv::SetTwist::Response> response);

  void handle_set_stabilization(
    const std::shared_ptr<stingray_core_interfaces::srv::SetStabilization::Request> request,
    std::shared_ptr<stingray_core_interfaces::srv::SetStabilization::Response> response);

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr control_data_publisher_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr loop_flags_publisher_;

  rclcpp::Service<stingray_core_interfaces::srv::SetTwist>::SharedPtr set_twist_service_;
  rclcpp::Service<stingray_core_interfaces::srv::SetStabilization>::SharedPtr set_stabilization_service_;

  geometry_msgs::msg::Twist twist_msg_{};
  std_msgs::msg::UInt8 loop_flags_msg_{};

  bool enable_loop_protection_{false};
  rclcpp::Time last_pub_stamp_{0, RCL_ROS_TIME};

  std::string input_service_;
  std::string input_stabilization_service_;
  std::string output_topic_;
  std::string loop_flags_topic_;
};
