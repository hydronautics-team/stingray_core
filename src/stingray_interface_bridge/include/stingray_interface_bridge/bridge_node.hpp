#pragma once

#include <memory>
#include <string>

#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "stingray_core_interfaces/msg/uv_state.hpp"
#include "stingray_core_interfaces/srv/set_stabilization.hpp"
#include "stingray_core_interfaces/srv/set_twist.hpp"
#include "vectornav_msgs/msg/common_group.hpp"

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

  void handle_imu_angular(const vectornav_msgs::msg::CommonGroup::SharedPtr msg);
  void handle_imu_linear_accel(const geometry_msgs::msg::Vector3::SharedPtr msg);
  void handle_depth(const std_msgs::msg::Float32::SharedPtr msg);
  void publish_uv_state();

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr control_data_publisher_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr loop_flags_publisher_;
  rclcpp::Publisher<stingray_core_interfaces::msg::UVState>::SharedPtr uv_state_publisher_;

  rclcpp::Service<stingray_core_interfaces::srv::SetTwist>::SharedPtr set_twist_service_;
  rclcpp::Service<stingray_core_interfaces::srv::SetStabilization>::SharedPtr set_stabilization_service_;

  rclcpp::Subscription<vectornav_msgs::msg::CommonGroup>::SharedPtr imu_angular_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr imu_linear_accel_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr depth_sub_;

  geometry_msgs::msg::Twist twist_msg_{};
  std_msgs::msg::UInt8 loop_flags_msg_{};
  stingray_core_interfaces::msg::UVState uv_state_msg_{};

  bool enable_loop_protection_{false};
  rclcpp::Time last_pub_stamp_{0, 0, RCL_ROS_TIME};

  std::string input_service_;
  std::string input_stabilization_service_;
  std::string output_topic_;
  std::string loop_flags_topic_;
  std::string uv_state_topic_;
  std::string imu_angular_topic_;
  std::string imu_linear_accel_topic_;
  std::string depth_topic_;
};
