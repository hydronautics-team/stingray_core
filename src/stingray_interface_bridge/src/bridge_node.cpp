#include "stingray_interface_bridge/bridge_node.hpp"

#include <cstdint>

#include "rclcpp/qos.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

StingrayInterfaceBridge::StingrayInterfaceBridge(const rclcpp::NodeOptions & options)
: rclcpp::Node("stingray_interface_bridge", options)
{
  // Declare parameters
  this->declare_parameter<std::string>("input_service", "/set_twist");
  this->declare_parameter<std::string>("input_stabilization_service", "/set_stabilization");
  this->declare_parameter<std::string>("output_topic", "/control/data");
  this->declare_parameter<std::string>("loop_flags_topic", "/control/loop_flags");
  this->declare_parameter<std::string>("uv_state_topic", "/stingray/topics/uv_state");
  this->declare_parameter<std::string>("imu_angular_topic", "/vectornav/raw/common");
  this->declare_parameter<std::string>("imu_linear_accel_topic", "/vectornav/imu_accel");
  this->declare_parameter<std::string>("depth_topic", "/sensors/pressure");
  this->declare_parameter<int>("qos_depth", 1);
  this->declare_parameter<std::string>("qos_reliability", "reliable"); // reliable | best_effort
  this->declare_parameter<bool>("enable_loop_protection", false);

  input_service_ = this->get_parameter("input_service").as_string();
  input_stabilization_service_ = this->get_parameter("input_stabilization_service").as_string();
  output_topic_ = this->get_parameter("output_topic").as_string();
  loop_flags_topic_ = this->get_parameter("loop_flags_topic").as_string();
  uv_state_topic_ = this->get_parameter("uv_state_topic").as_string();
  imu_angular_topic_ = this->get_parameter("imu_angular_topic").as_string();
  imu_linear_accel_topic_ = this->get_parameter("imu_linear_accel_topic").as_string();
  depth_topic_ = this->get_parameter("depth_topic").as_string();
  int qos_depth = this->get_parameter("qos_depth").as_int();
  std::string qos_reliability = this->get_parameter("qos_reliability").as_string();
  enable_loop_protection_ = this->get_parameter("enable_loop_protection").as_bool();

  // Build QoS
  auto pub_qos = rclcpp::QoS(rclcpp::KeepLast(qos_depth));
  pub_qos.durability_volatile();
  if (qos_reliability == "best_effort") {
    pub_qos.best_effort();
  } else {
    pub_qos.reliable();
  }

  // Preallocate twist message to avoid dynamic allocations in hot path
  twist_msg_.linear.x = 0.0;
  twist_msg_.linear.y = 0.0;
  twist_msg_.linear.z = 0.0;
  twist_msg_.angular.x = 0.0;
  twist_msg_.angular.y = 0.0;
  twist_msg_.angular.z = 0.0;
  loop_flags_msg_.data = 0u;

  control_data_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(output_topic_, pub_qos);
  loop_flags_publisher_ = this->create_publisher<std_msgs::msg::UInt8>(loop_flags_topic_, pub_qos);
  uv_state_publisher_ = this->create_publisher<stingray_core_interfaces::msg::UVState>(uv_state_topic_, pub_qos);

  imu_angular_sub_ = this->create_subscription<vectornav_msgs::msg::CommonGroup>(
    imu_angular_topic_,
    pub_qos,
    std::bind(&StingrayInterfaceBridge::handle_imu_angular, this, _1));

  imu_linear_accel_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
    imu_linear_accel_topic_,
    pub_qos,
    std::bind(&StingrayInterfaceBridge::handle_imu_linear_accel, this, _1));

  depth_sub_ = this->create_subscription<std_msgs::msg::Float32>(
    depth_topic_,
    pub_qos,
    std::bind(&StingrayInterfaceBridge::handle_depth, this, _1));

  // Service callback: map srv/SetTwist -> geometry_msgs/Twist publication
  set_twist_service_ = this->create_service<stingray_core_interfaces::srv::SetTwist>(
    input_service_,
    std::bind(&StingrayInterfaceBridge::handle_set_twist, this, _1, _2)
  );

  // Service callback: map srv/SetStabilization -> std_msgs/UInt8 loop flags publication
  set_stabilization_service_ = this->create_service<stingray_core_interfaces::srv::SetStabilization>(
    input_stabilization_service_,
    std::bind(&StingrayInterfaceBridge::handle_set_stabilization, this, _1, _2)
  );

  RCLCPP_INFO(
    this->get_logger(),
    "Started StingrayInterfaceBridge: twist_service='%s' -> '%s', stabilization_service='%s' -> '%s', uv_state='%s'",
    input_service_.c_str(),
    output_topic_.c_str(),
    input_stabilization_service_.c_str(),
    loop_flags_topic_.c_str(),
    uv_state_topic_.c_str());
}

void StingrayInterfaceBridge::handle_set_twist(
  const std::shared_ptr<stingray_core_interfaces::srv::SetTwist::Request> request,
  std::shared_ptr<stingray_core_interfaces::srv::SetTwist::Response> response)
{
  // Single-threaded executor assumed: no mutex required.
  twist_msg_.linear.x = static_cast<double>(request->surge);
  twist_msg_.linear.y = static_cast<double>(request->sway);
  twist_msg_.linear.z = static_cast<double>(request->depth);
  twist_msg_.angular.x = static_cast<double>(request->roll);
  twist_msg_.angular.y = static_cast<double>(request->pitch);
  twist_msg_.angular.z = static_cast<double>(request->yaw);

  // Optionally track last publication time for loop protection
  if (enable_loop_protection_) {
    last_pub_stamp_ = this->now();
  }

  // Publish the message. Publishing is thread-safe in rclcpp.
  control_data_publisher_->publish(twist_msg_);

  // Fill response
  response->success = true;
  response->message = "ok";
}

void StingrayInterfaceBridge::handle_set_stabilization(
  const std::shared_ptr<stingray_core_interfaces::srv::SetStabilization::Request> request,
  std::shared_ptr<stingray_core_interfaces::srv::SetStabilization::Response> response)
{
  // bit0 surge, bit1 sway, bit2 heave(depth), bit3 yaw, bit4 pitch, bit5 roll
  uint8_t flags = 0u;

  if (request->depth_stabilization) {
    flags |= static_cast<uint8_t>(1u << 2);
  }
  if (request->yaw_stabilization) {
    flags |= static_cast<uint8_t>(1u << 3);
  }
  if (request->pitch_stabilization) {
    flags |= static_cast<uint8_t>(1u << 4);
  }
  if (request->roll_stabilization) {
    flags |= static_cast<uint8_t>(1u << 5);
  }

  loop_flags_msg_.data = flags;
  uv_state_msg_.depth_stabilization = request->depth_stabilization;
  uv_state_msg_.yaw_stabilization = request->yaw_stabilization;
  uv_state_msg_.pitch_stabilization = request->pitch_stabilization;
  uv_state_msg_.roll_stabilization = request->roll_stabilization;

  if (enable_loop_protection_) {
    last_pub_stamp_ = this->now();
  }

  loop_flags_publisher_->publish(loop_flags_msg_);
  publish_uv_state();

  response->success = true;
  response->message = "ok";
}

void StingrayInterfaceBridge::handle_imu_angular(const vectornav_msgs::msg::CommonGroup::SharedPtr msg)
{
  uv_state_msg_.yaw = static_cast<float>(msg->yawpitchroll.x);
  uv_state_msg_.pitch = static_cast<float>(msg->yawpitchroll.y);
  uv_state_msg_.roll = static_cast<float>(msg->yawpitchroll.z);
  publish_uv_state();
}

void StingrayInterfaceBridge::handle_imu_linear_accel(const geometry_msgs::msg::Vector3::SharedPtr msg)
{
  uv_state_msg_.surge_accel = static_cast<float>(msg->x);
  uv_state_msg_.sway_accel = static_cast<float>(msg->y);
  publish_uv_state();
}

void StingrayInterfaceBridge::handle_depth(const std_msgs::msg::Float32::SharedPtr msg)
{
  uv_state_msg_.depth = static_cast<float>(msg->data);
  publish_uv_state();
}

void StingrayInterfaceBridge::publish_uv_state()
{
  uv_state_publisher_->publish(uv_state_msg_);
}
