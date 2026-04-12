#include "stingray_interface_bridge/bridge_node.hpp"

#include <cstdint>
#include <chrono>

#include "rclcpp/qos.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

StingrayInterfaceBridge::StingrayInterfaceBridge(const rclcpp::NodeOptions & options)
: rclcpp::Node("stingray_interface_bridge", options)
{
  // Declare parameters
  this->declare_parameter<std::string>("input_service", "/stingray/services/set_twist");
  this->declare_parameter<std::string>("input_stabilization_service", "/stingray/services/set_stabilization");
  this->declare_parameter<std::string>("output_topic", "/control/data");
  this->declare_parameter<std::string>("loop_flags_topic", "/control/loop_flags");
  this->declare_parameter<std::string>("uv_state_topic", "/stingray/topics/uv_state");
  this->declare_parameter<std::string>("reset_imu_service", "/stingray/services/reset_imu");
  this->declare_parameter<std::string>("imu_zero_yaw_topic", "/imu/zero_yaw");
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
  reset_imu_service_name_ = this->get_parameter("reset_imu_service").as_string();
  imu_zero_yaw_topic_ = this->get_parameter("imu_zero_yaw_topic").as_string();
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
  last_twist_request_ = twist_msg_;
  loop_flags_msg_.data = 0u;

  control_data_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(output_topic_, pub_qos);
  loop_flags_publisher_ = this->create_publisher<std_msgs::msg::UInt8>(loop_flags_topic_, pub_qos);
  uv_state_publisher_ = this->create_publisher<stingray_interfaces::msg::UVState>(uv_state_topic_, pub_qos);
  imu_zero_yaw_publisher_ = this->create_publisher<std_msgs::msg::Bool>(imu_zero_yaw_topic_, pub_qos);

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
  set_twist_service_ = this->create_service<stingray_interfaces::srv::SetTwist>(
    input_service_,
    std::bind(&StingrayInterfaceBridge::handle_set_twist, this, _1, _2)
  );

  // Service callback: map srv/SetStabilization -> std_msgs/UInt8 loop flags publication
  set_stabilization_service_ = this->create_service<stingray_interfaces::srv::SetStabilization>(
    input_stabilization_service_,
    std::bind(&StingrayInterfaceBridge::handle_set_stabilization, this, _1, _2)
  );

  // Service callback: map std_srvs/Trigger reset_imu -> std_msgs/Bool publication to /imu/zero_yaw
  reset_imu_service_ = this->create_service<std_srvs::srv::Trigger>(
    reset_imu_service_name_,
    std::bind(&StingrayInterfaceBridge::handle_reset_imu, this, _1, _2)
  );

  // Republish twist at 100 Hz for open-loop axes.
  twist_republish_timer_ = this->create_wall_timer(
    10ms,
    [this]() {
      if (republish_enabled_) {
        publish_filtered_twist();
      }
    });

  RCLCPP_INFO(
    this->get_logger(),
    "Started StingrayInterfaceBridge: twist_service='%s' -> '%s', stabilization_service='%s' -> '%s', "
    "reset_imu_service='%s' -> '%s', uv_state='%s'",
    input_service_.c_str(),
    output_topic_.c_str(),
    input_stabilization_service_.c_str(),
    loop_flags_topic_.c_str(),
    reset_imu_service_name_.c_str(),
    imu_zero_yaw_topic_.c_str(),
    uv_state_topic_.c_str());
}

void StingrayInterfaceBridge::handle_set_twist(
  const std::shared_ptr<stingray_interfaces::srv::SetTwist::Request> request,
  std::shared_ptr<stingray_interfaces::srv::SetTwist::Response> response)
{
  // Single-threaded executor assumed: no mutex required.
  last_twist_request_.linear.x = static_cast<double>(request->surge);
  last_twist_request_.linear.y = static_cast<double>(request->sway);
  last_twist_request_.linear.z = static_cast<double>(request->depth);
  last_twist_request_.angular.x = static_cast<double>(request->roll);
  last_twist_request_.angular.y = static_cast<double>(request->pitch);
  last_twist_request_.angular.z = static_cast<double>(request->yaw);
  has_twist_request_ = true;
  update_republish_state();

  // Optionally track last publication time for loop protection
  if (enable_loop_protection_) {
    last_pub_stamp_ = this->now();
  }

  // Publish once immediately; periodic timer is enabled only for non-zero open-loop commands.
  publish_filtered_twist();

  // Fill response
  response->success = true;
  response->message = "ok";
}

void StingrayInterfaceBridge::handle_set_stabilization(
  const std::shared_ptr<stingray_interfaces::srv::SetStabilization::Request> request,
  std::shared_ptr<stingray_interfaces::srv::SetStabilization::Response> response)
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

  update_republish_state();
  loop_flags_publisher_->publish(loop_flags_msg_);
  publish_filtered_twist();
  publish_uv_state();

  response->success = true;
  response->message = "ok";
}

void StingrayInterfaceBridge::handle_reset_imu(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;

  std_msgs::msg::Bool zero_yaw_msg;
  zero_yaw_msg.data = true;
  imu_zero_yaw_publisher_->publish(zero_yaw_msg);

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

bool StingrayInterfaceBridge::is_axis_closed(uint8_t bit) const
{
  return (loop_flags_msg_.data & static_cast<uint8_t>(1u << bit)) != 0u;
}

bool StingrayInterfaceBridge::has_nonzero_open_loop_command() const
{
  // bit0 surge, bit1 sway, bit2 heave(depth), bit3 yaw, bit4 pitch, bit5 roll
  if (!is_axis_closed(0) && last_twist_request_.linear.x != 0.0) {
    return true;
  }
  if (!is_axis_closed(1) && last_twist_request_.linear.y != 0.0) {
    return true;
  }
  if (!is_axis_closed(2) && last_twist_request_.linear.z != 0.0) {
    return true;
  }
  if (!is_axis_closed(3) && last_twist_request_.angular.z != 0.0) {
    return true;
  }
  if (!is_axis_closed(4) && last_twist_request_.angular.y != 0.0) {
    return true;
  }
  if (!is_axis_closed(5) && last_twist_request_.angular.x != 0.0) {
    return true;
  }
  return false;
}

void StingrayInterfaceBridge::update_republish_state()
{
  republish_enabled_ = has_twist_request_ && has_nonzero_open_loop_command();
}

void StingrayInterfaceBridge::publish_filtered_twist()
{
  if (!has_twist_request_) {
    return;
  }

  // Publish full command as-is. This keeps closed-loop setpoints updated (e.g. yaw=180).
  twist_msg_ = last_twist_request_;

  control_data_publisher_->publish(twist_msg_);
}
