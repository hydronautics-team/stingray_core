/**
 * This node:
 * - receives movement data from pilot and transforms it into byte array and publishes it
 * - receives byte array from protocol_bridge, parses it and publishes it
 */

#include "hardware_bridge.h"
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <fstream>

HardwareBridge::HardwareBridge() : Node("HardwareBridge") {

    // ROS PARAMETERS
    // topic names
    this->declare_parameter("driver_request_topic", "/stingray/topics/driver_request");
    this->declare_parameter("uv_state_topic", "/stingray/topics/uv_state");
    this->declare_parameter("driver_response_topic", "/stingray/topics/driver_response");
    // service names
    this->declare_parameter("set_twist_srv", "/stingray/services/set_twist");
    this->declare_parameter("set_stabilization_srv", "/stingray/services/set_stabilization");
    this->declare_parameter("reset_imu_srv", "/stingray/services/reset_imu");
    this->declare_parameter("enable_thrusters_srv", "/stingray/services/enable_thrusters_srv");
    this->declare_parameter("set_device_action_srv", "/stingray/services/set_device_action");

    // ROS publishers
    this->driverRequestPub = this->create_publisher<std_msgs::msg::UInt8MultiArray>(
        this->get_parameter("driver_request_topic").as_string(), 1000);
    this->uvStatePub = this->create_publisher<stingray_core_interfaces::msg::UVState>(
        this->get_parameter("uv_state_topic").as_string(), 1000);
    // ROS subscribers
    this->driverResponseSub = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
        this->get_parameter("driver_response_topic").as_string(), 1000,
        std::bind(&HardwareBridge::driverResponseCallback, this, std::placeholders::_1));

    // ROS services
    this->setTwistSrv = this->create_service<stingray_core_interfaces::srv::SetTwist>(
        this->get_parameter("set_twist_srv").as_string(),
        std::bind(&HardwareBridge::setTwistCallback, this, std::placeholders::_1, std::placeholders::_2));
    this->setStabilizationSrv = this->create_service<stingray_core_interfaces::srv::SetStabilization>(
        this->get_parameter("set_stabilization_srv").as_string(),
        std::bind(&HardwareBridge::setStabilizationCallback, this, std::placeholders::_1, std::placeholders::_2));
    this->resetImuSrv = this->create_service<std_srvs::srv::SetBool>(
        this->get_parameter("reset_imu_srv").as_string(), std::bind(&HardwareBridge::resetImuCallback, this, std::placeholders::_1, std::placeholders::_2));
    this->enableThrustersSrv = this->create_service<std_srvs::srv::SetBool>(
        this->get_parameter("enable_thrusters_srv").as_string(), std::bind(&HardwareBridge::enableThrustersCallback, this, std::placeholders::_1, std::placeholders::_2));
    this->setDeviceActionSrv = this->create_service<stingray_core_interfaces::srv::SetDeviceAction>(
        this->get_parameter("set_device_action_srv").as_string(), std::bind(&HardwareBridge::deviceActionCallback, this, std::placeholders::_1, std::placeholders::_2));

    // Output message container
    driverRequestMsg.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
    driverRequestMsg.layout.dim[0].size = RequestNormalMessage::length;
    driverRequestMsg.layout.dim[0].stride = RequestNormalMessage::length;
    driverRequestMsg.layout.dim[0].label = "driverRequestMsg";

    // Initializing timer for publishing messages. Callback interval: 0.05 ms
    this->publishingTimer = this->create_wall_timer(50ms, std::bind(&HardwareBridge::driverRequestCallback, this));
}


void HardwareBridge::setTwistCallback(const std::shared_ptr<stingray_core_interfaces::srv::SetTwist::Request> request,
    std::shared_ptr<stingray_core_interfaces::srv::SetTwist::Response> response) {
    requestMessage.surge = request->surge;
    requestMessage.sway = request->sway;

    if (requestMessage.stab_depth) {
        requestMessage.depth = request->depth;
    } else {
        RCLCPP_ERROR(this->get_logger(), "Depth stabilization is not enabled");
        requestMessage.depth = 0;
    }
    if (requestMessage.stab_yaw) {
        requestMessage.yaw = responseMessage.yaw + request->yaw;
    } else {
        RCLCPP_ERROR(this->get_logger(), "Yaw stabilization is not enabled");
        requestMessage.yaw = 0;
    }
    if (requestMessage.stab_roll) {
        requestMessage.roll = responseMessage.roll + request->roll;
    } else {
        RCLCPP_ERROR(this->get_logger(), "Roll stabilization is not enabled");
        requestMessage.roll = 0;
    }
    if (requestMessage.stab_pitch) {
        requestMessage.pitch = responseMessage.pitch + request->pitch;
    } else {
        RCLCPP_ERROR(this->get_logger(), "Pitch stabilization is not enabled");
        requestMessage.pitch = 0;
    }

    response->success = true;
}


void HardwareBridge::resetImuCallback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
    RCLCPP_INFO(this->get_logger(), "Resetting IMU to: %d", request->data);
    requestMessage.reset_imu = request->data;

    response->success = true;
}

void HardwareBridge::enableThrustersCallback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
    RCLCPP_INFO(this->get_logger(), "Enabling thrusters: %d", request->data);
    requestMessage.enable_thrusters = request->data;

    response->success = true;
}

void HardwareBridge::setStabilizationCallback(const std::shared_ptr<stingray_core_interfaces::srv::SetStabilization::Request> request,
    std::shared_ptr<stingray_core_interfaces::srv::SetStabilization::Response> response) {

    RCLCPP_INFO(this->get_logger(), "Setting depth stabilization %d", request->depth_stabilization);
    RCLCPP_INFO(this->get_logger(), "Setting roll stabilization %d", request->roll_stabilization);
    RCLCPP_INFO(this->get_logger(), "Setting pitch stabilization %d", request->pitch_stabilization);
    RCLCPP_INFO(this->get_logger(), "Setting yaw stabilization %d", request->yaw_stabilization);
    requestMessage.stab_depth = request->depth_stabilization;
    requestMessage.stab_roll = request->roll_stabilization;
    requestMessage.stab_pitch = request->pitch_stabilization;
    requestMessage.stab_yaw = request->yaw_stabilization;

    response->success = true;
}

void HardwareBridge::deviceActionCallback(const std::shared_ptr<stingray_core_interfaces::srv::SetDeviceAction::Request> request,
    std::shared_ptr<stingray_core_interfaces::srv::SetDeviceAction::Response> response) {
    RCLCPP_INFO(this->get_logger(), "Setting device [%d] action value to %d", request->device, request->value);
    // requestMessage.dev[request->device] = request->value;

    response->success = true;
}

/** @brief Timer callback. Make byte array to publish for protocol_node and publishes it
 *
 */
void HardwareBridge::driverRequestCallback() {
    // Make output message
    std::vector<uint8_t> output_vector;
    requestMessage.pack(output_vector);
    driverRequestMsg.data.clear();
    for (int i = 0; i < RequestNormalMessage::length; i++) {
        driverRequestMsg.data.push_back(output_vector[i]);
    }
    // Publish messages
    driverRequestPub->publish(driverRequestMsg);
    RCLCPP_INFO(this->get_logger(), "Sent message: %f %f %f %f %f %f %d %d", requestMessage.surge, requestMessage.sway, requestMessage.roll, requestMessage.pitch, requestMessage.yaw, requestMessage.depth, requestMessage.dropper, requestMessage.grabber);

    requestMessage.reset_imu = false;
    requestMessage.reset_pc = false;
}

void HardwareBridge::driverResponseCallback(const std_msgs::msg::UInt8MultiArray &msg) {
    std::vector<uint8_t> received_vector;
    for (int i = 0; i < ResponseNormalMessage::length; i++) {
        received_vector.push_back(msg.data[i]);
    }
    bool ok = responseMessage.parse(received_vector);
    if (ok) {
        stingray_core_interfaces::msg::UVState uvStateMsg;
        uvStateMsg.roll = responseMessage.roll;
        uvStateMsg.pitch = responseMessage.pitch;
        uvStateMsg.yaw = responseMessage.yaw;
        uvStateMsg.roll_speed = responseMessage.roll_speed;
        uvStateMsg.pitch_speed = responseMessage.pitch_speed;
        uvStateMsg.yaw_speed = responseMessage.yaw_speed;
        uvStateMsg.depth = responseMessage.depth;
        uvStateMsg.dropper = responseMessage.dropper;
        uvStateMsg.grabber = responseMessage.grabber;
        uvStatePub->publish(uvStateMsg);
        RCLCPP_INFO(this->get_logger(), "Received message: %f %f %f %f %f %f %f %d %d", responseMessage.roll, responseMessage.pitch, responseMessage.yaw, responseMessage.roll_speed, responseMessage.pitch_speed, responseMessage.yaw_speed, responseMessage.depth, responseMessage.dropper, responseMessage.grabber);
    } else
        RCLCPP_ERROR(this->get_logger(), "Wrong checksum");
}



int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = std::make_shared<HardwareBridge>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
