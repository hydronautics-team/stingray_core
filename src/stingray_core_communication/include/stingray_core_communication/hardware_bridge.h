#ifndef STINGRAY_CORE_COMMUNICATION_HARDWARE_BRIDGE_H
#define STINGRAY_CORE_COMMUNICATION_HARDWARE_BRIDGE_H

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include "std_msgs/msg/string.hpp"

#include <sstream>
#include <string>
#include <vector>

#include "stingray_core_interfaces/srv/set_twist.hpp"
#include "stingray_core_interfaces/srv/set_device.hpp"
#include "stingray_core_interfaces/srv/set_stabilization.hpp"
#include "stingray_core_interfaces/msg/uv_state.hpp"
#include "stingray_core_interfaces/msg/device_state.hpp"
#include "stingray_core_interfaces/msg/device_state_array.hpp"
#include "stingray_core_communication/messages/common.h"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

template <class RequestMessage, class ResponseMessage>
class HardwareBridge {
public:
    HardwareBridge(std::shared_ptr<rclcpp::Node> _node) : _node(_node) {

        // ROS PARAMETERS
        // topic names
        _node->declare_parameter("driver_request_topic", "/stingray/topics/driver_request");
        _node->declare_parameter("uv_state_topic", "/stingray/topics/uv_state");
        _node->declare_parameter("device_state_array_topic", "/stingray/topics/device_state_array");
        _node->declare_parameter("driver_response_topic", "/stingray/topics/driver_response");
        
        // service names
        _node->declare_parameter("set_twist_srv", "/stingray/services/set_twist");
        _node->declare_parameter("set_stabilization_srv", "/stingray/services/set_stabilization");
        _node->declare_parameter("reset_imu_srv", "/stingray/services/reset_imu");
        _node->declare_parameter("enable_thrusters_srv", "/stingray/services/enable_thrusters");
        _node->declare_parameter("set_device_srv", "/stingray/services/set_device");

        // ROS publishers
        this->driverRequestPub = _node->create_publisher<std_msgs::msg::UInt8MultiArray>(
            _node->get_parameter("driver_request_topic").as_string(), 1000);
        this->uvStatePub = _node->create_publisher<stingray_core_interfaces::msg::UVState>(
            _node->get_parameter("uv_state_topic").as_string(), 1000);
        this->deviceStateArrayPub = _node->create_publisher<stingray_core_interfaces::msg::DeviceStateArray>(
            _node->get_parameter("device_state_array_topic").as_string(), 1000);
        
        // ROS subscribers
        this->driverResponseSub = _node->create_subscription<std_msgs::msg::UInt8MultiArray>(
            _node->get_parameter("driver_response_topic").as_string(), 1000,
            std::bind(&HardwareBridge::driverResponseCallback, this, std::placeholders::_1));

        // ROS services
        this->setTwistSrv = _node->create_service<stingray_core_interfaces::srv::SetTwist>(
            _node->get_parameter("set_twist_srv").as_string(),
            std::bind(&HardwareBridge::setTwistCallback, this, std::placeholders::_1, std::placeholders::_2));
        this->setStabilizationSrv = _node->create_service<stingray_core_interfaces::srv::SetStabilization>(
            _node->get_parameter("set_stabilization_srv").as_string(),
            std::bind(&HardwareBridge::setStabilizationCallback, this, std::placeholders::_1, std::placeholders::_2));
        this->resetImuSrv = _node->create_service<std_srvs::srv::Trigger>(
            _node->get_parameter("reset_imu_srv").as_string(), std::bind(&HardwareBridge::resetImuCallback, this, std::placeholders::_1, std::placeholders::_2));
        this->enableThrustersSrv = _node->create_service<std_srvs::srv::SetBool>(
            _node->get_parameter("enable_thrusters_srv").as_string(), std::bind(&HardwareBridge::enableThrustersCallback, this, std::placeholders::_1, std::placeholders::_2));
        this->setDeviceActionSrv = _node->create_service<stingray_core_interfaces::srv::SetDevice>(
            _node->get_parameter("set_device_srv").as_string(), std::bind(&HardwareBridge::deviceActionCallback, this, std::placeholders::_1, std::placeholders::_2));

        // Output message container
        driverRequestMsg.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
        driverRequestMsg.layout.dim[0].size = RequestMessage::length;
        driverRequestMsg.layout.dim[0].stride = RequestMessage::length;
        driverRequestMsg.layout.dim[0].label = "driverRequestMsg";

        // Initializing timer for publishing messages. Callback interval: 0.05 ms
        this->publishingTimer = _node->create_wall_timer(50ms, std::bind(&HardwareBridge::driverRequestCallback, this));
    }

    std::shared_ptr<rclcpp::Node> _node;

private:
    void setTwistCallback(const std::shared_ptr<stingray_core_interfaces::srv::SetTwist::Request> request,
        std::shared_ptr<stingray_core_interfaces::srv::SetTwist::Response> response) {
        requestMessage.surge = request->surge;
        requestMessage.sway = request->sway;

        if (requestMessage.stab_depth) {
            requestMessage.depth = request->depth;
        } else {
            // RCLCPP_WARN(_node->get_logger(), "Depth stabilization is not enabled");
            requestMessage.depth = 0;
        }
        if (requestMessage.stab_yaw) {
            float request_yaw = std::fmod(uvStateMsg.yaw + request->yaw, 360);
            if (request_yaw > 180) {
                request_yaw = -(request_yaw - 180);
            } else if (request_yaw < -180) {
                request_yaw = -(request_yaw + 180);
            }
            requestMessage.yaw = request_yaw;
        } else {
            // RCLCPP_WARN(_node->get_logger(), "Yaw stabilization is not enabled");
            requestMessage.yaw = 0;
        }
        if (requestMessage.stab_roll) {
            requestMessage.roll = request->roll;
        } else {
            // RCLCPP_WARN(_node->get_logger(), "Roll stabilization is not enabled");
            requestMessage.roll = 0;
        }
        if (requestMessage.stab_pitch) {
            requestMessage.pitch = request->pitch;
        } else {
            // RCLCPP_WARN(_node->get_logger(), "Pitch stabilization is not enabled");
            requestMessage.pitch = 0;
        }

        response->success = true;
    }
    void resetImuCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        RCLCPP_INFO(_node->get_logger(), "Resetting IMU");
        requestMessage.reset_imu = true;

        response->success = true;
    }
    void enableThrustersCallback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
        RCLCPP_INFO(_node->get_logger(), "Enabling thrusters: %d", request->data);
        // requestMessage.enable_thrusters = request->data;

        response->success = true;
    }

    void deviceActionCallback(const std::shared_ptr<stingray_core_interfaces::srv::SetDevice::Request> request,
        std::shared_ptr<stingray_core_interfaces::srv::SetDevice::Response> response) {
        RCLCPP_INFO(_node->get_logger(), "Setting device [%d] action value to %d", request->device, request->value);
        requestMessage.dev[request->device] = request->value;

        response->success = true;
    }
    void setStabilizationCallback(const std::shared_ptr<stingray_core_interfaces::srv::SetStabilization::Request> request,
        std::shared_ptr<stingray_core_interfaces::srv::SetStabilization::Response> response) {

        RCLCPP_INFO(_node->get_logger(), "Setting depth stabilization %d", request->depth_stabilization);
        RCLCPP_INFO(_node->get_logger(), "Setting roll stabilization %d", request->roll_stabilization);
        RCLCPP_INFO(_node->get_logger(), "Setting pitch stabilization %d", request->pitch_stabilization);
        RCLCPP_INFO(_node->get_logger(), "Setting yaw stabilization %d", request->yaw_stabilization);
        requestMessage.stab_depth = request->depth_stabilization;
        requestMessage.stab_roll = request->roll_stabilization;
        requestMessage.stab_pitch = request->pitch_stabilization;
        requestMessage.stab_yaw = request->yaw_stabilization;

        response->success = true;
    }
    void driverRequestCallback() {
        // Make output message
        std::vector<uint8_t> output_vector;
        requestMessage.pack(output_vector);
        driverRequestMsg.data.clear();
        for (int i = 0; i < RequestMessage::length; i++) {
            driverRequestMsg.data.push_back(output_vector[i]);
        }
        // Publish messages
        driverRequestPub->publish(driverRequestMsg);
        // RCLCPP_INFO(_node->get_logger(), "Sent yaw: %f", requestMessage.yaw);

        requestMessage.reset_imu = false;
    }
    void driverResponseCallback(const std_msgs::msg::UInt8MultiArray &msg) {
        std::vector<uint8_t> received_vector;
        for (int i = 0; i < ResponseMessage::length; i++) {
            received_vector.push_back(msg.data[i]);
        }
        bool ok = responseMessage.parse(received_vector);
        if (ok) {
            uvStateMsg.roll = responseMessage.roll;
            uvStateMsg.pitch = responseMessage.pitch;
            if (!inited_yaw) {
                yaw_delta = responseMessage.yaw;
                inited_yaw = true;
            } else {
                if (last_response_yaw < -150 && responseMessage.yaw > 150) {
                    yaw_counter--;
                } else if (last_response_yaw > 150 && responseMessage.yaw < -150) {
                    yaw_counter++;
                }
            }
            last_response_yaw = responseMessage.yaw;
            uvStateMsg.yaw = yaw_counter * 360 + responseMessage.yaw - yaw_delta;
            uvStateMsg.surge_accel = responseMessage.surge_accel;
            uvStateMsg.sway_accel = responseMessage.sway_accel;
            uvStateMsg.depth = responseMessage.depth;
            uvStateMsg.depth_stabilization = requestMessage.stab_depth;
            uvStateMsg.roll_stabilization = requestMessage.stab_roll;
            uvStateMsg.pitch_stabilization = requestMessage.stab_pitch;
            uvStateMsg.yaw_stabilization = requestMessage.stab_yaw;
            stingray_core_interfaces::msg::DeviceStateArray deviceStateArrayMsg;
            for (int i = 0; i < ResponseMessage::dev_amount; i++) {
                stingray_core_interfaces::msg::DeviceState deviceStateMsg;
                deviceStateMsg.device = i;
                deviceStateMsg.value = responseMessage.dev[i];
                deviceStateArrayMsg.states.push_back(deviceStateMsg);
            }

            uvStatePub->publish(uvStateMsg);
            deviceStateArrayPub->publish(deviceStateArrayMsg);
            // RCLCPP_INFO(_node->get_logger(), "Received yaw: %f", responseMessage.yaw);
            // RCLCPP_INFO(_node->get_logger(), "Received surge_accel: %f", responseMessage.surge_accel);
            // RCLCPP_INFO(_node->get_logger(), "Received sway_accel: %f", responseMessage.sway_accel);
            // RCLCPP_INFO(_node->get_logger(), "Absolute yaw: %f", uvStateMsg.yaw);
        } else
            RCLCPP_ERROR(_node->get_logger(), "Wrong checksum");
    }

    // ROS publishers
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr driverRequestPub;
    rclcpp::Publisher<stingray_core_interfaces::msg::UVState>::SharedPtr uvStatePub;
    rclcpp::Publisher<stingray_core_interfaces::msg::DeviceStateArray>::SharedPtr deviceStateArrayPub;

    // ROS subscribers
    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr driverResponseSub;

    // ROS services
    rclcpp::Service<stingray_core_interfaces::srv::SetTwist>::SharedPtr setTwistSrv;
    rclcpp::Service<stingray_core_interfaces::srv::SetStabilization>::SharedPtr setStabilizationSrv;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr resetImuSrv;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enableThrustersSrv;
    rclcpp::Service<stingray_core_interfaces::srv::SetDevice>::SharedPtr setDeviceActionSrv;
    // Message containers
    stingray_core_interfaces::msg::UVState uvStateMsg; // UV state
    std_msgs::msg::UInt8MultiArray driverRequestMsg; // Hardware bridge -> Protocol_bridge
    RequestMessage requestMessage;
    ResponseMessage responseMessage;
    // Other
    rclcpp::TimerBase::SharedPtr publishingTimer; // Timer for publishing messages

    float yaw_delta = 0.0;
    float last_response_yaw = 0.0;
    int yaw_counter = 0;
    bool inited_yaw = false;
};

#endif // STINGRAY_CORE_COMMUNICATION_HARDWARE_BRIDGE_H
