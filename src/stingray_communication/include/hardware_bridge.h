#ifndef STINGRAY_COMMUNICATION_HARDWARE_BRIDGE_H
#define STINGRAY_COMMUNICATION_HARDWARE_BRIDGE_H

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include "std_msgs/msg/string.hpp"

#include <sstream>
#include <string>
#include <vector>

#include "stingray_core_interfaces/srv/set_twist.hpp"
#include "stingray_core_interfaces/srv/set_device_action.hpp"
#include "stingray_core_interfaces/srv/set_stabilization.hpp"
#include "stingray_core_interfaces/msg/uv_state.hpp"
#include "messages/common.h"
#include "messages/normal.h"
#include "nlohmann/json.hpp"

using json = nlohmann::json;
using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

class HardwareBridge : public rclcpp::Node {
public:
    HardwareBridge();

private:
    void setTwistCallback(const std::shared_ptr<stingray_core_interfaces::srv::SetTwist::Request> request,
        std::shared_ptr<stingray_core_interfaces::srv::SetTwist::Response> response);
    void resetImuCallback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response);
    void enableThrustersCallback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response);
    void deviceActionCallback(const std::shared_ptr<stingray_core_interfaces::srv::SetDeviceAction::Request> request,
        std::shared_ptr<stingray_core_interfaces::srv::SetDeviceAction::Response> response);
    void setStabilizationCallback(const std::shared_ptr<stingray_core_interfaces::srv::SetStabilization::Request> request,
        std::shared_ptr<stingray_core_interfaces::srv::SetStabilization::Response> response);
    void driverRequestCallback();
    void driverResponseCallback(const std_msgs::msg::UInt8MultiArray &msg);

    // ROS publishers
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr driverRequestPub;
    rclcpp::Publisher<stingray_core_interfaces::msg::UVState>::SharedPtr uvStatePub;

    // ROS subscribers
    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr driverResponseSub;

    // ROS services
    rclcpp::Service<stingray_core_interfaces::srv::SetTwist>::SharedPtr setTwistSrv;
    rclcpp::Service<stingray_core_interfaces::srv::SetStabilization>::SharedPtr setStabilizationSrv;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr resetImuSrv;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enableThrustersSrv;
    rclcpp::Service<stingray_core_interfaces::srv::SetDeviceAction>::SharedPtr setDeviceActionSrv;
    // Message containers
    stingray_core_interfaces::msg::UVState uvStateMsg; // UV state
    std_msgs::msg::UInt8MultiArray driverRequestMsg; // Hardware bridge -> Protocol_bridge
    RequestNormalMessage requestMessage;
    ResponseNormalMessage responseMessage;
    // Other
    rclcpp::TimerBase::SharedPtr publishingTimer; // Timer for publishing messages
};

#endif // STINGRAY_COMMUNICATION_HARDWARE_BRIDGE_H
