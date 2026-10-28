#pragma once

#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "dvl_msgs/msg/command_response.hpp"
#include "dvl_msgs/msg/config_command.hpp"
#include "dvl_msgs/msg/dvl.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace stingray_core::localization
{

class DvlAdapter : public rclcpp::Node
{
public:
    DvlAdapter();

private:
    void dvlCallback(const dvl_msgs::msg::DVL::ConstSharedPtr &msg);
    void commandResponseCallback(const dvl_msgs::msg::CommandResponse::ConstSharedPtr &msg);
    void resetOriginCallback(
        const std_srvs::srv::Trigger::Request::SharedPtr request,
        std_srvs::srv::Trigger::Response::SharedPtr response);

    rclcpp::Subscription<dvl_msgs::msg::DVL>::SharedPtr dvl_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr dvl_odom_pub_;
    rclcpp::Publisher<dvl_msgs::msg::ConfigCommand>::SharedPtr dvl_command_pub_;
    rclcpp::Subscription<dvl_msgs::msg::CommandResponse>::SharedPtr dvl_response_sub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_origin_service_;
    rclcpp::CallbackGroup::SharedPtr response_callback_group_;

    std::mutex reset_request_mutex_;
    std::mutex reset_response_mutex_;
    std::condition_variable reset_response_cv_;
    bool reset_response_received_{false};
    bool reset_response_success_{false};
    std::string reset_response_error_;

    std::string output_frame_;
    std::chrono::milliseconds reset_timeout_{2000};
};

} // namespace stingray_core::localization
