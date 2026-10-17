#include "stingray_core_localization/dvl_adapter.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

namespace stingray_core::localization
{

DvlAdapter::DvlAdapter() : Node("dvl_adapter")
{
    output_frame_ = declare_parameter<std::string>("output_frame", "dvl_link");
    input_topic_ = declare_parameter<std::string>("input_topic", "/dvl/data");
    odometry_topic_ = declare_parameter<std::string>(
        "odometry_topic", "/stingray_core/sensors/dvl/odometry");
    altitude_topic_ = declare_parameter<std::string>(
        "altitude_topic", "/stingray_core/sensors/dvl/altitude");
    reset_timeout_ = std::chrono::milliseconds(
        declare_parameter<int>("reset_timeout_ms", 2000));

    dvl_sub_ = create_subscription<dvl_msgs::msg::DVL>(
        input_topic_, rclcpp::SensorDataQoS(),
        std::bind(&DvlAdapter::dvlCallback, this, std::placeholders::_1));

    dvl_odom_pub_ =
        create_publisher<nav_msgs::msg::Odometry>(odometry_topic_, rclcpp::SensorDataQoS());
    altitude_pub_ =
        create_publisher<std_msgs::msg::Float32>(altitude_topic_, rclcpp::SensorDataQoS());

    dvl_command_pub_ = create_publisher<dvl_msgs::msg::ConfigCommand>(
        "/dvl/config/command", rclcpp::SensorDataQoS());

    response_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    rclcpp::SubscriptionOptions response_options;
    response_options.callback_group = response_callback_group_;
    dvl_response_sub_ = create_subscription<dvl_msgs::msg::CommandResponse>(
        "/dvl/command/response", rclcpp::SensorDataQoS(),
        std::bind(&DvlAdapter::commandResponseCallback, this, std::placeholders::_1),
        response_options);

    reset_origin_service_ = create_service<std_srvs::srv::Trigger>(
        "/core/localization/reset_origin",
        std::bind(&DvlAdapter::resetOriginCallback, this,
                  std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(get_logger(), "DVL adapter started: %s -> %s, %s; reset service: %s",
                input_topic_.c_str(), odometry_topic_.c_str(), altitude_topic_.c_str(),
                "/core/localization/reset_origin");
}

void DvlAdapter::commandResponseCallback(
    const dvl_msgs::msg::CommandResponse::ConstSharedPtr &msg)
{
    if (msg->response_to != "reset_dead_reckoning")
    {
        return;
    }

    {
        std::lock_guard<std::mutex> lock(reset_response_mutex_);
        reset_response_received_ = true;
        reset_response_success_ = msg->success;
        reset_response_error_ = msg->error_message;
    }
    reset_response_cv_.notify_all();
}

void DvlAdapter::resetOriginCallback(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response)
{
    (void)request;
    std::lock_guard<std::mutex> request_lock(reset_request_mutex_);

    {
        std::lock_guard<std::mutex> response_lock(reset_response_mutex_);
        reset_response_received_ = false;
        reset_response_success_ = false;
        reset_response_error_.clear();
    }

    dvl_msgs::msg::ConfigCommand command;
    command.command = "reset_dead_reckoning";
    dvl_command_pub_->publish(command);

    std::unique_lock<std::mutex> response_lock(reset_response_mutex_);
    const bool received = reset_response_cv_.wait_for(
        response_lock, reset_timeout_, [this]() { return reset_response_received_; });

    if (!received)
    {
        response->success = false;
        response->message = "Timed out waiting for DVL reset confirmation";
        RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
        return;
    }

    response->success = reset_response_success_;
    response->message = reset_response_success_
                            ? "DVL dead-reckoning origin reset"
                            : "DVL reset failed: " + reset_response_error_;

    if (response->success)
    {
        RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
    }
    else
    {
        RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
    }
}

void DvlAdapter::dvlCallback(const dvl_msgs::msg::DVL::ConstSharedPtr &msg)
{
    if (!msg->velocity_valid)
    {
        RCLCPP_DEBUG(get_logger(), "DVL velocity is invalid, skipping measurement");
        return;
    }

    if (std::isfinite(msg->altitude) && msg->altitude >= 0.0)
    {
        std_msgs::msg::Float32 altitude;
        altitude.data = static_cast<float>(msg->altitude);
        altitude_pub_->publish(altitude);
    }
    else
    {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                             "DVL altitude is invalid, skipping altitude measurement");
    }

    nav_msgs::msg::Odometry odom;

    odom.header.stamp = msg->header.stamp;
    // The A50 driver uses its own frame name. The vehicle configuration
    // declares that its firmware output is already aligned with dvl_link.
    odom.header.frame_id = output_frame_;

    odom.child_frame_id = "base_link";

    odom.twist.twist.linear.x = msg->velocity.x;
    odom.twist.twist.linear.y = msg->velocity.y;
    odom.twist.twist.linear.z = msg->velocity.z;

    // DVL does not provide vehicle pose.
    // Give robot_localization a very large pose covariance so
    // pose fields are effectively unusable.
    constexpr double INVALID_POSE_VARIANCE = 1e6;

    odom.pose.covariance.fill(0.0);

    odom.pose.covariance[0] = INVALID_POSE_VARIANCE;
    odom.pose.covariance[7] = INVALID_POSE_VARIANCE;
    odom.pose.covariance[14] = INVALID_POSE_VARIANCE;
    odom.pose.covariance[21] = INVALID_POSE_VARIANCE;
    odom.pose.covariance[28] = INVALID_POSE_VARIANCE;
    odom.pose.covariance[35] = INVALID_POSE_VARIANCE;

    odom.twist.covariance.fill(0.0);

    // dvl_msgs/DVL contains a flattened covariance vector.
    if (msg->covariance.size() == 36)
    {
        std::copy(msg->covariance.begin(), msg->covariance.end(), odom.twist.covariance.begin());
    }
    else if (msg->covariance.size() == 9)
    {
        // The driver reports a 3x3 XYZ velocity covariance. Embed it in the
        // linear-velocity block of the 6x6 Twist covariance.
        for (size_t row = 0; row < 3; ++row)
        {
            for (size_t column = 0; column < 3; ++column)
            {
                odom.twist.covariance[row * 6 + column] = msg->covariance[row * 3 + column];
            }
        }

        odom.twist.covariance[21] = INVALID_POSE_VARIANCE;
        odom.twist.covariance[28] = INVALID_POSE_VARIANCE;
        odom.twist.covariance[35] = INVALID_POSE_VARIANCE;
    }
    else
    {
        // Conservative fallback if driver did not provide covariance.
        constexpr double FALLBACK_VARIANCE = 0.01;

        odom.twist.covariance[0] = FALLBACK_VARIANCE;
        odom.twist.covariance[7] = FALLBACK_VARIANCE;
        odom.twist.covariance[14] = FALLBACK_VARIANCE;

        odom.twist.covariance[21] = INVALID_POSE_VARIANCE;
        odom.twist.covariance[28] = INVALID_POSE_VARIANCE;
        odom.twist.covariance[35] = INVALID_POSE_VARIANCE;
    }

    dvl_odom_pub_->publish(odom);
}

} // namespace stingray_core::localization

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
    executor.add_node(std::make_shared<stingray_core::localization::DvlAdapter>());
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
