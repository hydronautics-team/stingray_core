#include "stingray_core_localization/dvl_adapter.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

namespace stingray_core::localization
{

DvlAdapter::DvlAdapter() : Node("dvl_adapter")
{
    output_frame_ = declare_parameter<std::string>("output_frame", "dvl_link");

    dvl_sub_ = create_subscription<dvl_msgs::msg::DVL>(
        "/dvl/data", rclcpp::SensorDataQoS(),
        std::bind(&DvlAdapter::dvlCallback, this, std::placeholders::_1));

    dvl_odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/stingray_core/sensors/dvl/odometry",
                                                              rclcpp::SensorDataQoS());

    RCLCPP_INFO(get_logger(),
                "DVL adapter started: /dvl/data -> /stingray_core/sensors/dvl/odometry");
}

void DvlAdapter::dvlCallback(const dvl_msgs::msg::DVL::ConstSharedPtr &msg)
{
    if (!msg->velocity_valid)
    {
        RCLCPP_DEBUG(get_logger(), "DVL velocity is invalid, skipping measurement");
        return;
    }

    nav_msgs::msg::Odometry odom;

    odom.header = msg->header;

    if (odom.header.frame_id.empty())
    {
        odom.header.frame_id = output_frame_;
    }

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
    rclcpp::spin(std::make_shared<stingray_core::localization::DvlAdapter>());
    rclcpp::shutdown();
    return 0;
}
