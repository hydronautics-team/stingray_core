#include "stingray_core_localization/vectornav_adapter.hpp"

#include <cmath>

#include "tf2/LinearMath/Quaternion.h"

namespace stingray_core::localization
{

VectornavAdapter::VectornavAdapter()
    : Node("vectornav_adapter"),
      yaw_(0.0),
      pitch_(0.0),
      roll_(0.0),
      have_imu_(false),
      have_ypr_(false)
{
    frame_id_ = declare_parameter<std::string>("frame_id", "imu_link");

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        "/vectornav/imu", rclcpp::SensorDataQoS(),
        std::bind(&VectornavAdapter::imuCallback, this, std::placeholders::_1));

    common_sub_ = create_subscription<vectornav_msgs::msg::CommonGroup>(
        "/vectornav/raw/common", rclcpp::SensorDataQoS(),
        std::bind(&VectornavAdapter::commonCallback, this, std::placeholders::_1));

    imu_pub_ =
        create_publisher<sensor_msgs::msg::Imu>("/core/sensors/imu", rclcpp::SensorDataQoS());

    RCLCPP_INFO(get_logger(), "VectorNav adapter started");
}

void VectornavAdapter::imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr &msg)
{
    latest_imu_ = *msg;
    have_imu_ = true;
    publishImu();
}

void VectornavAdapter::commonCallback(const vectornav_msgs::msg::CommonGroup::ConstSharedPtr &msg)
{
    // VectorNav publishes yaw/pitch/roll in degrees.
    yaw_ = msg->yawpitchroll.x;
    pitch_ = msg->yawpitchroll.y;
    roll_ = msg->yawpitchroll.z;

    have_ypr_ = true;

    publishImu();
}

void VectornavAdapter::publishImu()
{
    if (!have_imu_ || !have_ypr_)
    {
        return;
    }

    sensor_msgs::msg::Imu imu = latest_imu_;

    imu.header.frame_id = frame_id_;

    constexpr double DEG_TO_RAD = M_PI / 180.0;

    double yaw_rad = yaw_ * DEG_TO_RAD;
    double pitch_rad = pitch_ * DEG_TO_RAD;
    double roll_rad = roll_ * DEG_TO_RAD;

    tf2::Quaternion q;

    q.setRPY(roll_rad, pitch_rad, yaw_rad);

    q.normalize();

    imu.orientation.x = q.x();
    imu.orientation.y = q.y();
    imu.orientation.z = q.z();
    imu.orientation.w = q.w();

    imu_pub_->publish(imu);
}

} // namespace stingray_core::localization

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<stingray_core::localization::VectornavAdapter>());
    rclcpp::shutdown();
    return 0;
}
