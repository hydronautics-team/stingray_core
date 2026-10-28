#include "stingray_core_localization/imu_adapter.hpp"

#include <functional>

#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

namespace stingray_core::localization
{

ImuAdapter::ImuAdapter() : Node("imu_adapter")
{
    input_topic_ = declare_parameter<std::string>("input_topic", "/imu_full_data");
    output_topic_ = declare_parameter<std::string>("output_topic", "/core/sensors/imu");
    frame_id_ = declare_parameter<std::string>("frame_id", "imu_link");
    const auto zero_yaw_topic =
        declare_parameter<std::string>("zero_yaw_topic", "/imu/zero_yaw");

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        input_topic_, rclcpp::SensorDataQoS(),
        std::bind(&ImuAdapter::imuCallback, this, std::placeholders::_1));
    zero_yaw_sub_ = create_subscription<std_msgs::msg::Bool>(
        zero_yaw_topic, rclcpp::QoS(1).reliable(),
        std::bind(&ImuAdapter::zeroYawCallback, this, std::placeholders::_1));
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(output_topic_, rclcpp::SensorDataQoS());

    RCLCPP_INFO(get_logger(), "IMU adapter started: %s -> %s", input_topic_.c_str(),
                output_topic_.c_str());
}

void ImuAdapter::imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr &msg)
{
    sensor_msgs::msg::Imu output = *msg;
    output.header.frame_id = frame_id_;

    const tf2::Quaternion input(
        msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
    if (input.length2() < 1e-12)
    {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Received zero IMU quaternion");
        return;
    }

    tf2::Quaternion normalized = input.normalized();
    double roll = 0.0;
    double pitch = 0.0;
    tf2::Matrix3x3(normalized).getRPY(roll, pitch, current_yaw_);
    have_orientation_ = true;

    tf2::Quaternion corrected;
    corrected.setRPY(roll, pitch, current_yaw_ - yaw_offset_);
    corrected.normalize();
    output.orientation.x = corrected.x();
    output.orientation.y = corrected.y();
    output.orientation.z = corrected.z();
    output.orientation.w = corrected.w();

    imu_pub_->publish(output);
}

void ImuAdapter::zeroYawCallback(const std_msgs::msg::Bool::ConstSharedPtr &msg)
{
    if (msg->data && have_orientation_)
    {
        yaw_offset_ = current_yaw_;
        RCLCPP_INFO(get_logger(), "Yaw zeroed at %.3f rad", yaw_offset_);
    }
}

} // namespace stingray_core::localization

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<stingray_core::localization::ImuAdapter>());
    rclcpp::shutdown();
    return 0;
}
