#include "stingray_core_localization/pressure_adapter.hpp"

#include <stdexcept>

namespace stingray_core::localization
{

PressureAdapter::PressureAdapter() : Node("pressure_adapter")
{
    depth_variance_ = declare_parameter<double>("depth_variance", 0.01);
    depth_scale_ = declare_parameter<double>("depth_scale", 1.0);
    output_frame_ = declare_parameter<std::string>("output_frame", "odom");
    input_topic_ = declare_parameter<std::string>(
        "input_topic", "/stingray_core/pressure_sensor/depth");
    input_type_ = declare_parameter<std::string>("input_type", "point_stamped");

    if (input_type_ == "point_stamped")
    {
        pressure_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
            input_topic_, rclcpp::SensorDataQoS(),
            std::bind(&PressureAdapter::pressureCallback, this, std::placeholders::_1));
    }
    else if (input_type_ == "float32")
    {
        ms5837_sub_ = create_subscription<std_msgs::msg::Float32>(
            input_topic_, rclcpp::SensorDataQoS(),
            std::bind(&PressureAdapter::ms5837Callback, this, std::placeholders::_1));
    }
    else
    {
        throw std::invalid_argument(
            "input_type must be 'point_stamped' or 'float32', got: " + input_type_);
    }

    measurement_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/stingray_core/sensors/depth", rclcpp::SensorDataQoS());

    RCLCPP_INFO(get_logger(), "Pressure adapter started: %s [%s], scale=%.6f",
                input_topic_.c_str(), input_type_.c_str(), depth_scale_);
}

void PressureAdapter::pressureCallback(const geometry_msgs::msg::PointStamped::ConstSharedPtr &msg)
{
    publishDepth(msg->point.z, msg->header.stamp);
}

void PressureAdapter::ms5837Callback(const std_msgs::msg::Float32::ConstSharedPtr &msg)
{
    publishDepth(static_cast<double>(msg->data), now());
}

void PressureAdapter::publishDepth(double depth, const rclcpp::Time &stamp)
{
    geometry_msgs::msg::PoseWithCovarianceStamped measurement;

    measurement.header.stamp = stamp;
    measurement.header.frame_id = output_frame_;

    measurement.pose.pose.position.x = 0.0;
    measurement.pose.pose.position.y = 0.0;
    // Depth is positive down, while REP-103 odom Z is positive up.
    measurement.pose.pose.position.z = -(depth * depth_scale_);

    measurement.pose.pose.orientation.x = 0.0;
    measurement.pose.pose.orientation.y = 0.0;
    measurement.pose.pose.orientation.z = 0.0;
    measurement.pose.pose.orientation.w = 1.0;

    measurement.pose.covariance.fill(0.0);

    // Only Z is a valid measurement.
    measurement.pose.covariance[14] = depth_variance_;

    // X/Y and orientation are not measured by the pressure sensor.
    constexpr double INVALID_VARIANCE = 1e6;

    measurement.pose.covariance[0] = INVALID_VARIANCE;
    measurement.pose.covariance[7] = INVALID_VARIANCE;

    measurement.pose.covariance[21] = INVALID_VARIANCE;
    measurement.pose.covariance[28] = INVALID_VARIANCE;
    measurement.pose.covariance[35] = INVALID_VARIANCE;

    measurement_pub_->publish(measurement);
}

} // namespace stingray_core::localization

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<stingray_core::localization::PressureAdapter>());
    rclcpp::shutdown();
    return 0;
}
