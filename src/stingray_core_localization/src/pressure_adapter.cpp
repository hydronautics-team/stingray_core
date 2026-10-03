#include "stingray_core_localization/pressure_adapter.hpp"

namespace stingray_core::localization
{

PressureAdapter::PressureAdapter() : Node("pressure_adapter")
{
    depth_variance_ = declare_parameter<double>("depth_variance", 0.01);

    pressure_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
        "/stingray_core/pressure_sensor/depth", rclcpp::SensorDataQoS(),
        std::bind(&PressureAdapter::pressureCallback, this, std::placeholders::_1));

    measurement_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/stingray_core/sensors/depth", rclcpp::SensorDataQoS());

    RCLCPP_INFO(get_logger(), "Pressure adapter started");
}

void PressureAdapter::pressureCallback(const geometry_msgs::msg::PointStamped::ConstSharedPtr &msg)
{
    geometry_msgs::msg::PoseWithCovarianceStamped measurement;

    measurement.header = msg->header;

    measurement.pose.pose.position.x = 0.0;
    measurement.pose.pose.position.y = 0.0;
    measurement.pose.pose.position.z = msg->point.z;

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
