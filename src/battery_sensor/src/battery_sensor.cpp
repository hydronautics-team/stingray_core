#include <battery_sensor/battery_sensor.hpp>
#include <rclcpp/rclcpp.hpp>

namespace stingray_core::battery_sensor {

BatterySensor::BatterySensor(rclcpp::NodeOptions options)
    : node_(
          rclcpp::Node::make_shared("battery_sensor", std::move(options))),
      config_(node_),
      data_pub_(
          node_->create_publisher<sensor_msgs::msg::BatteryState>("state", rclcpp::SensorDataQoS())),
      low_voltage_pub_(node_->create_publisher<std_msgs::msg::Int32MultiArray>(
          "voltage_warning", rclcpp::SensorDataQoS())),
      data_raw_sub_(
          node_->create_subscription<std_msgs::msg::Float64MultiArray>(
              "/stingray_core/battery_link_node/data_raw", rclcpp::SensorDataQoS(),
              [this](const std_msgs::msg::Float64MultiArray::ConstSharedPtr& msg) {
                  this->data_raw_callback(msg);
              })) {

    RCLCPP_INFO(node_->get_logger(), "Battery sensor node initialized.");
    RCLCPP_INFO(node_->get_logger(), "voltage_scale: %f", voltage_scale_);
    RCLCPP_INFO(node_->get_logger(), "current_scale: %f", current_scale_);
    RCLCPP_INFO(node_->get_logger(), "filter_window: %f", filter_window_);
    RCLCPP_INFO(node_->get_logger(), "low_battery_threshold: %f",
                low_battery_threshold_);
}

void BatterySensor::data_raw_callback(
    const std_msgs::msg::Float64MultiArray::ConstSharedPtr& msg) {
    const auto& data_array = msg->data;

    if (data_array.size() < 2) {
        RCLCPP_WARN(node_->get_logger(),
                    "Received array with only %zu elements, expected "
                    "at least 2",
                    data_array.size());
        return;
    }

    const double voltage_raw_1 = data_array[0];
    const double voltage_raw_2 = data_array[1];

    process_battery_data(voltage_raw_1, voltage_raw_2);
}

void BatterySensor::process_battery_data(const double voltage_raw_1,
                                             const double voltage_raw_2) {
    double voltage_1 = voltage_raw_1 * voltage_scale_;
    double voltage_2 = voltage_raw_2 * voltage_scale_;

    auto battery1_msg =
        sensor_msgs::msg::BatteryState();  // FIXME change ("use
                                           // ::build().data(...)")
                                           // in the future
    battery1_msg.header.stamp = node_->now();
    battery1_msg.header.frame_id = "battery_system";
    battery1_msg.location = "1";
    battery1_msg.voltage = voltage_1;
    battery1_msg.percentage = voltage_1 * 100 / 16.8;
    data_pub_->publish(battery1_msg);

    auto battery2_msg = sensor_msgs::msg::BatteryState();
    battery2_msg.header.stamp = node_->now();
    battery2_msg.header.frame_id = "battery_system";
    battery2_msg.location = "2";
    battery2_msg.voltage = voltage_2;
    battery2_msg.percentage = voltage_2 * 100 / 16.8;
    data_pub_->publish(battery2_msg);

    auto low_voltage_msg = std_msgs::msg::Int32MultiArray();
    low_voltage_msg.data.resize(2);

    if (voltage_1 < low_battery_threshold_) {
        low_voltage_msg.data[0] = 1;
        RCLCPP_WARN(node_->get_logger(), "Low battery 1 voltage: %.2f V",
                    voltage_1);
    } else {
        low_voltage_msg.data[0] = 0;
    }
    if (voltage_2 < low_battery_threshold_) {
        low_voltage_msg.data[1] = 1;
        RCLCPP_WARN(node_->get_logger(), "Low battery 2 voltage: %.2f V",
                    voltage_2);
    } else {
        low_voltage_msg.data[1] = 0;
    }

    low_voltage_pub_->publish(std::move(low_voltage_msg));
    RCLCPP_DEBUG(node_->get_logger(), "Published battery status.");
    RCLCPP_DEBUG(node_->get_logger(),
                 "Voltage on battery 1: %.2f V, Voltage on battery 2: %.2f V",
                 voltage_1, voltage_2);
}

}  // namespace stingray_core::battery_sensor