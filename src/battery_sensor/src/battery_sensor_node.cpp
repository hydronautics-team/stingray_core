#include <battery_sensor/battery_sensor_node.hpp>
#include <rclcpp/rclcpp.hpp>

namespace stingray_core::battery_sensor {

BatterySensorNode::BatterySensorNode(rclcpp::NodeOptions options)
    : node_(
          rclcpp::Node::make_shared("battery_sensor_node", std::move(options))),
      data_pub_(
          node_->create_publisher<sensor_msgs::msg::BatteryState>("state", 10)),
      low_voltage_pub_(node_->create_publisher<std_msgs::msg::Int32MultiArray>(
          "voltage_warning", 10)),
      data_raw_sub_(
          node_->create_subscription<std_msgs::msg::Float64MultiArray>(
              "/stingray_core/battery_link_node/data_raw", 10,
              [this](const std_msgs::msg::Float64MultiArray::ConstSharedPtr& msg) {
                  this->data_raw_callback(msg);
              })) {
    node_->declare_parameter<double>("voltage_scale", DEFAULT_VOLTAGE_SCALE);
    node_->declare_parameter<double>("current_scale", DEFAULT_CURRENT_SCALE);
    node_->declare_parameter<double>("filter_window", DEFAULT_FILTER_WINDOW);
    node_->declare_parameter<double>("low_battery_threshold",
                                     DEFAULT_LOW_BATTERY_THRESHOLD);

    voltage_scale_ = node_->get_parameter("voltage_scale").as_double();
    current_scale_ = node_->get_parameter("current_scale").as_double();
    filter_window_ = node_->get_parameter("filter_window").as_double();
    low_battery_threshold_ =
        node_->get_parameter("low_battery_threshold").as_double();

    RCLCPP_INFO(node_->get_logger(), "Battery sensor node initialized.");
    RCLCPP_INFO(node_->get_logger(), "voltage_scale: %f", voltage_scale_);
    RCLCPP_INFO(node_->get_logger(), "current_scale: %f", current_scale_);
    RCLCPP_INFO(node_->get_logger(), "filter_window: %f", filter_window_);
    RCLCPP_INFO(node_->get_logger(), "low_battery_threshold: %f",
                low_battery_threshold_);
}

void BatterySensorNode::data_raw_callback(
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

void BatterySensorNode::process_battery_data(const double voltage_raw_1,
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

    low_voltage_pub_->publish(low_voltage_msg);
    RCLCPP_DEBUG(node_->get_logger(), "Published battery status.");
    RCLCPP_DEBUG(node_->get_logger(),
                 "Voltage on battery 1: %.2f V, Voltage on battery 2: %.2f V",
                 voltage_1, voltage_2);
}

}  // namespace stingray_core::battery_sensor