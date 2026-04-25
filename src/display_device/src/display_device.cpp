#include "display_device/display_device_node.hpp"
#include "display_device/msg/display_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "string"

namespace stingray_core::display_device {

DisplayDevice::DisplayDevice(rclcpp::NodeOptions options)
    : node_(
          rclcpp::Node::make_shared("display_device_node", std::move(options))),
      config_(node_),
      display_status_pub_(node_->create_publisher<sensor_msgs::msg::BatteryState>(
          "status", rclcpp::SensorDataQoS())) {

    init_subscriptions();

    auto publish_period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    timer_ = node_->create_wall_timer(publish_period, [this] { update_display(); });

    RCLCPP_INFO(node_->get_logger(), "Display device node initialized.");
    RCLCPP_INFO(node_->get_logger(), "publish_rate_hz: %f", config_.publish_rate_hz);
    RCLCPP_INFO(node_->get_logger(), "max_line_length: %f", config_.max_line_length);
    RCLCPP_INFO(node_->get_logger(), "dedupe: %f", config_.dedupe);
    RCLCPP_INFO(node_->get_logger(), "display_fields: %f", config_.display_fields);
}

void DisplayDevice::init_subscriptions() {
    low_voltage_sub_ = node_->create_subscription<std_msgs::msg::Int32MultiArray>(
        "/stingray_core/battery/voltage_warning", rclcpp::SensorDataQoS(),
        [this](
            const std_msgs::msg::Int32MultiArray::ConstSharedPtr& msg) {
            this->low_voltage_callback(msg); });
    battery_status_sub_ = node_->create_subscription<sensor_msgs::msg::BatteryState>(
        "/stingray_core/battery/state", rclcpp::SensorDataQoS(),
        [this](
            const sensor_msgs::msg::BatteryState::ConstSharedPtr& msg) {
            this->battery_status_callback(msg); });
}

void DisplayDevice::low_voltage_callback(
    const std_msgs::msg::Int32MultiArray::ConstSharedPtr& msg) {
    const auto& low_voltage_msg = msg->data;

    if (low_voltage_msg.size() >= 2) {
        current_status_.battery1_emergency = low_voltage_msg[0];
        current_status_.battery2_emergency = low_voltage_msg[1];
        
        // TODO add warnings
        
        current_status_.update = true;
    }
}

void DisplayDevice::battery_status_callback(
    const sensor_msgs::msg::BatteryState::ConstSharedPtr& msg) {
    const auto& battery_status_msg = msg->data;
    
    if (battery_status_msg.location == "1" || battery_status_msg.location == "battery1") {
        system_status_.battery1_percentage = battery_status_msg.percentage;
    }
    else if (battery_status_msg.location == "2" || battery_status_msg.location == "battery2") {
        system_status_.battery2_percentage = battery_status_msg.percentage;
    }
    system_status_.update = true;
}

display_device::msg::DisplayStatus DisplayDevice::format_display_message(const SystemStatus& status) {
    display_device::msg::DisplayStatus display_msg;
    
    std::string battery_ss = "B1: " + status.battery1_percentage + "%" + " B2: "
        + status.battery2_percentage + "%";
    display_msg.battery_info = truncate_string(battery_ss, config_.max_line_length);
    
    display_msg.timestamp = this->now();
    return display_msg;
}

std::string DisplayDevice::serialize_display_message(const display_device::msg::DisplayStatus& msg) {
    std::string ss = msg.battery_info;
    
    return ss.str();
}

std::string DisplayDevice::truncate_string(const std::string& str, unsigned max_length) {
    if (str.length() <= max_length) {
        return str;
    }
    
    if (max_length <= 3) {
        return str.substr(0, max_length);
    }
    
    return str.substr(0, max_length - 3) + "...";
}

bool DisplayDevice::should_publish(const display_device::msg::DisplayStatus& msg) {
    if (!config_.dedupe) {
        return true;
    }
    
    std::string serialized = serialize_display_message(msg);
    if (serialized != last_serialized_status_) {
        last_serialized_status_ = serialized;
        return true;
    }
    
    return false;
}

void DisplayDevice::update_display() {
    display_device::msg::DisplayStatus display_msg = format_display_message(current_status_);
    
    if (should_publish(display_msg)) {
        display_status_pub_->publish(std::move(display_msg));
        
        if (current_status_.update) {
            RCLCPP_DEBUG(this->get_logger(), "Display status updated and published");
            current_status_.update = false;
        }
    }
    
    /* TODO Add warnings and clear warnings/errors after they've been displayed
    if (!current_status_.warnings.empty() || !current_status_.errors.empty()) {
        current_status_.warnings.clear();
        current_status_.errors.clear();
        current_status_.update = true;
    }*/
}

}  // namespace stingray_core::display_device