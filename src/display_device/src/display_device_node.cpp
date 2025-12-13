#include <display_device/display_device_node.hpp>
#include <rclcpp/rclcpp.hpp>

namespace stingray_core::display_device {

DisplayDeviceNode::DisplayDeviceNode(rclcpp::NodeOptions options)
    : node_(
          rclcpp::Node::make_shared("display_device_node", std::move(options))),

      display_status_pub_(node_->create_publisher<sensor_msgs::msg::BatteryState>(
          "status", 10)),
      low_voltage_sub_(
          node_->create_subscription<std_msgs::msg::Int32MultiArray>(
              "/stingray_core/battery/voltage_warning", 10)),
      battery_status_sub_(
          node_->create_subscription<sensor_msgs::msg::BatteryState>(
              "/stingray_core/battery/state", 10,
              [this](
                  const sensor_msgs::msg::BatteryState::ConstSharedPtr& msg) {
                  this->battery_status_callback(msg);
              })) {
    node_->declare_parameter<double>("publish_rate_hz",
                                     DEFAULT_PUBLISH_RATE_HZ);
    node_->declare_parameter<double>("max_line_length",
                                     DEFAULT_MAX_LINE_LENGTH);
    node_->declare_parameter<double>("dedupe", DEFAULT_DEDUPE);
    node_->declare_parameter<double>("display_fields", DEFAULT_DISPLAY_FIELDS);

    publish_rate_hz_ = node_->get_parameter("publish_rate_hz").as_unsigned();
    max_line_length_ = node_->get_parameter("max_line_length").as_unsigned();
    dedupe_ = node_->get_parameter("dedupe").as_bool();
    display_fields_ = node_->get_parameter("display_fields").as_double();

    auto publish_period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    timer_ = node_->create_wall_timer(publish_period, 
        std::bind(&DisplayDeviceNode::update_display, this));

    RCLCPP_INFO(node_->get_logger(), "Display device node initialized.");
    RCLCPP_INFO(node_->get_logger(), "publish_rate_hz: %f", publish_rate_hz_);
    RCLCPP_INFO(node_->get_logger(), "max_line_length: %f", max_line_length_);
    RCLCPP_INFO(node_->get_logger(), "dedupe: %f", dedupe_);
    RCLCPP_INFO(node_->get_logger(), "display_fields: %f", display_fields_);
}

void DisplayDeviceNode::low_voltage_callback(
    const std_msgs::msg::Int32MultiArray::ConstSharedPtr& msg) {
    const auto& low_voltage_msg = msg->data;

    system_status_.battery1_emergency = low_voltage_msg[0];
    system_status_.battery2_emergency = low_voltage_msg[1];

    system_status_.update = true;
}


void DisplayDeviceNode::battery_status_callback(
    const sensor_msgs::msg::BatteryState::ConstSharedPtr& msg) {
    const auto& battery_status = msg->data;
    
    if (battery_status.location == "1") {
        system_status_.battery1_percentage = battery_status.percentage;
    }
    else {
        system_status_.battery2_percentage = battery_status.percentage;
    }
    system_status_.update = true;
}

bool DisplayDeviceNode::is_publish() {
    if (!dedupe_) {
        return true;
    }
    else {
        if (system_status_.update) {
            system_status_.update = false;
            return true;
        }
        else {
            return false;
        }
    }
}

void DisplayDeviceNode::

void DisplayDeviceNode::update_display() {  
    stingray_core_msgs::msg::DisplayStatus display_msg = format_display_message(current_status);
    
    if (is_publish(display_msg)) {
        display_status_pub_->publish(display_msg);
        last_published_status_ = serialize_display_message(display_msg);
        
        RCLCPP_INFO(node_->get_logger(), "Display status published");
    }
}

}  // namespace stingray_core::display_device