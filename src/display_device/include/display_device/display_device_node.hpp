#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>

#define DEFAULT_PUBLISH_RATE_HZ 60
#define DEFAULT_MAX_LINE_LENGTH 15
#define DEFAULT_DEDUPE true
#define DEFAULT_DISPLAY_FIELDS 13.6

namespace stingray_core::panel_device {

class PanelDeviceNode {
   private:
    struct SystemStatus {
        unsigned battery1_percentage = 0;
        unsigned battery2_percentage = 0;
        unsigned battery1_emergency = 0;
        unsigned battery2_emergency = 0;

        bool update = false;
    };

   public:
    explicit PanelDeviceNode(
        rclcpp::NodeOptions options = rclcpp::NodeOptions());

    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
    get_node_base_interface() {
        return node_->get_node_base_interface();
    }

    rclcpp::Logger get_logger() const { return node_->get_logger(); }

   private:
    void init_sub();
    void battery_status_callback(
        const sensor_msgs::msg::BatteryState::ConstSharedPtr& msg);

    void update_display();

    SystemStatus system_status_;

    unsigned publish_rate_hz_;
    unsigned max_line_length_;
    bool dedupe_;
    double display_fields_;  // FIXME choose the field type

    std::shared_ptr<rclcpp::Node> node_;

    rclcpp::Publisher<
    rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr
        battery_status_sub_;

};

}  // namespace stingray_core::panel_device