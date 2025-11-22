#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>

#define DEFAULT_DUMP_PARAM 1.0

namespace stingray_core::pressure_sensor {

class PressureSensorNode {
   public:
    explicit PressureSensorNode(
        rclcpp::NodeOptions options = rclcpp::NodeOptions());

    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
    get_node_base_interface() {
        return node_->get_node_base_interface();
    }

    rclcpp::Logger get_logger() const { return node_->get_logger(); }

   private:
    void data_raw_callback(const std_msgs::msg::String::ConstSharedPtr& msg);
    void publish_depth(const double depth);

    std::shared_ptr<rclcpp::Node> node_;

    double dump_param_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr depth_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr data_raw_sub_;
};

}  // namespace stingray_core::pressure_sensor