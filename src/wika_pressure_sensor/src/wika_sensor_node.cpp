#include "wika_pressure_sensor/wika_sensor_node.hpp"
#include <rclcpp/rclcpp.hpp>
#include <stdexcept>

namespace stingray_core::wika_pressure_sensor {

WikaPressureSensorNode::WikaPressureSensorNode()
    : Node("wika_sensor_node")
{

    this->declare_parameter<double>("depth_coefficient", 1.0);
    depth_coefficient_ = this->get_parameter("depth_coefficient").as_double();

    depth_pub_ = this->create_publisher<std_msgs::msg::Float64>("depth", 10);
    
    data_raw_sub_ = this->create_subscription<std_msgs::msg::String>("/stingray_core/depth_link_node/data_raw", 10,
        std::bind(&WikaPressureSensorNode::data_raw_callback_, this, std::placeholders::_1));
    
    RCLCPP_INFO(this->get_logger(), "Wika pressure sensor node initialized");
    RCLCPP_INFO(this->get_logger(), "depth_coefficient: %.3f", depth_coefficient_);
}

void WikaPressureSensorNode::data_raw_callback_(std_msgs::msg::String::SharedPtr msg)
{
    try {
        double depth = parse_wika_data_(msg->data);
        
        publish_depth_(depth * depth_coefficient_);
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error processing sensor data: %s", e.what());
    }
}

double WikaPressureSensorNode::parse_wika_data_(const std::string& raw_data)
{
    if (raw_data.empty()) {
        throw std::runtime_error("Empty data received");
    }
    
    try {
        return std::stod(raw_data);
    } catch (const std::exception& e) {
        throw std::runtime_error("Failed to parse depth data: " + std::string(e.what()));
    }
}

void WikaPressureSensorNode::publish_depth_(double depth)
{
    auto depth_msg = std_msgs::msg::Float64();
    depth_msg.data = depth;
    depth_pub_->publish(depth_msg);

    RCLCPP_DEBUG(this->get_logger(), "Published depth: %.3f m", depth);
}
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<stingray_core::wika_pressure_sensor::WikaPressureSensorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}