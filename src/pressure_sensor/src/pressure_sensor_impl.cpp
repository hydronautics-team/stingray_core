#include <pressure_sensor/pressure_sensor_impl.hpp>
#include <rclcpp/rclcpp.hpp>

namespace stingray_core::pressure_sensor {

PressureSensorNode::PressureSensorNode(rclcpp::NodeOptions options)
    : node_(rclcpp::Node::make_shared("pressure_sensor_impl",
                                      std::move(options))),
      depth_pub_(node_->create_publisher<std_msgs::msg::Float64>("depth", 10)),
      data_raw_sub_(node_->create_subscription<std_msgs::msg::String>(
          "/stingray_core/depth_link_node/data_raw", 10,
          [this](const std_msgs::msg::String::SharedPtr& msg) {
              this->data_raw_callback(msg)
          })) {
    node_->declare_parameter<double>("dump_param", DEFAULT_DUMP_PARAM);

    dump_param_ = node_->get_parameter("dump_param").as_double();

    RCLCPP_INFO(node_->get_logger(), "Pressure sensor node initialized");
    RCLCPP_INFO(node_->get_logger(), "dump_param: %.3f", dump_param_);
}

void PressureSensorNode::data_raw_callback(
    const std_msgs::msg::String::ConstSharedPtr& msg) {
    const double depth = std::stod(msg->data) * dump_param_;
    publish_depth(depth);
}

void PressureSensorNode::publish_depth(const double depth) {
    const auto depth_msg = std_msgs::msg::Float64::build().data(depth);
    depth_pub_->publish(depth_msg);
    RCLCPP_DEBUG(node_->get_logger(), "Published depth: %.3f m", depth);
}

}  // namespace stingray_core::pressure_sensor