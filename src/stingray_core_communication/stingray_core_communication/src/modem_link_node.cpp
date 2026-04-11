#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/int8_multi_array.hpp>

namespace stingray_core::modem_link_node
{
class ModemLink
{
public:
    explicit ModemLink(rclcpp::NodeOptions options = rclcpp::NodeOptions());

    void spin() { rclcpp::spin(node_); }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr raw_packet_pub_;
    rclcpp::Publisher<std_msgs::msg::Int8MultiArray>::SharedPtr data_raw_pub_;
    rclcpp::Publisher<std_msgs::msg::Int8MultiArray>::SharedPtr serial_pub_;
    rclcpp::Subscription<std_msgs::msg::Int8MultiArray>::SharedPtr serial_sub_;
};

ModemLink::ModemLink(rclcpp::NodeOptions options)
    : node_(rclcpp::Node::make_shared("modem_link_node", std::move(options))),
      raw_packet_pub_(node_->create_publisher<std_msgs::msg::Int32MultiArray>("raw_packet", rclcpp::SensorDataQoS())),
      data_raw_pub_(node_->create_publisher<std_msgs::msg::Int8MultiArray>("data_raw", rclcpp::SensorDataQoS()))
      serial_pub_(node_->create_publisher<std_msgs::msg::Int8MultiArray>("serial_write", rclcpp::SensorDataQoS())),
      serial_sub_(node_->create_subscription<std_msgs::msg::Int8MultiArray>(
          "serial_read", rclcpp::SensorDataQoS(),
          [this](const std_msgs::msg::Int8MultiArray::ConstSharedPtr &msg) { this->serial_sub_callback(msg); }))
{
    RCLCPP_INFO(node_->get_logger(), "Modem link node initialized");
}

void ModemLink::serial_sub_callback(const std_msgs::msg::Int8MultiArray::ConstSharedPtr &msg)
{
    const auto &data_array = msg->data;

    auto message_data_raw = std_msgs::msg::Int8MultiArray();
    message_data_raw.data.assign(data_array, data_array + data_array.length());
    start_code_pub_->publish(message_start_code);

    RCLCPP_DEBUG(node_->get_logger(), data_array);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto modem_link_node = std::make_shared<stingray_core::acoustic_modem::AcousticModem>();
    modem_link_node->spin();
    rclcpp::shutdown();
    return 0;
}

} // namespace stingray_core::modem_link_node