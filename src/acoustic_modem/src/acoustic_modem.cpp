#include <acoustic_modem/acoustic_modem.hpp>
#include <cstdint>

namespace stingray_core::acoustic_modem
{
// AcousticModem::AcousticModem() : Node("acoustic_modem")
// {
//     raw_packet_sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
//         "/modem/raw_packet", rclcpp::SensorDataQoS(),
//         [this](const std_msgs::msg::Int32MultiArray::ConstSharedPtr &msg) { this->raw_packet_callback(msg); });

//     distance_to_start_pub_ = this->create_publisher<std_msgs::msg::Float32>("/modem/distance_to_start", rclcpp::SensorDataQoS());
//     start_code_pub_ = this->create_publisher<std_msgs::msg::Int8MultiArray>("/modem/start_code", rclcpp::SensorDataQoS());
// }

AcousticModem::AcousticModem(rclcpp::NodeOptions options)
    : node_(rclcpp::Node::make_shared("acoustic_modem", std::move(options))),
      distance_to_start_pub_(node_->create_publisher<std_msgs::msg::Float32>("distance_to_start", rclcpp::SensorDataQoS())),
      start_code_pub_(node_->create_publisher<std_msgs::msg::Int8MultiArray>("start_code", rclcpp::SensorDataQoS())),
      raw_packet_sub_(node_->create_subscription<std_msgs::msg::Int32MultiArray>(
          "/stingray_core/modem_link_node/raw_packet", rclcpp::SensorDataQoS(),
          [this](const std_msgs::msg::Int32MultiArray::ConstSharedPtr &msg) { this->raw_packet_callback(msg); }))
{

    RCLCPP_INFO(node_->get_logger(), "Acoustic modem node initialized");
}

void AcousticModem::raw_packet_callback(const std_msgs::msg::Int32MultiArray::ConstSharedPtr &msg)
{
    const auto &data_array = msg->data;

    if (data_array.size() < 4)
    {
        RCLCPP_WARN(node_->get_logger(), "Invalid modem packet");
        return;
    }

    float distance = data_array[0] / 100.0;
    signed char start_code[3];
    for (int i = 0; i < 3; i++)
    {
        start_code[i] = static_cast<signed char>(data_array[i + 1]);
    }

    auto message_distance = std_msgs::msg::Float32();
    message_distance.data = distance;
    distance_to_start_pub_->publish(message_distance);

    auto message_start_code = std_msgs::msg::Int8MultiArray();
    message_start_code.data.assign(start_code, start_code + 3);
    start_code_pub_->publish(message_start_code);
}
} // namespace stingray_core::acoustic_modem