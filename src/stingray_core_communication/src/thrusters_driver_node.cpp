#include <chrono>
#include <functional>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"

namespace stingray_core
{

class ThrustersDriverNode : public rclcpp::Node
{
public:
    ThrustersDriverNode() : Node("thrusters_driver_node")
    {
        thrusters_sub_ = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
            "/thruster/cmd", 10, std::bind(&ThrustersDriverNode::thrustersCallback, this, std::placeholders::_1));

        serial_pub_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>("serial_write", 10);
        RCLCPP_INFO(this->get_logger(), "Thrusters driver node initialized");
    }

private:
    void thrustersCallback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg)
    {
        if (!msg->data.empty())
        {
            std::vector<uint8_t> packet = createPacket(msg->data);
            auto serial_msg = std_msgs::msg::UInt8MultiArray();
            serial_msg.data = packet;
            serial_pub_->publish(serial_msg);

            // RCLCPP_INFO(this->get_logger(), "Sent %zu bytes to serial", packet.size());
        }
    }

    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr thrusters_sub_;

    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr serial_pub_;

    std::vector<uint8_t> createPacket(const std::vector<uint8_t> &thrusters)
    {
        std::vector<uint8_t> packet;
        // Заголовок пакета
        packet.push_back(0xFF);
        packet.push_back(0xFD);

        // Закидываем значения в массив и считаем контрольную сумму
        for (auto value : thrusters)
        {
            packet.push_back(value);
        }
        // packet.push_back(150);
        // packet.push_back(150);

        return packet;
    }
}; // class ThrustersDriverNode

} // namespace stingray_core

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<stingray_core::ThrustersDriverNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}