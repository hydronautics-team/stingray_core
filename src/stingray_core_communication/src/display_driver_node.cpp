#include <array>
#include <chrono>
#include <functional>
#include <memory>
#include <vector>

#include "link_node_base.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"

namespace stingray_core
{

class DisplayDriverNode : public baseLink::LinkNodeBase
{
public:
    DisplayDriverNode() : LinkNodeBase("display_driver_node", 2, 1)

    {
        display_sub_ = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
            "/display/status", 10, std::bind(&DisplayDriverNode::display_callback, this, std::placeholders::_1));
        display_pub_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>("/display/buttons", 10);

        RCLCPP_INFO(this->get_logger(), "Display driver node initialized");
    }

private:
    void display_callback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg)
    {
        if (!msg->data.empty())
        {
            const void *data = msg->data.data();
            unsigned length = static_cast<unsigned>(msg->data.size());
            serialWrite(data, 0, length);
        }
    }

    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr display_sub_;
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr display_pub_;

}; // class DisplayDriverNode

} // namespace stingray_core

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<stingray_core::DisplayDriverNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}