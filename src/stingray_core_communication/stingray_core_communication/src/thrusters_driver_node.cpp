#include <array>
#include <chrono>
#include <functional>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "link_node_base.hpp"

namespace stingray_core
{

class ThrustersDriverNode : public baseLink::LinkNodeBase
{
public:
    ThrustersDriverNode() : LinkNodeBase("thrusters_driver_node", 2, 1)

    {
        thrusters_sub_ = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
            "/thruster/cmd", 10, std::bind(&ThrustersDriverNode::thrustersCallback, this, std::placeholders::_1));
            
        RCLCPP_INFO(this->get_logger(), "Thrusters driver node initialized");
    }

private:
    void thrustersCallback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg)
    {
        if (!msg->data.empty())
        {
            const void *data = msg->data.data();
            unsigned length = static_cast<unsigned>(msg->data.size());
            serialWrite(data, 0, length);
        }
    }

    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr thrusters_sub_;

    
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