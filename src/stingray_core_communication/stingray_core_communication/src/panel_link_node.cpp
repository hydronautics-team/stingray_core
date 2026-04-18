#include <array>
#include <chrono>
#include <functional>
#include <memory>
#include <vector>

#include "link_node_base.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "std_msgs/msg/string.hpp"


namespace stingray_core
{

using namespace std::chrono_literals;
class PanelLinkNode : public baseLink::LinkNodeBase
{
public:
    PanelLinkNode() : LinkNodeBase("Panel_driver_node", 2, 3)

    {
        thrusters_sub_ = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
            "/thruster/cmd", 10, std::bind(&PanelLinkNode::thrustersCallback, this, std::placeholders::_1));
        data_disp_pub_ = this->create_publisher<std_msgs::msg::String>("data_disp", 10);
        auto timer_callback = [this]() -> void
        {
            serialWrite(static_cast<const void *>(displayMemomry_.data()), VmaStatusAddr_, VmaStatusLen_);
            serialRead(static_cast<void *>(displayMemomry_.data()+MissionAddr_), MissionAddr_, 1);
            publishRaw(displayMemomry_[MissionAddr_]);
            RCLCPP_INFO(this->get_logger(), "Send");
        };
        timer_ = this->create_wall_timer(500ms, timer_callback);
        RCLCPP_INFO(this->get_logger(), "Panel driver node initialized");
    }

private:
    void publishRaw(const uint8_t value)
    {
        auto msg = std_msgs::msg::String();
        msg.data = std::to_string(value);
        data_disp_pub_->publish(std::move(msg));
    }

    void thrustersCallback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg)
    {
        if (!msg->data.empty())
        {
            for (size_t i = 0; i < 10; i++)
            {
                displayMemomry_[i] = 1;
            }
            

            // std::memcpy(&displayMemomry_, msg->data.data(), msg->data.size());
        }
    }
    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr thrusters_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr data_disp_pub_;

    rclcpp::TimerBase::SharedPtr timer_;
    static constexpr uint8_t VmaStatusLen_{10};
    static constexpr uint8_t VmaStatusAddr_{0};
    static constexpr uint8_t MissionAddr_{11};
    std::array<uint8_t, 146> displayMemomry_{};
}; // class PanelLinkNode

} // namespace stingray_core

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<stingray_core::PanelLinkNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}