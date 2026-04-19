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
    
struct MemoryMapDisplay
{
    int8_t vma_statuses[10];     // 1 = "OK"; 0 = "ERROR"
    int8_t light_status;         // 1 = ON; 0 = OFF
    int8_t current_mission;      // current mision's number
    int16_t batL_voltage;        // 1250 => 12.50
    int16_t batR_voltage;        // 1250 => 12.50
    int8_t mission_names[4][16]; // <= 4 mission names (length = 16)
    int8_t error_logs[4][16];
    int16_t free_bytes;
} __attribute__((__packed__));

using namespace std::chrono_literals;
class PanelLinkNode : public baseLink::LinkNodeBase
{
public:
    PanelLinkNode() : LinkNodeBase("Panel_driver_node", 2, 3)

    {
        thrusters_sub_ = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
            "/thruster/telemetry", 10, std::bind(&PanelLinkNode::telemetryCallback, this, std::placeholders::_1));
        data_disp_pub_ = this->create_publisher<std_msgs::msg::String>("data_disp", 10);
        auto timer_callback = [this]() -> void
        {
            serialWrite(static_cast<const void *>(displayMemomry_.data()), VmaStatusAddr_, VmaStatusLen_);
            serialWrite(static_cast<const void *>(displayMemomry_.data() + KillSwitchAddr_), KillSwitchAddr_, 1);
            serialWrite(static_cast<const void *>(displayMemomry_.data() + BatAddr_), BatAddr_, 4);
            serialRead(static_cast<void *>(displayMemomry_.data()+MissionAddr_), MissionAddr_, 1);
            publishRaw(displayMemomry_[MissionAddr_]);
        };
        serialWrite(static_cast<const void *>(mision1), BatAddr_ + 4, 16);
        serialWrite(static_cast<const void *>(mision2), BatAddr_ + 4 + 16, 16);
        serialWrite(static_cast<const void *>(mision3), BatAddr_ + 4 + 16 + 16, 16);
        serialWrite(static_cast<const void *>(mision4), BatAddr_ + 4 + 16 + 16 + 16, 16);
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

    void telemetryCallback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg)
    {
        if (!msg->data.empty())
        {
            std::memcpy(displayMemomry_.data() + BatAddr_, msg->data.data(), 4);
            std::memcpy(displayMemomry_.data() + KillSwitchAddr_, msg->data.data() + 4, 1);
        }
    }
    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr thrusters_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr data_disp_pub_;

    rclcpp::TimerBase::SharedPtr timer_;
    char mision1[16] {'g', 'a', 't', 'e'};
    char mision2[16] {'t', 'e', 's', 't'};
    char mision3[16] {'S', 't', 'o', 'p'};
    char mision4[16] {'M', 'i', 'c', 'h','a', 'e', 'l'};
    
    static constexpr uint8_t VmaStatusLen_{10};
    static constexpr uint8_t VmaStatusAddr_{0};
    static constexpr uint8_t MissionAddr_{11};
    static constexpr uint8_t BatAddr_{12};
    static constexpr uint8_t KillSwitchAddr_{10};
    std::array<uint8_t, 146> displayMemomry_{1};
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