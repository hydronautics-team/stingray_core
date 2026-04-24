#include <array>
#include <chrono>
#include <functional>
#include <memory>
#include <vector>

#include "link_node_base.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"

namespace stingray_core {
using namespace std::chrono_literals;
class ThrustersDriverNode : public baseLink::LinkNodeBase
{
public:
    ThrustersDriverNode()
        : LinkNodeBase("thrusters_driver_node", 2, 1)

    {
        thrusters_sub_ = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
            "/thruster/cmd", 10, std::bind(&ThrustersDriverNode::thrustersCallback, this, std::placeholders::_1));
        light_sub_ = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
            "/thruster/cmd", 10, std::bind(&ThrustersDriverNode::lightCallback, this, std::placeholders::_1));
        plate_vma_telemetry_pub_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>("/thruster/telemetry", 10);
        telemetry_.fill(0);
        auto timer_callback = [this]() -> void
        {
            serialRead(static_cast<void*>(telemetry_.data()), TelemetryAddr_, TelemetryLen_);
            auto msg = std_msgs::msg::UInt8MultiArray();
            msg.data = createPacket(telemetry_.data(), TelemetryLen_);
            plate_vma_telemetry_pub_->publish(std::move(msg));
        };
        timer_ = this->create_wall_timer(500ms, timer_callback);
        RCLCPP_INFO(this->get_logger(), "Thrusters driver node initialized");
    }

private:
    void thrustersCallback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg)
    {
        if (!msg->data.empty()) {
            std::memcpy(thrusterData_.data(), msg->data.data(), 10);
            serialWrite(static_cast<const void *>(thrusterData_.data()), 0, 10);
        }
    }
    
    void lightCallback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg) {
        if (!msg->data.empty()) {
            std::memcpy(thrusterData_.data() + 10, msg->data.data(), 2);
            serialWrite(static_cast<const void *>(thrusterData_.data() + 10), 10, 2);
        }
    }
    static std::vector<uint8_t> createPacket(const void* data, unsigned length)
    {
        const auto* value = static_cast<const uint8_t*>(data);
        return std::vector<uint8_t>(value, value + length);
    }
    static constexpr uint8_t TelemetryLen_ { 5 };
    static constexpr uint8_t TelemetryAddr_ { 14 };
    static constexpr uint8_t ThrusterPacketSize_ { 14 };
    std::array<uint8_t, ThrusterPacketSize_> thrusterData_;
    std::array<uint8_t, TelemetryLen_> telemetry_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr thrusters_sub_;
    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr light_sub_;
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr plate_vma_telemetry_pub_;
}; // class ThrustersDriverNode

} // namespace stingray_core

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<stingray_core::ThrustersDriverNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}