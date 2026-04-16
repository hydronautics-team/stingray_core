#include <array>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>

#include "link_node_base.hpp"
#include "std_msgs/msg/string.hpp"

namespace stingray_core
{
using namespace std::chrono_literals;

class PressureLinkNode : public baseLink::LinkNodeBase
{
public:
    PressureLinkNode() : LinkNodeBase("pressure_link_node", 2, 1)
    {

        data_raw_pub_ = this->create_publisher<std_msgs::msg::String>("data_raw", 10);
        auto timer_callback = [this]() -> void
        {
            serialRead(static_cast<void *>(memory_.data()), 0, memory_.size());
            publishRaw(decodeUint32LE(memory_.data()));
        };
        timer_ = this->create_wall_timer(10ms, timer_callback);
        RCLCPP_INFO(this->get_logger(), "Pressure link node initialized");
    }

private:
    void publishRaw(const uint32_t value_mm)
    {
        auto msg = std_msgs::msg::String();
        msg.data = std::to_string(value_mm);
        data_raw_pub_->publish(std::move(msg));
    }

    static uint32_t decodeUint32LE(const uint8_t *data)
    {
        return static_cast<uint32_t>(data[0]) | (static_cast<uint32_t>(data[1]) << 8U) | (static_cast<uint32_t>(data[2]) << 16U) |
               (static_cast<uint32_t>(data[3]) << 24U);
    }

    std::array<uint8_t, 4> memory_{};
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr data_raw_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace stingray_core

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<stingray_core::PressureLinkNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
