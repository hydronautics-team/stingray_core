#include <array>
#include <bit>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>

#include "link_node_base.hpp"
#include "std_msgs/msg/string.hpp"

namespace stingray_core
{
class PressureLinkNode : public baseLink::LinkNodeBase
{
public:
    PressureLinkNode()
        : LinkNodeBase(
              "pressure_link_node", 1, 2,
              [this](void *buffer, unsigned address,
                     unsigned length) { return this->memoryRead(buffer, address, length); },
              [this](const void *buffer, unsigned address,
                     unsigned length) { return this->memoryWrite(buffer, address, length); })
    {
        data_raw_pub_ =
            this->create_publisher<std_msgs::msg::String>("data_raw", 10);
        RCLCPP_INFO(this->get_logger(), "Pressure link node initialized");
    }

private:
    void publishRaw(const float value)
    {
        auto msg = std_msgs::msg::String();
        msg.data = std::to_string(value);
        data_raw_pub_->publish(std::move(msg));
    }

    static float decodeFloatLE(const uint8_t *data)
    {
        const uint32_t raw = static_cast<uint32_t>(data[0]) |
                             (static_cast<uint32_t>(data[1]) << 8U) |
                             (static_cast<uint32_t>(data[2]) << 16U) |
                             (static_cast<uint32_t>(data[3]) << 24U);
        return std::bit_cast<float>(raw);
    }

    hydrolib::ReturnCode memoryRead(void *buffer, unsigned address,
                                    unsigned length)
    {
        if (address + length > memory_.size())
        {
            return hydrolib::ReturnCode::FAIL;
        }

        std::memcpy(buffer, memory_.data() + address, length);
        return hydrolib::ReturnCode::OK;
    }

    hydrolib::ReturnCode memoryWrite(const void *buffer, unsigned address,
                                     unsigned length)
    {
        RCLCPP_DEBUG(this->get_logger(),
                     "Pressure slave write: addr=%u len=%u",
                     address, length);

        if (address + length > memory_.size())
        {
            return hydrolib::ReturnCode::FAIL;
        }

        std::memcpy(memory_.data() + address, buffer, length);

        constexpr unsigned kValueAddress = 0U;
        constexpr unsigned kValueLength = sizeof(float);
        const bool value_is_written =
            (address <= kValueAddress) &&
            ((address + length) >= (kValueAddress + kValueLength));

        if (value_is_written)
        {
            const float value = decodeFloatLE(memory_.data() + kValueAddress);
            publishRaw(value);
        }

        return hydrolib::ReturnCode::OK;
    }

    std::array<uint8_t, 256> memory_{};
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr data_raw_pub_;
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
