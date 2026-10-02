#include <cstdint>
#include <memory>

#include "link_node_base.hpp"
#include "std_msgs/msg/float64.hpp"

namespace stingray_core
{

class PressureLinkNode : public baseLink::LinkNodeBase
{
public:
    PressureLinkNode()
        : LinkNodeBase(
              "pressure_link_node",
              2,
              1,
              [this](void *buffer, unsigned address, unsigned length) {
                  return this->memoryRead(buffer, address, length);
              },
              [this](const void *buffer, unsigned address, unsigned length) {
                  return this->memoryWrite(buffer, address, length);
              })
    {
        data_raw_pub_ =
            this->create_publisher<std_msgs::msg::Float64>(
                "data_raw",
                10);

        RCLCPP_INFO(
            this->get_logger(),
            "Pressure link node initialized");
    }

private:
    static uint64_t decodeUint64LE(const uint8_t *data)
    {
        return static_cast<uint64_t>(data[0]) |
               (static_cast<uint64_t>(data[1]) << 8U) |
               (static_cast<uint64_t>(data[2]) << 16U) |
               (static_cast<uint64_t>(data[3]) << 24U) |
               (static_cast<uint64_t>(data[4]) << 32U) |
               (static_cast<uint64_t>(data[5]) << 40U) |
               (static_cast<uint64_t>(data[6]) << 48U) |
               (static_cast<uint64_t>(data[7]) << 56U);
    }

    void publishRaw(uint64_t value)
    {
        auto msg = std_msgs::msg::Float64();

        msg.data = static_cast<double>(value);

        data_raw_pub_->publish(std::move(msg));
    }

    hydrolib::ReturnCode memoryRead(
        void *buffer,
        unsigned address,
        unsigned length)
    {
        // Пока не используется.
        // Оставляем callback для совместимости с LinkNodeBase.
        (void)buffer;
        (void)address;
        (void)length;

        return hydrolib::ReturnCode::FAIL;
    }

    hydrolib::ReturnCode memoryWrite(
        const void *buffer,
        unsigned address,
        unsigned length)
    {
        RCLCPP_INFO(
            this->get_logger(),
            "memoryWrite: address=%u length=%u",
            address,
            length);

        if (length != sizeof(uint64_t))
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "Unexpected pressure data length: %u",
                length);

            return hydrolib::ReturnCode::FAIL;
        }

        const auto *data =
            static_cast<const uint8_t *>(buffer);

        RCLCPP_INFO(
            this->get_logger(),
            "RAW: %u %u %u %u %u %u %u %u",
            data[0],
            data[1],
            data[2],
            data[3],
            data[4],
            data[5],
            data[6],
            data[7]);

        const uint64_t value =
            decodeUint64LE(data);

        RCLCPP_INFO(
            this->get_logger(),
            "Pressure raw: %llu",
            static_cast<unsigned long long>(value));

        publishRaw(value);

        return hydrolib::ReturnCode::OK;
    }

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr data_raw_pub_;
};

}  // namespace stingray_core

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node =
        std::make_shared<stingray_core::PressureLinkNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}