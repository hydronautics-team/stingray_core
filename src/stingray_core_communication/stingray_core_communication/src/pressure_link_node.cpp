#include <array>
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
    void onSerialBytes(const uint8_t *data, size_t length) override
    {
        serial_line_buffer_.append(reinterpret_cast<const char *>(data), length);

        size_t delimiter_pos = 0;
        while ((delimiter_pos = serial_line_buffer_.find_first_of("\r\n")) !=
               std::string::npos)
        {
            std::string line = serial_line_buffer_.substr(0, delimiter_pos);
            serial_line_buffer_.erase(0, delimiter_pos + 1);

            while (!serial_line_buffer_.empty() &&
                   (serial_line_buffer_.front() == '\r' ||
                    serial_line_buffer_.front() == '\n'))
            {
                serial_line_buffer_.erase(0, 1);
            }

            if (!line.empty())
            {
                publishRaw(line);
            }
        }
    }

    void publishRaw(const std::string &raw_data)
    {
        auto msg = std_msgs::msg::String();
        msg.data = raw_data;
        data_raw_pub_->publish(std::move(msg));
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
        if (address + length > memory_.size())
        {
            return hydrolib::ReturnCode::FAIL;
        }

        std::memcpy(memory_.data() + address, buffer, length);

        if (address == 0U && length > 0U)
        {
            const auto *bytes = static_cast<const char *>(buffer);
            std::string raw_data(bytes, bytes + length);

            const auto null_pos = raw_data.find('\0');
            if (null_pos != std::string::npos)
            {
                raw_data.resize(null_pos);
            }

            publishRaw(raw_data);
        }

        return hydrolib::ReturnCode::OK;
    }

    std::array<uint8_t, 256> memory_{};
    std::string serial_line_buffer_;
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
