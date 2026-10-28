#include <algorithm>
#include <array>
#include <chrono>
#include <cstring>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "link_node_base.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"

namespace stingray_core
{
using namespace std::chrono_literals;
class ThrustersDriverNode : public baseLink::LinkNodeBase
{
public:
    ThrustersDriverNode() : LinkNodeBase("thrusters_driver_node", 2, 1)

    {
        protocol_ = this->get_parameter("motor_protocol").as_string();
        if (protocol_ != "hydrolib" && protocol_ != "nulina_crc8")
        {
            throw std::invalid_argument(
                "motor_protocol must be 'hydrolib' or 'nulina_crc8', got: " + protocol_);
        }

        thrusters_sub_ = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
            "/thruster/cmd", 10, std::bind(&ThrustersDriverNode::thrustersCallback, this, std::placeholders::_1));
        light_sub_ = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
            "/lights/cmd", 10, std::bind(&ThrustersDriverNode::lightCallback, this, std::placeholders::_1));
        plate_vma_telemetry_pub_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>("/thruster/telemetry", 10);
        if (protocol_ == "nulina_crc8")
        {
            raw_serial_pub_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>("serial_write", 10);
            raw_serial_sub_ = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
                "serial_read", 10,
                std::bind(&ThrustersDriverNode::rawTelemetryCallback, this, std::placeholders::_1));
        }
        thrusterData_.fill(0);
        telemetry_.fill(0);
        auto timer_callback = [this]() -> void
        {
            if (protocol_ == "hydrolib")
            {
                serialRead(static_cast<void *>(telemetry_.data()), TelemetryAddr_, TelemetryLen_);
                auto msg = std_msgs::msg::UInt8MultiArray();
                msg.data = createPacket(telemetry_.data(), TelemetryLen_);
                plate_vma_telemetry_pub_->publish(std::move(msg));
            }
        };
        timer_ = this->create_wall_timer(500ms, timer_callback);
        RCLCPP_INFO(this->get_logger(), "Thrusters driver initialized (motor_protocol=%s)", protocol_.c_str());
        if (protocol_ == "hydrolib")
        {
            serialWrite(static_cast<const void *>(thrusterData_.data() + 12), 12, 2);
        }
    }

private:
    void thrustersCallback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg)
    {
        if (protocol_ == "nulina_crc8")
        {
            publishNulinaPacket(msg->data);
            return;
        }

        if (!msg->data.empty())
        {
            const auto count = std::min(msg->data.size(), WeltMotorCount_);
            std::fill_n(thrusterData_.begin(), WeltMotorCount_, 0);
            std::copy_n(msg->data.begin(), count, thrusterData_.begin());
            serialWrite(static_cast<const void *>(thrusterData_.data()), 0, WeltMotorCount_);
        }
    }

    void lightCallback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg)
    {
        if (protocol_ == "hydrolib" && !msg->data.empty())
        {
            const auto count = std::min(msg->data.size(), LightChannelCount_);
            std::fill_n(thrusterData_.begin() + WeltMotorCount_, LightChannelCount_, 0);
            std::copy_n(msg->data.begin(), count, thrusterData_.begin() + WeltMotorCount_);
            serialWrite(static_cast<const void *>(thrusterData_.data() + 10), 10, 4);
        }
    }

    static uint8_t calculateCrc8(const uint8_t *data, std::size_t length)
    {
        uint8_t crc = 0;
        for (std::size_t i = 0; i < length; ++i)
        {
            crc ^= data[i];
            for (uint8_t bit = 0; bit < 8; ++bit)
            {
                crc = (crc & 0x80U) != 0U ? static_cast<uint8_t>((crc << 1U) ^ 0x07U)
                                         : static_cast<uint8_t>(crc << 1U);
            }
        }
        return crc;
    }

    void publishNulinaPacket(const std::vector<uint8_t> &commands)
    {
        std::array<uint8_t, NulinaPacketSize_> packet{};
        packet[0] = NulinaHeader1_;
        packet[1] = NulinaHeader2_;
        const auto count = std::min(commands.size(), NulinaMotorCount_);
        std::copy_n(commands.begin(), count, packet.begin() + NulinaHeaderSize_);
        packet.back() = calculateCrc8(packet.data() + NulinaHeaderSize_, NulinaMotorCount_);

        auto serial_msg = std_msgs::msg::UInt8MultiArray();
        serial_msg.data.assign(packet.begin(), packet.end());
        raw_serial_pub_->publish(std::move(serial_msg));
    }

    void rawTelemetryCallback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg)
    {
        raw_rx_buffer_.insert(raw_rx_buffer_.end(), msg->data.begin(), msg->data.end());
        if (raw_rx_buffer_.size() > MaxRawBufferSize_)
        {
            raw_rx_buffer_.erase(
                raw_rx_buffer_.begin(),
                raw_rx_buffer_.begin() + (raw_rx_buffer_.size() - MaxRawBufferSize_));
        }

        while (raw_rx_buffer_.size() >= NulinaTelemetryPacketSize_)
        {
            const auto header = std::search(
                raw_rx_buffer_.begin(), raw_rx_buffer_.end(), NulinaHeader_.begin(), NulinaHeader_.end());
            if (header == raw_rx_buffer_.end())
            {
                raw_rx_buffer_.clear();
                return;
            }
            raw_rx_buffer_.erase(raw_rx_buffer_.begin(), header);
            if (raw_rx_buffer_.size() < NulinaTelemetryPacketSize_)
            {
                return;
            }

            auto telemetry_msg = std_msgs::msg::UInt8MultiArray();
            telemetry_msg.data.assign(
                raw_rx_buffer_.begin() + NulinaHeaderSize_,
                raw_rx_buffer_.begin() + NulinaTelemetryPacketSize_);
            plate_vma_telemetry_pub_->publish(std::move(telemetry_msg));
            raw_rx_buffer_.erase(
                raw_rx_buffer_.begin(), raw_rx_buffer_.begin() + NulinaTelemetryPacketSize_);
        }
    }
    static std::vector<uint8_t> createPacket(const void *data, unsigned length)
    {
        const auto *value = static_cast<const uint8_t *>(data);
        return std::vector<uint8_t>(value, value + length);
    }
    static constexpr uint8_t TelemetryLen_{5};
    static constexpr uint8_t TelemetryAddr_{14};
    static constexpr uint8_t ThrusterPacketSize_{14};
    static constexpr std::size_t WeltMotorCount_{10};
    static constexpr std::size_t LightChannelCount_{4};
    static constexpr uint8_t NulinaHeader1_{0xFF};
    static constexpr uint8_t NulinaHeader2_{0xFD};
    static constexpr std::array<uint8_t, 2> NulinaHeader_{NulinaHeader1_, NulinaHeader2_};
    static constexpr std::size_t NulinaHeaderSize_{2};
    static constexpr std::size_t NulinaMotorCount_{6};
    static constexpr std::size_t NulinaPacketSize_{NulinaHeaderSize_ + NulinaMotorCount_ + 1};
    static constexpr std::size_t NulinaTelemetryPacketSize_{7};
    static constexpr std::size_t MaxRawBufferSize_{32};
    std::array<uint8_t, ThrusterPacketSize_> thrusterData_;
    std::array<uint8_t, TelemetryLen_> telemetry_;
    std::vector<uint8_t> raw_rx_buffer_;
    std::string protocol_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr thrusters_sub_;
    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr light_sub_;
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr plate_vma_telemetry_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr raw_serial_pub_;
    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr raw_serial_sub_;
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
