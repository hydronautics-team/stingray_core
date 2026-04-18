#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>

#include "std_msgs/msg/string.hpp"

#include "pressure_panel_link_node_base.hpp"

namespace stingray_core
{
using namespace std::chrono_literals;

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

class PressurePanelLinkNode : public PPbaseLink::PanelPressureLinkNodeBase
{
public:
    PressurePanelLinkNode() : PanelPressureLinkNodeBase("pressure_panel_driver_node", 2, 3, 1)
    {
        data_raw_pub_ = this->create_publisher<std_msgs::msg::String>("data_raw", 10);
        thrusters_sub_ = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
            "/thruster/telemetry", 10, std::bind(&PressurePanelLinkNode::telemetryCallback, this, std::placeholders::_1));

        timerDisp_ = this->create_wall_timer(500ms, [this]() { publishDisplayState(); });
        timerPres_ = this->create_wall_timer(10ms, [this]() { pollPressure(); });

        // Prime the first asynchronous pressure response before the publish timer starts.
        PressureRead(static_cast<void *>(pressure_.data()), 0, pressure_.size());

        RCLCPP_INFO(this->get_logger(), "Pressure link node initialized");
    }

private:
    void publishDisplayState() { DispWrite(static_cast<const void *>(&memory_.vma_statuses), 0, sizeof(memory_.vma_statuses)); }

    void pollPressure()
    {
        PressureRead(static_cast<void *>(pressure_.data()), 0, pressure_.size());
        publishRaw(decodeUint32LE(pressure_.data()));
    }

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

    void telemetryCallback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg)
    {
        constexpr std::size_t kTelemetrySize = sizeof(int16_t) + sizeof(int16_t) + sizeof(int8_t);

        if (msg->data.size() < kTelemetrySize)
        {
            RCLCPP_WARN(this->get_logger(), "Thruster telemetry too short: expected at least %zu bytes, got %zu", kTelemetrySize,
                        msg->data.size());
            return;
        }

        std::size_t offset = 0;
        std::memcpy(&memory_.batL_voltage, msg->data.data() + offset, sizeof(memory_.batL_voltage));
        offset += sizeof(memory_.batL_voltage);
        std::memcpy(&memory_.batR_voltage, msg->data.data() + offset, sizeof(memory_.batR_voltage));
        offset += sizeof(memory_.batR_voltage);
        std::memcpy(&memory_.light_status, msg->data.data() + offset, sizeof(memory_.light_status));
    }

    MemoryMapDisplay memory_{
        .vma_statuses = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
        .light_status = 0,
        .current_mission = 0,
        .batL_voltage = -1,
        .batR_voltage = -1,
        .mission_names = {"--no name--", "--no name--", "--no name--", "--no name--"},
        .error_logs = {"--no logs--", "", "", ""},
        .free_bytes = 0,
    };

    std::array<uint8_t, 4> pressure_{};
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr data_raw_pub_;
    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr thrusters_sub_;
    rclcpp::TimerBase::SharedPtr timerDisp_;
    rclcpp::TimerBase::SharedPtr timerPres_;
};

} // namespace stingray_core

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<stingray_core::PressurePanelLinkNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
