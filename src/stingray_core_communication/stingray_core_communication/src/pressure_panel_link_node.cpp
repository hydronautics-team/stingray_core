#include <array>
#include <cstdint>
#include <cstring>
#include <memory>
#include <mutex>         
#include <string>
#include <chrono>

#include "link_node_base.hpp"
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
    PressurePanelLinkNode() : PanelPressureLinkNodeBase("pressure_panel_driver_node", 2, 1, 3)
    {
        data_raw_pub_ = this->create_publisher<std_msgs::msg::String>("data_raw", 10);
        thrusters_sub_ = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
            "/thruster/telemetry", 10, std::bind(&PressurePanelLinkNode::telemetryCallback, this, std::placeholders::_1));
        
        auto timerDisp_callback = [this]() -> void
        {
            DispWrite(static_cast<const void *>(&memory_.vma_statuses), 0, sizeof(memory_.vma_statuses));
            uint32_t pressure_value = 0;
            PressureRead(static_cast<void *>(pressure_.data()), 0, 1);
            pressure_value = decodeUint32LE(pressure_.data());
            publishRaw(pressure_value);
        };
        
        auto timerPres_callback = [this]() -> void
        {
            // uint32_t pressure_value = 0;
            // {
            //     std::lock_guard<std::mutex> lock(serial_mutex_);
            //     PressureRead(static_cast<void *>(pressure_.data()), 0, 1);
            //     pressure_value = decodeUint32LE(pressure_.data());
            // }
            // publishRaw(pressure_value);
        };
        
        timerDisp_ = this->create_wall_timer(10ms, timerDisp_callback);
        timerPres_ = this->create_wall_timer(500ms, timerPres_callback);

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
        return static_cast<uint32_t>(data[0]) | (static_cast<uint32_t>(data[1]) << 8U) |
               (static_cast<uint32_t>(data[2]) << 16U) | (static_cast<uint32_t>(data[3]) << 24U);
    }
    
    void telemetryCallback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg)
    {
        if (!msg->data.empty())
        {
            // Если доступ к memory_ также разделяется с другими потоками (например, DispWrite),
            // нужно защитить и его. Для простоты пока оставим без мьютекса,
            // но в реальном коде стоит добавить отдельный мьютекс или использовать атомарные операции.
            std::memcpy(&memory_.batL_voltage, msg->data.data(), sizeof(memory_.batL_voltage));
            std::memcpy(&memory_.batR_voltage, msg->data.data() + sizeof(memory_.batL_voltage), sizeof(memory_.batR_voltage));
            std::memcpy(&memory_.light_status, msg->data.data() + sizeof(memory_.batL_voltage) + sizeof(memory_.batR_voltage),
                        sizeof(memory_.light_status));
        }
    }

    MemoryMapDisplay memory_{
        .vma_statuses = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
        .light_status = 0,
        .current_mission = 0,
        .batL_voltage = -1,
        .batR_voltage = -1,
        .mission_names = {"--no name--", "--no name--", "--no name--", "--no name--"},
        .error_logs = {"--no logs--", "", "", ""}
    };
    
    std::array<uint8_t, 4> pressure_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr data_raw_pub_;
    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr thrusters_sub_;
    rclcpp::TimerBase::SharedPtr timerDisp_;
    rclcpp::TimerBase::SharedPtr timerPres_;
    std::mutex serial_mutex_;   // мьютекс для защиты последовательного порта
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