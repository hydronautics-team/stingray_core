#include <chrono>
#include <functional>
#include <memory>
#include <vector>
#include <array>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"

#define TX_HEADER_1 0xFF
#define TX_HEADER_2 0xFD

#define MAGIC_BYTE 0x00
#define HEADER_SIZE 2
#define TOTAL_MOTOR_COUNT 14
#define DATA_SIZE TOTAL_MOTOR_COUNT
#define CRC_SIZE 1

#define PACKET_SIZE (HEADER_SIZE + DATA_SIZE + CRC_SIZE)
#define MAX_SCAN_BYTES 32
#define TELEMETRY_DATA 5
#define TELEMETRY_SIZE (HEADER_SIZE + DATA_SIZE + CRC_SIZE)

namespace stingray_core
{

    class ThrustersDriverNode : public rclcpp::Node
    {
    public:
        ThrustersDriverNode() : Node("thrusters_driver_node")
        {
            thrusters_sub_ = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
                "/thruster/cmd", 10, std::bind(&ThrustersDriverNode::thrustersCallback, this, std::placeholders::_1));

            serial_pub_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>("serial_write", 10);
            serial_sub_ = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
                "serial_read", 10, std::bind(&ThrustersDriverNode::telemetryCallback, this, std::placeholders::_1));
            RCLCPP_INFO(this->get_logger(), "Thrusters driver node initialized");
        }

    private:
        rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr thrusters_sub_;
        rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr serial_sub_;

        rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr serial_pub_;

        std::array<uint8_t, PACKET_SIZE> tx_packet_;
        std::array<uint8_t, MAX_SCAN_BYTES> rx_packet_;
        telemetry_data telemetry_;

        struct telemetry_data
        {
            uint16_t vbat1_adc = 0;
            uint16_t vbat2_adc = 0;
            bool killswitch_state = false;
        };

        void thrustersCallback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg)
        {
            if (!msg->data.empty())
            {
                createPacket(msg->data, tx_packet_);
                auto serial_msg = std_msgs::msg::UInt8MultiArray();
                serial_msg.data = std::vector<uint8_t>(tx_packet_.begin(), tx_packet_.end());
                serial_pub_->publish(serial_msg);

                // RCLCPP_INFO(this->get_logger(), "Sent %zu bytes to serial", packet.size());
            }
        }

        void telemetryCallback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg)
        {
            if (!msg->data.empty())
            {
                std::vector<uint8_t> raw_rx_packet = msg->data;

                // Добавляем новые данные в буфер
                rx_packet_.insert(rx_packet_.end(), raw_rx_packet.begin(), raw_rx_packet.end());

                // Если буфер превышает максимальный размер, обрезаем его
                if (rx_packet_.size() > MAX_SCAN_BYTES)
                {
                    rx_packet_.erase(rx_packet_.begin(), rx_packet_.begin() + (rx_packet_.size() - MAX_SCAN_BYTES));
                }

                // Ищем заголовок и парсим данные
                while (rx_packet_.size() >= 2)
                {
                    // Поиск заголовка
                    size_t header_pos = 0;
                    bool header_found = false;

                    for (size_t i = 0; i <= rx_packet_.size() - 2; ++i)
                    {
                        if (rx_packet_[i] == TX_HEADER_1 && rx_packet_[i + 1] == TX_HEADER_2)
                        {
                            header_pos = i;
                            header_found = true;
                            break;
                        }
                    }

                    // Если заголовок найден, парсим данные
                    if (header_found && rx_packet_.size() >= header_pos + 7)
                    {
                        // Проверяем, что у нас достаточно данных для полной телеметрии
                        uint8_t *telemetry = &rx_packet_[header_pos];

                        if (telemetry[0] == TX_HEADER_1 && telemetry[1] == TX_HEADER_2)
                        {
                            telemetry_.vbat1_adc = telemetry[2] | (telemetry[3] << 8);
                            telemetry_.vbat2_adc = telemetry[4] | (telemetry[5] << 8);
                            telemetry_.killswitch_state = telemetry[6] & 0x01;
                        }

                        // Удаляем обработанные данные из буфера
                        rx_packet_.erase(rx_packet_.begin(), rx_packet_.begin() + header_pos + 7);
                    }
                    else
                    {
                        // Если заголовок не найден или недостаточно данных, удаляем первый байт
                        // и продолжаем поиск
                        rx_packet_.erase(rx_packet_.begin());
                    }
                }
            }
        }

        void createPacket(const std::vector<uint8_t> &thrusters, std::array<uint8_t, PACKET_SIZE> &packet)
        {
            // Заголовок пакета
            packet[0] = TX_HEADER_1;
            packet[1] = TX_HEADER_2;

            // Закидываем значения в массив и считаем контрольную сумму
            for (int i = 0; i < DATA_SIZE; i++)
            {
                if (i < thrusters.size())
                {
                    packet[HEADER_SIZE + i] = thrusters[i];
                }
                else
                {
                    packet[HEADER_SIZE + i] = 0;
                }
            }

            packet[PACKET_SIZE - 1] = Calculate_CRC8(&packet[HEADER_SIZE], DATA_SIZE);
        }

        uint8_t Calculate_CRC8(const uint8_t *data, uint16_t length)
        {
            uint8_t crc = 0x00;
            for (uint16_t i = 0; i < length; i++)
            {
                crc ^= data[i];
                for (uint8_t j = 0; j < 8; j++)
                {
                    if (crc & 0x80)
                    {
                        crc = (crc << 1) ^ 0x07;
                    }
                    else
                    {
                        crc <<= 1;
                    }
                }
            }
            return crc;
        }

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