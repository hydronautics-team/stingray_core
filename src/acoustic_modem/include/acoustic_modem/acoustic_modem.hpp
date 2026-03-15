#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/int8_multi_array.hpp>
#include <std_msgs/msg/float32.hpp>

namespace stingray_core::acoustic_modem 
{
    class AcousticModem : public rclcpp::Node
    {
        public:
        AcousticModem();

        private:
        void raw_packet_callback(const std_msgs::msg::Int32MultiArray::ConstSharedPtr& msg);

        private:
        rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr raw_packet_sub_;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr distance_to_start_pub_;
        rclcpp::Publisher<std_msgs::msg::Int8MultiArray>::SharedPtr start_code_pub_;
    };
}