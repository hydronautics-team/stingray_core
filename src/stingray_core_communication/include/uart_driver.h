#ifndef STINGRAY_CORE_COMMUNICATION_UART_DRIVER
#define STINGRAY_CORE_COMMUNICATION_UART_DRIVER

#include <serial/serial.h>

#include <sstream>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int16.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"

using std::placeholders::_1;

class UartDriver : public rclcpp::Node {
   public:
    UartDriver();

   private:
    void inputMessage_callback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg);
    void portInitialize();
    bool sendData();
    bool receiveData();

    // ROS publishers
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr driverResponsePub;
    // ROS subscribers
    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr driverRequestSub;
    // Other
    serial::Serial port;  // serial port
    // Message containers
    std_msgs::msg::UInt8MultiArray driverRequestMsg;   // Hardware bridge -> Protocol_driver
    std_msgs::msg::UInt8MultiArray driverResponseMsg;  // Protocol_driver -> Hardware bridge
};

#endif  // STINGRAY_CORE_COMMUNICATION_UART_DRIVER
