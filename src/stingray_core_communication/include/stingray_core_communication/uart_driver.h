#ifndef STINGRAY_CORE_COMMUNICATION_UART_DRIVER
#define STINGRAY_CORE_COMMUNICATION_UART_DRIVER


#include <sstream>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "serial/serial.h"
#include "std_msgs/msg/u_int16.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "stingray_core_communication/messages/common.h"

using std::placeholders::_1;

template <class RequestMessage, class ResponseMessage>
class UartDriver {
public:
    UartDriver(std::shared_ptr<rclcpp::Node> _node) : _node(_node) {
        // ROS PARAMETERS
        // topic names
        _node->declare_parameter("driver_request_topic", "/stingray/topics/driver_request");
        _node->declare_parameter("driver_response_topic", "/stingray/topics/driver_response");
        // com params
        _node->declare_parameter("device", "/dev/ttyS0");
        _node->declare_parameter("baudrate", 57600);
        _node->declare_parameter("data_bits", 8);
        _node->declare_parameter("stop_bits", 1);
        _node->declare_parameter("parity", "none");
        _node->declare_parameter("serial_timeout", 1000);

        // Serial port initialization
        portInitialize();
        // ROS publishers
        this->driverResponsePub = _node->create_publisher<std_msgs::msg::UInt8MultiArray>(
            _node->get_parameter("driver_response_topic").as_string(), 1000);
        // ROS subscribers
        this->driverRequestSub = _node->create_subscription<std_msgs::msg::UInt8MultiArray>(
            _node->get_parameter("driver_request_topic").as_string(), 1000,
            std::bind(
                &UartDriver::inputMessage_callback,
                this, _1));

        // Outnput message container
        driverResponseMsg.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
        driverResponseMsg.layout.dim[0].size = ResponseMessage::length;
        driverResponseMsg.layout.dim[0].stride = ResponseMessage::length;
        driverResponseMsg.layout.dim[0].label = "driverResponseMsg";
        driverResponseMsg.data = { 0 };

        RCLCPP_INFO(_node->get_logger(), "Uart driver initialized");
    }

    std::shared_ptr<rclcpp::Node> _node;

private:
    void inputMessage_callback(const std_msgs::msg::UInt8MultiArray &msg) {
        std::vector<uint8_t> to_send;
        for (int i = 0; i < RequestMessage::length; i++)
            to_send.push_back(msg.data[i]);
        try {
            if (!port.isOpen()) {
                port.open();
                if (!port.isOpen())
                    RCLCPP_ERROR(_node->get_logger(), "Unable to open UART port");
            }
        } catch (serial::IOException &ex) {
            RCLCPP_ERROR(_node->get_logger(), "Serial exception when trying to open. Error: %s", ex.what());
            return;
        }
        if (!sendData(to_send)) {
            RCLCPP_ERROR(_node->get_logger(), "Unable to send message to STM32");
            return;
        }
        if (receiveData())
            driverResponsePub->publish(driverResponseMsg);
        else {
            RCLCPP_ERROR(_node->get_logger(), "Unable to receive message from STM32");
            return;
        }
    }
    void portInitialize() {
        std::string device = _node->get_parameter("device").as_string();
        int baudrate = _node->get_parameter("baudrate").as_int();
        int dataBytesInt = _node->get_parameter("data_bits").as_int();
        serial::bytesize_t dataBytes;
        switch (dataBytesInt) {
            case 5:
                dataBytes = serial::bytesize_t::fivebits;
                break;
            case 6:
                dataBytes = serial::bytesize_t::sixbits;
                break;
            case 7:
                dataBytes = serial::bytesize_t::sevenbits;
                break;
            case 8:
                dataBytes = serial::bytesize_t::eightbits;
                break;
            default:
                RCLCPP_ERROR(_node->get_logger(), "Forbidden data bytes size %d, available sizes: 5, 6, 7, 8", dataBytesInt);
                return;
        }
        std::string parityStr = _node->get_parameter("parity").as_string();
        std::transform(parityStr.begin(), parityStr.end(), parityStr.begin(), ::tolower);
        serial::parity_t parity;
        if (parityStr == "even")
            parity = serial::parity_t::parity_even;
        else if (parityStr == "odd")
            parity = serial::parity_t::parity_odd;
        else if (parityStr == "none")
            parity = serial::parity_t::parity_none;
        else {
            RCLCPP_ERROR(_node->get_logger(), "Unrecognised parity \"%s\", available parities: \"none\", \"odd\", \"even\"",
                parityStr.c_str());
            return;
        }
        int stopBitsInt = _node->get_parameter("stop_bits").as_int();
        serial::stopbits_t stopBits;
        switch (stopBitsInt) {
            case 1:
                stopBits = serial::stopbits_t::stopbits_one;
                break;
            case 2:
                stopBits = serial::stopbits_t::stopbits_two;
                break;
            default:
                RCLCPP_ERROR(_node->get_logger(), "Forbidden stop bits size %d, available sizes: 1, 2", stopBitsInt);
                return;
        }
        RCLCPP_INFO(_node->get_logger(),
            "UART settings: Device: %s, Baudrate: %d, Data bytes: %d, Parity: %s, Stop bits: %d",
            device.c_str(), baudrate, dataBytes, parityStr.c_str(), stopBitsInt);
        if (port.isOpen())
            port.close();
        port.setPort(device);
        serial::Timeout serialTimeout = serial::Timeout::simpleTimeout(_node->get_parameter("serial_timeout").as_int());
        port.setTimeout(serialTimeout);
        port.setBaudrate(baudrate);
        port.setBytesize(dataBytes);
        port.setParity(parity);
        port.setStopbits(stopBits);
    }
    bool sendData(std::vector<uint8_t> &to_send) {
        size_t toWrite = sizeof(uint8_t) * to_send.size();
        try {
            port.flush();
            size_t written = port.write(to_send);
            return written == toWrite;
        } catch (serial::IOException &ex) {
            RCLCPP_ERROR(_node->get_logger(), "Serial exception, when trying to flush and send. Error: %s", ex.what());
            return false;
        }
    }
    bool receiveData() {
        if (port.available() < ResponseMessage::length)
            return false;
        std::vector<uint8_t> answer;
        port.read(answer, ResponseMessage::length);
        driverResponseMsg.data.clear();
        for (int i = 0; i < ResponseMessage::length; i++)
            driverResponseMsg.data.push_back(answer[i]);
        RCLCPP_DEBUG(_node->get_logger(), "RECEIVE FROM STM");

        return true;
    }

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
