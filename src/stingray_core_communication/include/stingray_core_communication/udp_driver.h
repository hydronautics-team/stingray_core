#ifndef STINGRAY_CORE_COMMUNICATION_UDP_DRIVER
#define STINGRAY_CORE_COMMUNICATION_UDP_DRIVER

#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <fstream>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <string>
#include <thread>
#include <vector>
#include "stingray_core_communication/messages/common.h"

using namespace std::chrono_literals;
using boost::asio::ip::address;
using boost::asio::ip::udp;
using std::placeholders::_1;

template <class RequestMessage>
class UDPBridgeSender {
public:
    UDPBridgeSender(std::shared_ptr<rclcpp::Node> _node, boost::asio::io_service &io_service) : _node(_node), _io_service(io_service), _send_socket(io_service) {

        // ROS PARAMETERS
        // topic names
        _node->declare_parameter("driver_request_topic", "/stingray/topics/driver_request");
        _node->declare_parameter("send_to_ip", "127.0.0.1");
        _node->declare_parameter("send_to_port", 5000);

        // UDP sender
        _send_endpoint =
            udp::endpoint(address::from_string(_node->get_parameter("send_to_ip").as_string()), _node->get_parameter("send_to_port").as_int());
        _send_socket.open(udp::v4());
        RCLCPP_INFO(_node->get_logger(), "UDPBridgeSender: socket opened. Address: %s, port: %d",
            _node->get_parameter("send_to_ip").as_string().c_str(), _node->get_parameter("send_to_port").as_int());

        // ROS subscribers
        this->driverRequestSub = _node->create_subscription<std_msgs::msg::UInt8MultiArray>(
            _node->get_parameter("driver_request_topic").as_string(), 1, std::bind(&UDPBridgeSender::driver_request_callback, this, std::placeholders::_1));
        // Input message container
        driverRequestMsg.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
        driverRequestMsg.layout.dim[0].size = RequestMessage::length;
        driverRequestMsg.layout.dim[0].stride = RequestMessage::length;
        driverRequestMsg.layout.dim[0].label = "driverRequestMsg";
        driverRequestMsg.data = { 0 };
    }
    ~UDPBridgeSender() { _send_socket.close(); }

    std::shared_ptr<rclcpp::Node> _node;


private:
    void driver_request_callback(const std_msgs::msg::UInt8MultiArray &msg) {
        RCLCPP_INFO(_node->get_logger(), "Received from driver");

        boost::system::error_code err;
        std::vector<uint8_t> to_send;
        for (int i = 0; i < RequestMessage::length; i++)
            to_send.push_back(msg.data[i]);
        _send_socket.send_to(boost::asio::buffer(to_send), _send_endpoint, 0, err);
        RCLCPP_INFO(_node->get_logger(), "Sent to gui %s", err.message().c_str());
    }

    // ROS subscribers
    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr driverRequestSub;

    // Message containers
    std_msgs::msg::UInt8MultiArray driverRequestMsg;   // Hardware bridge -> Protocol_driver

    // udp connection
    boost::asio::io_service &_io_service;
    udp::endpoint _send_endpoint;
    udp::socket _send_socket;
};

template <class ResponseMessage>
class UDPBridgeReceiver {
public:
    UDPBridgeReceiver(std::shared_ptr<rclcpp::Node> _node, boost::asio::io_service &io_service) : _node(_node), _io_service(io_service), _receive_socket(io_service) {

        // ROS PARAMETERS
        // topic names
        _node->declare_parameter("driver_response_topic", "/stingray/topics/driver_response");
        _node->declare_parameter("receive_from_ip", "127.0.0.1");
        _node->declare_parameter("receive_from_port", 5001);

        // ROS publishers
        this->driverResponsePub = _node->create_publisher<std_msgs::msg::UInt8MultiArray>(_node->get_parameter("driver_response_topic").as_string(), 1);

        // UDP receiver
        _receive_socket.open(udp::v4());
        _receive_socket.bind(udp::endpoint(address::from_string(_node->get_parameter("receive_from_ip").as_string()),
            _node->get_parameter("receive_from_port").as_int()));
        RCLCPP_INFO(_node->get_logger(), "UDPBridgeReceiver: socket binded to address: %s, port: %d",
            _node->get_parameter("receive_from_ip").as_string().c_str(),
            _node->get_parameter("receive_from_port").as_int());

        // Outnput message container
        driverResponseMsg.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
        driverResponseMsg.layout.dim[0].size = ResponseMessage::length;
        driverResponseMsg.layout.dim[0].stride = ResponseMessage::length;
        driverResponseMsg.layout.dim[0].label = "driverResponseMsg";
        driverResponseMsg.data = { 0 };
    }
    ~UDPBridgeReceiver() { _receive_socket.close(); }

    std::shared_ptr<rclcpp::Node> _node;

    void try_receive() {
        RCLCPP_INFO(_node->get_logger(), "Trying to receive from gui...");
        // std::this_thread::sleep_for(std::chrono::milliseconds(500));
        _receive_socket.async_receive_from(
            boost::asio::buffer(request_buffer), _receive_endpoint,
            boost::bind(&UDPBridgeReceiver::driver_response_callback, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
    }

private:
    void driver_response_callback(const boost::system::error_code &error, size_t bytes_transferred) {
        if (!rclcpp::ok()){
            rclcpp::shutdown();
        }
        // Make output message
        if (error) {
            RCLCPP_ERROR(_node->get_logger(), "Receive failed: %s", error.message().c_str());
            return;
        }
        driverResponseMsg.data.clear();
        for (int i = 0; i < ResponseMessage::length; i++) {
            // RCLCPP_INFO(_node->get_logger(), "Receive: %f", request_buffer[i]);
            driverResponseMsg.data.push_back(request_buffer[i]);
        }

        // Publish messages
        driverResponsePub->publish(driverResponseMsg);
        // RCLCPP_INFO(_node->get_logger(), "Udp publishing to driver ...");
        try_receive();
    }

    // ROS publishers
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr driverResponsePub;

    // Message containers
    std_msgs::msg::UInt8MultiArray driverResponseMsg;  // Protocol_driver -> Hardware bridge

    // udp connection
    boost::asio::io_service &_io_service;
    udp::endpoint _receive_endpoint;
    udp::socket _receive_socket;
    boost::array<uint8_t, 1024> request_buffer;
};

#endif  // STINGRAY_CORE_COMMUNICATION_UDP_DRIVER
