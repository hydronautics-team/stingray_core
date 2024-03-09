#ifndef STINGRAY_COMMUNICATION_UDP_BRIDGE
#define STINGRAY_COMMUNICATION_UDP_BRIDGE

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

using namespace std::chrono_literals;
using boost::asio::ip::address;
using boost::asio::ip::udp;
using std::placeholders::_1;

class UDPBridgeSender : public rclcpp::Node {
   public:
    UDPBridgeSender(boost::asio::io_service &io_service);
    ~UDPBridgeSender();

   private:
    void driver_request_callback(const std_msgs::msg::UInt8MultiArray &msg);

    // ROS subscribers
    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr driverRequestSub;

    // Message containers
    std_msgs::msg::UInt8MultiArray driverRequestMsg;   // Hardware bridge -> Protocol_driver

    // udp connection
    boost::asio::io_service &_io_service;
    udp::endpoint _send_endpoint;
    udp::socket _send_socket;
};

class UDPBridgeReceiver : public rclcpp::Node {
   public:
    UDPBridgeReceiver(boost::asio::io_service &io_service);
    ~UDPBridgeReceiver();
    void try_receive();

   private:
    void driver_response_callback(const boost::system::error_code &error, size_t bytes_transferred);

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

#endif  // STINGRAY_COMMUNICATION_UDP_BRIDGE
