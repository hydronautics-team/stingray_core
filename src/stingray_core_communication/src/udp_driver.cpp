#include "stingray_core_communication/udp_driver.h"
#include "stingray_core_communication/messages/welt.h"

UDPBridgeSender::UDPBridgeSender(boost::asio::io_service& io_service) : Node("UDPBridgeSender"), _io_service(io_service), _send_socket(io_service) {

    // ROS PARAMETERS
    // topic names
    this->declare_parameter("driver_request_topic", "/stingray/topics/driver_request");
    this->declare_parameter("send_to_ip", "127.0.0.1");
    this->declare_parameter("send_to_port", 5000);

    // UDP sender
    _send_endpoint =
        udp::endpoint(address::from_string(this->get_parameter("send_to_ip").as_string()), this->get_parameter("send_to_port").as_int());
    _send_socket.open(udp::v4());
    RCLCPP_INFO(this->get_logger(), "UDPBridgeSender: socket opened. Address: %s, port: %d",
        this->get_parameter("send_to_ip").as_string().c_str(), this->get_parameter("send_to_port").as_int());

    // ROS subscribers
    this->driverRequestSub = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
        this->get_parameter("driver_request_topic").as_string(), 1, std::bind(&UDPBridgeSender::driver_request_callback, this, std::placeholders::_1));
    // Input message container
    driverRequestMsg.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
    driverRequestMsg.layout.dim[0].size = WeltMessage::length;
    driverRequestMsg.layout.dim[0].stride = WeltMessage::length;
    driverRequestMsg.layout.dim[0].label = "driverRequestMsg";
    driverRequestMsg.data = { 0 };
}

UDPBridgeSender::~UDPBridgeSender() { _send_socket.close(); }

void UDPBridgeSender::driver_request_callback(const std_msgs::msg::UInt8MultiArray &msg) {
    RCLCPP_INFO(this->get_logger(), "Received from driver");

    // std::string str(msg.data.begin(), msg.data.end());
    // RCLCPP_INFO(this->get_logger(), "Received from driver %s", str.c_str());

    boost::system::error_code err;
    _send_socket.send_to(boost::asio::buffer(msg.data), _send_endpoint, 0, err);
    RCLCPP_INFO(this->get_logger(), "Sent to gui %s", err.message().c_str());
}

UDPBridgeReceiver::UDPBridgeReceiver(boost::asio::io_service& io_service)
    : Node("UDPBridgeReceiver"), _io_service(io_service), _receive_socket(io_service) {

    // ROS PARAMETERS
    // topic names
    this->declare_parameter("driver_response_topic", "/stingray/topics/driver_response");
    this->declare_parameter("receive_from_ip", "127.0.0.1");
    this->declare_parameter("receive_from_port", 5001);

    // ROS publishers
    this->driverResponsePub = this->create_publisher<std_msgs::msg::UInt8MultiArray>(this->get_parameter("driver_response_topic").as_string(), 1);

    // UDP receiver
    _receive_socket.open(udp::v4());
    _receive_socket.bind(udp::endpoint(address::from_string(this->get_parameter("receive_from_ip").as_string()),
        this->get_parameter("receive_from_port").as_int()));
    RCLCPP_INFO(this->get_logger(), "UDPBridgeReceiver: socket binded to address: %s, port: %d",
        this->get_parameter("receive_from_ip").as_string().c_str(),
        this->get_parameter("receive_from_port").as_int());
    
    // Outnput message container
    driverResponseMsg.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
    driverResponseMsg.layout.dim[0].size = WeltMessage::length;
    driverResponseMsg.layout.dim[0].stride = WeltMessage::length;
    driverResponseMsg.layout.dim[0].label = "driverResponseMsg";
    driverResponseMsg.data = { 0 };
}

UDPBridgeReceiver::~UDPBridgeReceiver() { _receive_socket.close(); }

void UDPBridgeReceiver::driver_response_callback(const boost::system::error_code& error, size_t bytes_transferred) {
    // Make output message
    if (error) {
        RCLCPP_ERROR(this->get_logger(), "Receive failed: %s", error.message().c_str());
        return;
    }
    driverResponseMsg.data.clear();
    for (int i = 0; i < WeltMessage::length; i++) {
        RCLCPP_INFO(this->get_logger(), "Receive: %f", request_buffer[i]);
        driverResponseMsg.data.push_back(request_buffer[i]);
    }

    // Publish messages
    driverResponsePub->publish(driverResponseMsg);
    // RCLCPP_INFO(this->get_logger(), "Udp publishing to driver ...");
    try_receive();
}

void UDPBridgeReceiver::try_receive() {
    RCLCPP_INFO(this->get_logger(), "Trying to receive from gui...");
    // std::this_thread::sleep_for(std::chrono::milliseconds(500));
    _receive_socket.async_receive_from(
        boost::asio::buffer(request_buffer), _receive_endpoint,
        boost::bind(&UDPBridgeReceiver::driver_response_callback, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
}

// int main(int argc, char* argv[]) {
//     rclcpp::init(argc, argv);
//     rclcpp::executors::MultiThreadedExecutor executor;
//     boost::asio::io_service io_service;
//     auto sender = std::make_shared<UDPBridgeSender>(io_service);
//     auto receiver = std::make_shared<UDPBridgeReceiver>(io_service);
//     std::thread s([&] {
//         receiver->try_receive();
//         io_service.run();
//         });
//     executor.add_node(sender);
//     executor.add_node(receiver);
//     executor.spin();
//     rclcpp::shutdown();
//     return 0;
// }
