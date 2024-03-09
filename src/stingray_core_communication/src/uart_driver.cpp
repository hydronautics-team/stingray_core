/**
 * This node:
 * - receives data from STM32 and publishes it
 * - receives byte array from hardware_bridge and send it to STM32 via UART
 */

#include <fstream>
#include "stingray_core_communication/uart_driver.h"
#include "stingray_core_communication/messages/normal.h"

UartDriver::UartDriver() : Node("UartDriver") {
    // ROS PARAMETERS
    // topic names
    this->declare_parameter("driver_request_topic", "/stingray/topics/driver_request");
    this->declare_parameter("driver_response_topic", "/stingray/topics/driver_response");
    // com params
    this->declare_parameter("device", "/dev/ttyS0");
    this->declare_parameter("baudrate", 57600);
    this->declare_parameter("data_bits", 8);
    this->declare_parameter("stop_bits", 1);
    this->declare_parameter("parity", "none");
    this->declare_parameter("serial_timeout", 1000);

    // Serial port initialization
    portInitialize();
    // ROS publishers
    this->driverResponsePub = this->create_publisher<std_msgs::msg::UInt8MultiArray>(
        this->get_parameter("driver_response_topic").as_string(), 1000);
    // ROS subscribers
    this->driverRequestSub = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
        this->get_parameter("driver_request_topic").as_string(), 1000,
        std::bind(
            &UartDriver::inputMessage_callback,
            this, _1));

    // Input message container
    driverRequestMsg.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
    driverRequestMsg.layout.dim[0].size = RequestNormalMessage::length;
    driverRequestMsg.layout.dim[0].stride = RequestNormalMessage::length;
    driverRequestMsg.layout.dim[0].label = "driverRequestMsg";
    driverRequestMsg.data = { 0 };
    // Outnput message container
    driverResponseMsg.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
    driverResponseMsg.layout.dim[0].size = ResponseNormalMessage::length;
    driverResponseMsg.layout.dim[0].stride = ResponseNormalMessage::length;
    driverResponseMsg.layout.dim[0].label = "driverResponseMsg";
    driverResponseMsg.data = { 0 };

    RCLCPP_INFO(this->get_logger(), "Uart driver initialized");
}
/**
 * Initialasing serial port
 * Closes port if it is closed, initialized it
 * with given parameter and DOES NOT OPEN IT.
 */
void UartDriver::portInitialize() {
    std::string device = this->get_parameter("device").as_string();
    int baudrate = this->get_parameter("baudrate").as_int();
    int dataBytesInt = this->get_parameter("data_bits").as_int();
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
            RCLCPP_ERROR(this->get_logger(), "Forbidden data bytes size %d, available sizes: 5, 6, 7, 8", dataBytesInt);
            return;
    }
    std::string parityStr = this->get_parameter("parity").as_string();
    std::transform(parityStr.begin(), parityStr.end(), parityStr.begin(), ::tolower);
    serial::parity_t parity;
    if (parityStr == "even")
        parity = serial::parity_t::parity_even;
    else if (parityStr == "odd")
        parity = serial::parity_t::parity_odd;
    else if (parityStr == "none")
        parity = serial::parity_t::parity_none;
    else {
        RCLCPP_ERROR(this->get_logger(), "Unrecognised parity \"%s\", available parities: \"none\", \"odd\", \"even\"",
            parityStr.c_str());
        return;
    }
    int stopBitsInt = this->get_parameter("stop_bits").as_int();
    serial::stopbits_t stopBits;
    switch (stopBitsInt) {
        case 1:
            stopBits = serial::stopbits_t::stopbits_one;
            break;
        case 2:
            stopBits = serial::stopbits_t::stopbits_two;
            break;
        default:
            RCLCPP_ERROR(this->get_logger(), "Forbidden stop bits size %d, available sizes: 1, 2", stopBitsInt);
            return;
    }
    RCLCPP_INFO(this->get_logger(),
        "UART settings: Device: %s, Baudrate: %d, Data bytes: %d, Parity: %s, Stop bits: %d",
        device.c_str(), baudrate, dataBytes, parityStr.c_str(), stopBitsInt);
    if (port.isOpen())
        port.close();
    port.setPort(device);
    serial::Timeout serialTimeout = serial::Timeout::simpleTimeout(this->get_parameter("serial_timeout").as_int());
    port.setTimeout(serialTimeout);
    port.setBaudrate(baudrate);
    port.setBytesize(dataBytes);
    port.setParity(parity);
    port.setStopbits(stopBits);
}

bool UartDriver::sendData() {
    std::vector<uint8_t> msg;
    for (int i = 0; i < RequestNormalMessage::length; i++)
        msg.push_back(driverRequestMsg.data[i]);
    size_t toWrite = sizeof(uint8_t) * msg.size();
    try {
        port.flush();
        size_t written = port.write(msg);
        return written == toWrite;
    } catch (serial::IOException &ex) {
        RCLCPP_ERROR(this->get_logger(), "Serial exception, when trying to flush and send. Error: %s", ex.what());
        return false;
    }
}

bool UartDriver::receiveData() {
    if (port.available() < ResponseNormalMessage::length)
        return false;
    std::vector<uint8_t> answer;
    port.read(answer, ResponseNormalMessage::length);
    driverResponseMsg.data.clear();
    for (int i = 0; i < ResponseNormalMessage::length; i++)
        driverResponseMsg.data.push_back(answer[i]);
    RCLCPP_DEBUG(this->get_logger(), "RECEIVE FROM STM");

    return true;
}

/** @brief Parse string bitwise correctly into ResponseNormalMessage and check 16bit checksum.
 *
 * @param[in]  &input String to parse.
 */
void UartDriver::inputMessage_callback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg) {
    driverRequestMsg.data.clear();
    for (auto copy : msg->data) {
        driverRequestMsg.data.push_back(copy);
    }
    for (int i = 0; i < RequestNormalMessage::length; i++)
        driverRequestMsg.data.push_back(msg->data[i]);
    try {
        if (!port.isOpen()) {
            port.open();
            if (!port.isOpen())
                RCLCPP_ERROR(this->get_logger(), "Unable to open UART port");
        }
    } catch (serial::IOException &ex) {
        RCLCPP_ERROR(this->get_logger(), "Serial exception when trying to open. Error: %s", ex.what());
        return;
    }
    if (!sendData()) {
        RCLCPP_ERROR(this->get_logger(), "Unable to send message to STM32");
        return;
    }
    if (receiveData())
        driverResponsePub->publish(driverResponseMsg);
    else {
        RCLCPP_ERROR(this->get_logger(), "Unable to receive message from STM32");
        return;
    }
}

// int main(int argc, char *argv[]) {
//     rclcpp::init(argc, argv);
//     std::shared_ptr<rclcpp::Node> node = std::make_shared<UartDriver>();
//     rclcpp::spin(node);
//     rclcpp::shutdown();
//     return 0;
// }
