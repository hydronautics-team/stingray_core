/**
 * This node:
 * - receives data from STM32 and publishes it
 * - receives byte array from hardware_bridge and send it to STM32 via UART
 */

#include "stingray_core_communication/uart_driver.h"
#include "stingray_core_communication/messages/normal.h"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("UartDriver");
    auto bridge = UartDriver<RequestNormalMessage, ResponseNormalMessage>(node);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
