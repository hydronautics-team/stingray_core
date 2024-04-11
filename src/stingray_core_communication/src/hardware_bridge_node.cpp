#include <rclcpp/rclcpp.hpp>
#include "stingray_core_communication/hardware_bridge.h"
#include "stingray_core_communication/messages/normal.h"


int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("HardwareBridge");
    auto bridge = HardwareBridge<RequestNormalMessage, ResponseNormalMessage>(node);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
