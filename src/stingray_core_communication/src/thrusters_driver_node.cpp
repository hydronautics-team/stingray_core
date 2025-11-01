
#include <chrono>
#include <functional>
#include <memory>
#include <std_msgs/msg/uint8_multi_array.hpp>
#include <vector>

#include "rclcpp/rclcpp.hpp"

class ThrustersDriverNode : public rclcpp::Node {
 public:
  ThrustersDriverNode() : Node("thrusters_driver_node") {
    thrusters_sub_ = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
        "/thrusters/cmd", 10,
        std::bind(&ThrustersDriverNode::thrustersCallback, this,
                  std::placeholders::_1));

    serial_pub_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>(
        "serial_write", 10);
    RCLCPP_INFO(this->get_logger(), "Thrusters driver node initialized");
  }

 private:
  //Тут получаем сообщение от ебаной сау и начинаем формировать пакет
  // Чтобы заколотить их в сериал порт
  void thrustersCallback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg) {
    if (msg->data.empty()) {
      RCLCPP_WARN(this->get_logger(), "Empty thrusters command received");
      return 0;
    }

    std::vector<uint8_t> packet = createPacket(msg->data);
    auto serial_msg = std_msgs::msg::UInt8MultiArray();
    serial_msg.data = packet;
    serial_pub_->publish(serial_msg);

    RCLCPP_DEBUG(this->get_logger(), "Sent %zu bytes to serial", packet.size());
  }

  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr
      thrusters_sub_;

  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr serial_pub_;

  //Значения от 100 до 200, 100 - полный назад, 200 - полный вперед
  uint8_t checksum = 0;
  std::vector<uint8_t> createPacket(const std::vector<uint8_t>& thrusters) {
    std::vector<uint8_t> packet{};
    // Заголовок пакета
    packet.push_back(0xFF);
    packet.push_back(0xFD);
    //Закидываем значения в массив и считаем чек сумму
    for (auto value : thrusters) {
      packet.push_back(value);
      checksum ^= value;
    }
    packet.push_back(checksum);

    return packet;
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ThrustersDriverNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}