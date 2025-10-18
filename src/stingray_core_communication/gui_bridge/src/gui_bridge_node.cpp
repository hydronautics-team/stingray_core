#include <rclcpp/rclcpp.hpp>
#include <udp_msgs/msg/udp_packet.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32.hpp>
#include <memory>
#include <vector>

using udp_msgs::msg::UdpPacket;
using std_msgs::msg::String;
using std_msgs::msg::Float32;

class GuiBridge : public rclcpp::Node {
public:
    GuiBridge() : Node("gui_bridge") {

        emergency_stop_pub_ = this->create_publisher<String>("/emergency_stop", 10);
        
        // UDP Receiver
        udp_receiver_sub_ = this->create_subscription<UdpPacket>(
            "udp_read", 10,
            [this](const UdpPacket::SharedPtr msg) {
                ParseControlData(msg->data);
            });

        sub1_ = this->create_subscription<String>(
            "/topic1", 10,
            [this](const String::SharedPtr msg) {
                SendData("ID1:" + msg->data);
            });

        sub2_ = this->create_subscription<Float32>(
            "/topic2", 10,
            [this](const Float32::SharedPtr msg) {
                SendData("ID2:" + std::to_string(msg->data));
            });

        // UDP Sender
        udp_sender_pub_ = this->create_publisher<UdpPacket>("udp_write", 10);
    }

private:
    void ParseControlData(const std::vector<uint8_t> &data) {
        std::string received_str(data.begin(), data.end());
        RCLCPP_INFO(this->get_logger(), "Received: %s", received_str.c_str());
    }

    void SendData(const std::string &data) {
        auto udp_msg = UdpPacket();
        udp_msg.data = std::vector<uint8_t>(data.begin(), data.end());
        udp_sender_pub_->publish(udp_msg);
    }
    
    rclcpp::Subscription<UdpPacket>::SharedPtr udp_receiver_sub_;
    rclcpp::Publisher<UdpPacket>::SharedPtr udp_sender_pub_;
    rclcpp::Subscription<String>::SharedPtr sub1_;
    rclcpp::Subscription<Float32>::SharedPtr sub2_;
    rclcpp::Publisher<String>::SharedPtr emergency_stop_pub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GuiBridge>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}