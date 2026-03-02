#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <QCoreApplication>
#include "AH127Cprotocol.h"


#include "parser/msg/data.hpp"

using namespace std::chrono_literals;

class AH127CPublisher : public rclcpp::Node


{
public:
    AH127CPublisher()
    : Node("ah127c_publisher")
    {
        publisher_ = this->create_publisher<parser::msg::Data>("imu_full_data", 10);
        
        protocol_ = new AH127Cprotocol("/dev/ttyUSB0", 9600); //for socat
        timer_ = this->create_wall_timer(
            20ms, std::bind(&AH127CPublisher::timer_callback, this));
            
    }

private:
    void timer_callback()
    {
        for(int i = 0; i < 5; i++) {
            QCoreApplication::processEvents();
        }


        auto message = parser::msg::Data();

        message.roll  = this->protocol_->data.roll;
        message.pitch = this->protocol_->data.pitch;
        message.yaw   = this->protocol_->data.yaw;

        message.accel_x = this->protocol_->data.X_accel;
        message.accel_y = this->protocol_->data.Y_accel;
        message.accel_z = this->protocol_->data.Z_accel;

        message.rate_x = this->protocol_->data.X_rate;
        message.rate_y = this->protocol_->data.Y_rate;
        message.rate_z = this->protocol_->data.Z_rate;

        message.mag_x = this->protocol_->data.X_magn;
        message.mag_y = this->protocol_->data.Y_magn;
        message.mag_z = this->protocol_->data.Z_magn;

        message.q_x = this->protocol_->data.first_qvat;
        message.q_y = this->protocol_->data.second_qvat;
        message.q_z = this->protocol_->data.third_qvat;
        message.q_w = this->protocol_->data.four_qvat;

        RCLCPP_INFO(this->get_logger(), "IMU Data Sent: R:%.2f P:%.2f Y:%.2f", 
                message.roll, message.pitch, message.yaw);

        publisher_->publish(message);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<parser::msg::Data>::SharedPtr publisher_;
    AH127Cprotocol *protocol_;
};

int main(int argc, char * argv[])
{
    int qt_argc = 1;
    char* qt_argv[] = {(char*)"imu_node"};
    QCoreApplication a(qt_argc, qt_argv);

    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<AH127CPublisher>();
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
