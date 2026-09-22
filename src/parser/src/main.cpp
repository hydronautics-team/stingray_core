#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2/LinearMath/Quaternion.h"

#include "AH127Cprotocol.h"

using namespace std::chrono_literals;

class AH127CPublisher : public rclcpp::Node
{
public:
    AH127CPublisher() : Node("ah127c_publisher")
    {
        // === Топик 1: IMU в стандартном формате ===
        publisher_imu_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu_full_data", 10);

        // === Топик 2: Углы Эйлера (Roll, Pitch, Yaw) ===
        publisher_euler_ =
            this->create_publisher<geometry_msgs::msg::Vector3Stamped>("/imu_euler", 10);

        protocol_ = new AH127Cprotocol("/dev/ttyUSB0", 115200);

        timer_ = this->create_wall_timer(10ms, std::bind(&AH127CPublisher::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "AH127C Publisher started (Imu + Euler)");
    }

    ~AH127CPublisher()
    {
        delete protocol_;
        RCLCPP_INFO(this->get_logger(), "AH127C Publisher stopped");
    }

private:
    void timer_callback()
    {
        protocol_->readData();
        protocol_->timeoutSlot();

        // =============================================
        // 1. sensor_msgs::msg::Imu (стандартный формат)
        // =============================================
        auto imu_msg = sensor_msgs::msg::Imu();

        imu_msg.header.stamp = this->now();
        imu_msg.header.frame_id = "imu_link";

        // Преобразование RPY → кватернион
        double roll_rad = this->protocol_->data.roll * M_PI / 180.0;
        double pitch_rad = this->protocol_->data.pitch * M_PI / 180.0;
        double yaw_rad = this->protocol_->data.yaw * M_PI / 180.0;

        tf2::Quaternion q;
        q.setRPY(roll_rad, pitch_rad, yaw_rad);

        imu_msg.orientation.x = q.x();
        imu_msg.orientation.y = q.y();
        imu_msg.orientation.z = q.z();
        imu_msg.orientation.w = q.w();

        imu_msg.angular_velocity.x = this->protocol_->data.X_rate * M_PI / 180.0;
        imu_msg.angular_velocity.y = this->protocol_->data.Y_rate * M_PI / 180.0;
        imu_msg.angular_velocity.z = this->protocol_->data.Z_rate * M_PI / 180.0;

        imu_msg.linear_acceleration.x = this->protocol_->data.X_accel;
        imu_msg.linear_acceleration.y = this->protocol_->data.Y_accel;
        imu_msg.linear_acceleration.z = this->protocol_->data.Z_accel;

        // Ковариации (пока нулевые)
        std::fill(std::begin(imu_msg.orientation_covariance),
                  std::end(imu_msg.orientation_covariance), 0.0);
        std::fill(std::begin(imu_msg.angular_velocity_covariance),
                  std::end(imu_msg.angular_velocity_covariance), 0.0);
        std::fill(std::begin(imu_msg.linear_acceleration_covariance),
                  std::end(imu_msg.linear_acceleration_covariance), 0.0);

        publisher_imu_->publish(imu_msg);

        // =============================================
        // 2. geometry_msgs::msg::Vector3Stamped (Углы Эйлера в градусах)
        // =============================================
        auto euler_msg = geometry_msgs::msg::Vector3Stamped();
        euler_msg.header.stamp = this->now();
        euler_msg.header.frame_id = "imu_link";

        euler_msg.vector.x = this->protocol_->data.roll;  // Roll (градусы)
        euler_msg.vector.y = this->protocol_->data.pitch; // Pitch (градусы)
        euler_msg.vector.z = this->protocol_->data.yaw;   // Yaw (градусы)

        publisher_euler_->publish(euler_msg);

        // Лог в терминал
        RCLCPP_INFO(this->get_logger(), "IMU: R=%.2f° P=%.2f° Y=%.2f° | Freq=100Hz",
                    this->protocol_->data.roll, this->protocol_->data.pitch,
                    this->protocol_->data.yaw);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_imu_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr publisher_euler_;
    AH127Cprotocol *protocol_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AH127CPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
