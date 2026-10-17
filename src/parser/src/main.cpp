#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/imu.hpp"

#include "geometry_msgs/msg/vector3_stamped.hpp"

#include "tf2/LinearMath/Quaternion.h"

#include "AH127Cprotocol.h"

using namespace std::chrono_literals;

class AH127CPublisher : public rclcpp::Node
{
public:
    AH127CPublisher()
        : Node("ah127c_publisher")
    {
        // =====================================================
        // ROS2 PARAMETERS
        // =====================================================

        port_name_ = this->declare_parameter<std::string>(
            "port",
            "/dev/ttyUSB0"
        );

        frame_id_ = this->declare_parameter<std::string>(
            "frame_id",
            "imu_link"
        );

        enable_diagnostics_ =
            this->declare_parameter<bool>(
                "enable_diagnostics",
                true
            );

        // =====================================================
        // PUBLISHERS
        // =====================================================

        publisher_imu_ =
            this->create_publisher<
                sensor_msgs::msg::Imu
            >(
                "/imu_full_data",
                10
            );

        publisher_euler_ =
            this->create_publisher<
                geometry_msgs::msg::Vector3Stamped
            >(
                "/imu_euler",
                10
            );

        // Диагностический интеграл гироскопа.
        publisher_gyro_integrated_ =
            this->create_publisher<
                geometry_msgs::msg::Vector3Stamped
            >(
                "/imu_gyro_integrated",
                10
            );

        // Магнитометр: значения в исходных единицах.
        publisher_magnetic_ =
            this->create_publisher<
                geometry_msgs::msg::Vector3Stamped
            >(
                "/imu_magnetic_field",
                10
            );

        // =====================================================
        // PROTOCOL
        // =====================================================

        protocol_ = std::make_unique<AH127Cprotocol>(
            port_name_,
            115200
        );

        // Таймер опроса порта.
        //
        // Частота таймера: 200 Гц.
        // Частота публикации определяется датчиком:
        // новое сообщение появляется только при новом
        // корректном полном кадре.
        //
        // Это НЕ означает увеличение частоты AH127C
        // до 200 Гц.

        timer_ = this->create_wall_timer(
            5ms,
            std::bind(
                &AH127CPublisher::timer_callback,
                this
            )
        );

        RCLCPP_INFO(
            this->get_logger(),
            "AH127C Publisher started"
        );

        RCLCPP_INFO(
            this->get_logger(),
            "Port: %s",
            port_name_.c_str()
        );

        RCLCPP_INFO(
            this->get_logger(),
            "Frame: %s",
            frame_id_.c_str()
        );
    }

    ~AH127CPublisher() override
    {
        RCLCPP_INFO(
            this->get_logger(),
            "AH127C Publisher stopped"
        );
    }

private:

    // =========================================================
    // CONSTANTS
    // =========================================================

    static constexpr double PI =
        3.14159265358979323846;

    static constexpr double DEG_TO_RAD = PI / 180.0;

    static constexpr double RAD_TO_DEG = 180.0 / PI;

    // =========================================================
    // HELPERS
    // =========================================================

    static double normalizeAngle(double angle)
    {
        while (angle > 180.0)
        {
            angle -= 360.0;
        }

        while (angle < -180.0)
        {
            angle += 360.0;
        }

        return angle;
    }

    // =========================================================
    // TIMER
    // =========================================================

    void timer_callback()
    {
        try
        {
            const bool new_data =
                protocol_->readData();

            protocol_->timeoutSlot();

            // Ничего не публикуем без нового полного кадра.

            if (!new_data)
            {
                return;
            }

            const auto stamp = this->now();

            const DataFromAH127C &d =
                protocol_->data;

            // =================================================
            // 1. IMU
            // =================================================

            sensor_msgs::msg::Imu imu_msg;

            imu_msg.header.stamp = stamp;
            imu_msg.header.frame_id = frame_id_;

            // -------------------------------------------------
            // EULER -> QUATERNION
            // -------------------------------------------------

            const double roll_rad =
                d.roll * DEG_TO_RAD;

            const double pitch_rad =
                d.pitch * DEG_TO_RAD;

            const double yaw_rad =
                d.yaw * DEG_TO_RAD;

            tf2::Quaternion q;

            q.setRPY(
                roll_rad,
                pitch_rad,
                yaw_rad
            );

            q.normalize();

            imu_msg.orientation.x = q.x();
            imu_msg.orientation.y = q.y();
            imu_msg.orientation.z = q.z();
            imu_msg.orientation.w = q.w();

            // -------------------------------------------------
            // ANGULAR VELOCITY
            // -------------------------------------------------

            imu_msg.angular_velocity.x =
                d.X_rate * DEG_TO_RAD;

            imu_msg.angular_velocity.y =
                d.Y_rate * DEG_TO_RAD;

            imu_msg.angular_velocity.z =
                d.Z_rate * DEG_TO_RAD;

            // -------------------------------------------------
            // LINEAR ACCELERATION
            // -------------------------------------------------

            imu_msg.linear_acceleration.x =
                d.X_accel;

            imu_msg.linear_acceleration.y =
                d.Y_accel;

            imu_msg.linear_acceleration.z =
                d.Z_accel;

            // -------------------------------------------------
            // COVARIANCE
            // -------------------------------------------------
            //
            // Ковариации измерений пока неизвестны.
            //
            // Нулевые матрицы здесь означают
            // "ковариация неизвестна", а не
            // "измерение абсолютно точное".
            //
            // Для EKF лучше задать реальные
            // ковариации после испытаний.

            std::fill(
                imu_msg.orientation_covariance.begin(),
                imu_msg.orientation_covariance.end(),
                0.0
            );

            std::fill(
                imu_msg.angular_velocity_covariance.begin(),
                imu_msg.angular_velocity_covariance.end(),
                0.0
            );

            std::fill(
                imu_msg.linear_acceleration_covariance.begin(),
                imu_msg.linear_acceleration_covariance.end(),
                0.0
            );

            publisher_imu_->publish(imu_msg);

            // =================================================
            // 2. EULER ANGLES
            // =================================================

            geometry_msgs::msg::Vector3Stamped euler_msg;

            euler_msg.header.stamp = stamp;
            euler_msg.header.frame_id = frame_id_;

            euler_msg.vector.x = d.roll;
            euler_msg.vector.y = d.pitch;
            euler_msg.vector.z = d.yaw;

            publisher_euler_->publish(euler_msg);

            // =================================================
            // 3. MAGNETOMETER
            // =================================================
            //
            // Используем Vector3Stamped, поскольку
            // физические единицы исходных BCD-полей
            // пока не подтверждены.
            //
            // НЕ выдаём эти данные за Tesla.

            geometry_msgs::msg::Vector3Stamped magn_msg;

            magn_msg.header.stamp = stamp;
            magn_msg.header.frame_id = frame_id_;

            magn_msg.vector.x = d.X_magn;
            magn_msg.vector.y = d.Y_magn;
            magn_msg.vector.z = d.Z_magn;

            publisher_magnetic_->publish(magn_msg);

            // =================================================
            // 4. GYROSCOPE INTEGRATION
            // =================================================

            if (enable_diagnostics_)
            {
                updateGyroIntegration(
                    d,
                    stamp
                );
            }

            // =================================================
            // 5. DEBUG
            // =================================================

            RCLCPP_INFO_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                200,

                "AHRS: R=%7.2f P=%7.2f Y=%7.2f | "
                "GyroZ=%8.2f deg/s | "
                "IntegratedZ=%8.2f",

                d.roll,
                d.pitch,
                d.yaw,
                d.Z_rate,
                integrated_yaw_deg_
            );

            // =================================================
            // 6. STATISTICS
            // =================================================

            RCLCPP_INFO_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                5000,

                "Frames=%lu CRC_errors=%lu "
                "BCD_errors=%lu Discarded=%lu",

                static_cast<unsigned long>(
                    protocol_->valid_frames
                ),

                static_cast<unsigned long>(
                    protocol_->checksum_errors
                ),

                static_cast<unsigned long>(
                    protocol_->bcd_errors
                ),

                static_cast<unsigned long>(
                    protocol_->discarded_bytes
                )
            );
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,

                "AH127C error: %s",
                e.what()
            );
        }
    }

    // =========================================================
    // GYRO INTEGRATION
    // =========================================================

    void updateGyroIntegration(
        const DataFromAH127C &d,
        const rclcpp::Time &stamp
    )
    {
        const auto sample_time =
            protocol_->last_sample_time();

        if (!gyro_initialized_)
        {
            // Первое значение курса AHRS принимаем
            // за начальное положение.
            //
            // Далее сравниваем изменение курса
            // и изменение угла по гироскопу.

            integrated_yaw_deg_ =
                normalizeAngle(d.yaw);

            previous_gyro_rate_ = d.Z_rate;

            last_gyro_time_ = sample_time;

            gyro_initialized_ = true;
        }
        else
        {
            // Время между измерениями.
            //
            // Используем steady_clock вместо ROS-времени
            // для расчёта dt.

            const double dt =
                std::chrono::duration<double>(
                    sample_time - last_gyro_time_
                ).count();

            // Защита от больших пауз.
            //
            // После потери связи интегрирование
            // возобновляется с текущего курса,
            // чтобы диагностический угол
            // не получал произвольного скачка.

            if (dt > 0.0 && dt <= 0.1)
            {
                // Трапецеидальное интегрирование.
                //
                // Угловая скорость: град/с.
                // Время: секунды.
                // Результат: градусы.

                const double average_rate =
                    0.5 * (
                        previous_gyro_rate_ +
                        d.Z_rate
                    );

                integrated_yaw_deg_ +=
                    average_rate * dt;

                integrated_yaw_deg_ =
                    normalizeAngle(
                        integrated_yaw_deg_
                    );
            }
            else if (dt > 0.1)
            {
                integrated_yaw_deg_ =
                    normalizeAngle(d.yaw);
            }

            previous_gyro_rate_ = d.Z_rate;
            last_gyro_time_ = sample_time;
        }

        // =====================================================
        // СРАВНЕНИЕ AHRS И ГИРОСКОПА
        // =====================================================

        const double ahrs_yaw =
            normalizeAngle(d.yaw);

        const double gyro_yaw =
            normalizeAngle(
                integrated_yaw_deg_
            );

        const double error =
            normalizeAngle(
                ahrs_yaw - gyro_yaw
            );

        // =====================================================
        // PUBLISH DEBUG TOPIC
        // =====================================================
        //
        // vector.x = Yaw AHRS
        // vector.y = Yaw integrated from Gyro Z
        // vector.z = difference AHRS - Gyro
        //
        // Все значения в градусах.

        geometry_msgs::msg::Vector3Stamped msg;

        msg.header.stamp = stamp;
        msg.header.frame_id = frame_id_;

        msg.vector.x = ahrs_yaw;
        msg.vector.y = gyro_yaw;
        msg.vector.z = error;

        publisher_gyro_integrated_->publish(msg);

        // =====================================================
        // WARNING
        // =====================================================

        if (std::abs(error) > 15.0)
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,

                "AHRS/GYRO mismatch: "
                "AHRS=%.2f Gyro=%.2f Error=%.2f deg",

                ahrs_yaw,
                gyro_yaw,
                error
            );
        }
    }

private:

    // =========================================================
    // PARAMETERS
    // =========================================================

    std::string port_name_;
    std::string frame_id_;

    bool enable_diagnostics_ = true;

    // =========================================================
    // PROTOCOL
    // =========================================================

    std::unique_ptr<AH127Cprotocol> protocol_;

    // =========================================================
    // ROS2
    // =========================================================

    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<
        sensor_msgs::msg::Imu
    >::SharedPtr publisher_imu_;

    rclcpp::Publisher<
        geometry_msgs::msg::Vector3Stamped
    >::SharedPtr publisher_euler_;

    rclcpp::Publisher<
        geometry_msgs::msg::Vector3Stamped
    >::SharedPtr publisher_gyro_integrated_;

    rclcpp::Publisher<
        geometry_msgs::msg::Vector3Stamped
    >::SharedPtr publisher_magnetic_;

    // =========================================================
    // DIAGNOSTICS
    // =========================================================

    bool gyro_initialized_ = false;

    double integrated_yaw_deg_ = 0.0;

    double previous_gyro_rate_ = 0.0;

    std::chrono::steady_clock::time_point
        last_gyro_time_;
};


// ============================================================
// MAIN
// ============================================================

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    try
    {
        auto node =
            std::make_shared<AH127CPublisher>();

        rclcpp::spin(node);
    }
    catch (const std::exception &e)
    {
        RCLCPP_FATAL(
            rclcpp::get_logger("ah127c_main"),
            "Fatal error: %s",
            e.what()
        );

        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();

    return 0;
}
