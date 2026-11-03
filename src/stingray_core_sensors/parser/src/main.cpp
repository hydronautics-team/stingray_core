#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#include "AH127Cprotocol.h"

using namespace std::chrono_literals;


// ============================================================
// AH127C ROS2 PUBLISHER
// ============================================================

class AH127CPublisher : public rclcpp::Node
{
public:

    AH127CPublisher()
        : Node("ah127c_publisher")
    {
        // ====================================================
        // PARAMETERS
        // ====================================================

        port_name_ =
            this->declare_parameter<std::string>(
                "port",
                "/dev/imu"
            );

        frame_id_ =
            this->declare_parameter<std::string>(
                "frame_id",
                "imu_link"
            );

        enable_diagnostics_ =
            this->declare_parameter<bool>(
                "enable_diagnostics",
                true
            );

        orientation_source_ =
            this->declare_parameter<std::string>(
                "orientation_source",
                "mahony"
            );

        mahony_kp_ =
            this->declare_parameter<double>(
                "mahony_kp",
                1.5
            );

        accel_rejection_threshold_ =
            this->declare_parameter<double>(
                "accel_rejection_threshold",
                2.0
            );

        gyro_bias_calibration_sec_ =
            this->declare_parameter<double>(
                "gyro_bias_calibration_sec",
                3.0
            );

        if (orientation_source_ != "mahony" &&
            orientation_source_ != "sensor")
        {
            throw std::invalid_argument(
                "orientation_source must be 'mahony' or 'sensor'"
            );
        }

        // ====================================================
        // PUBLISHERS
        // ====================================================

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

        publisher_gyro_integrated_ =
            this->create_publisher<
                geometry_msgs::msg::Vector3Stamped
            >(
                "/imu_gyro_integrated",
                10
            );

        publisher_magnetic_ =
            this->create_publisher<
                geometry_msgs::msg::Vector3Stamped
            >(
                "/imu_magnetic_field",
                10
            );

        // ====================================================
        // PROTOCOL
        // ====================================================

        protocol_ =
            std::make_unique<AH127Cprotocol>(
                port_name_,
                115200
            );

        // ====================================================
        // TIMER
        // ====================================================
        //
        // Проверяем наличие новых данных каждые 5 мс.
        //
        // Датчик выдаёт данные с частотой 50 Гц.
        //
        // Сообщения публикуются только при появлении
        // новых корректных полных кадров.

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

        RCLCPP_INFO(
            this->get_logger(),
            "Orientation source: %s",
            orientation_source_.c_str()
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

    // ========================================================
    // CONSTANTS
    // ========================================================

    static constexpr double PI =
        3.14159265358979323846;

    static constexpr double DEG_TO_RAD =
        PI / 180.0;

    static constexpr double RAD_TO_DEG =
        180.0 / PI;

    static constexpr double GRAVITY =
        9.80665;


    // ========================================================
    // NORMALIZE ANGLE
    // ========================================================

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


    // ========================================================
    // MAHONY IMU FILTER (GYROSCOPE + ACCELEROMETER)
    // ========================================================

    tf2::Quaternion updateOrientation(
        const DataFromAH127C &d
    )
    {
        const auto sample_time = protocol_->last_sample_time();

        if (!orientation_initialized_)
        {
            orientation_q_.setRPY(
                d.roll * DEG_TO_RAD,
                d.pitch * DEG_TO_RAD,
                d.yaw * DEG_TO_RAD
            );
            orientation_q_.normalize();
            last_orientation_time_ = sample_time;
            bias_calibration_start_ = sample_time;
            orientation_initialized_ = true;
            return orientation_q_;
        }

        const double dt = std::chrono::duration<double>(
            sample_time - last_orientation_time_
        ).count();
        last_orientation_time_ = sample_time;

        if (dt <= 0.0 || dt > 0.1)
        {
            return orientation_q_;
        }

        const double raw_gx = d.X_rate * DEG_TO_RAD;
        const double raw_gy = d.Y_rate * DEG_TO_RAD;
        const double raw_gz = d.Z_rate * DEG_TO_RAD;
        const double gyro_norm_deg = std::sqrt(
            d.X_rate * d.X_rate +
            d.Y_rate * d.Y_rate +
            d.Z_rate * d.Z_rate
        );

        const double calibration_elapsed =
            std::chrono::duration<double>(
                sample_time - bias_calibration_start_
            ).count();

        // Bias is estimated only from samples where the vehicle is still.
        if (!gyro_bias_ready_ && gyro_norm_deg < 5.0)
        {
            gyro_bias_sum_x_ += raw_gx;
            gyro_bias_sum_y_ += raw_gy;
            gyro_bias_sum_z_ += raw_gz;
            ++gyro_bias_samples_;
        }

        if (!gyro_bias_ready_ &&
            calibration_elapsed >= gyro_bias_calibration_sec_)
        {
            if (gyro_bias_samples_ > 0)
            {
                gyro_bias_x_ = gyro_bias_sum_x_ / gyro_bias_samples_;
                gyro_bias_y_ = gyro_bias_sum_y_ / gyro_bias_samples_;
                gyro_bias_z_ = gyro_bias_sum_z_ / gyro_bias_samples_;
            }
            gyro_bias_ready_ = true;
            RCLCPP_INFO(
                this->get_logger(),
                "Gyro bias: x=%.4f y=%.4f z=%.4f deg/s (%lu samples)",
                gyro_bias_x_ * RAD_TO_DEG,
                gyro_bias_y_ * RAD_TO_DEG,
                gyro_bias_z_ * RAD_TO_DEG,
                static_cast<unsigned long>(gyro_bias_samples_)
            );
        }

        double gx = raw_gx - gyro_bias_x_;
        double gy = raw_gy - gyro_bias_y_;
        double gz = raw_gz - gyro_bias_z_;

        const double accel_norm = std::sqrt(
            d.X_accel * d.X_accel +
            d.Y_accel * d.Y_accel +
            d.Z_accel * d.Z_accel
        );

        // Reject acceleration feedback during translation or strong vibration.
        if (accel_norm > 1e-6 &&
            std::abs(accel_norm - GRAVITY) <= accel_rejection_threshold_)
        {
            const double ax = d.X_accel / accel_norm;
            const double ay = d.Y_accel / accel_norm;
            const double az = d.Z_accel / accel_norm;

            const double qw = orientation_q_.w();
            const double qx = orientation_q_.x();
            const double qy = orientation_q_.y();
            const double qz = orientation_q_.z();

            // Expected gravity direction in the sensor frame.
            const double vx = 2.0 * (qx * qz - qw * qy);
            const double vy = 2.0 * (qw * qx + qy * qz);
            const double vz = qw * qw - qx * qx - qy * qy + qz * qz;

            // measured gravity x expected gravity
            gx += mahony_kp_ * (ay * vz - az * vy);
            gy += mahony_kp_ * (az * vx - ax * vz);
            gz += mahony_kp_ * (ax * vy - ay * vx);
        }

        const double qw = orientation_q_.w();
        const double qx = orientation_q_.x();
        const double qy = orientation_q_.y();
        const double qz = orientation_q_.z();
        const double half_dt = 0.5 * dt;

        orientation_q_.setValue(
            qx + half_dt * ( qw * gx + qy * gz - qz * gy),
            qy + half_dt * ( qw * gy - qx * gz + qz * gx),
            qz + half_dt * ( qw * gz + qx * gy - qy * gx),
            qw + half_dt * (-qx * gx - qy * gy - qz * gz)
        );
        orientation_q_.normalize();
        return orientation_q_;
    }


    // ========================================================
    // GYROSCOPE INTEGRATION
    // ========================================================
    //
    // Рассчитываем курс по угловой скорости Z.
    //
    // ВАЖНО:
    //
    // Это тот же алгоритм, который ранее выдавал
    // правильный курс в:
    //
    // /imu_gyro_integrated/vector/y
    //
    // Теперь этот угол используется как основной Yaw.
    //
    // Метод рассчитан на движение преимущественно
    // вокруг оси Z при небольших Roll и Pitch.

    void updateGyroIntegration(
        const DataFromAH127C &d
    )
    {
        const auto sample_time =
            protocol_->last_sample_time();

        // ====================================================
        // FIRST SAMPLE
        // ====================================================

        if (!gyro_initialized_)
        {
            // Начальный курс берём из AH127C.
            //
            // Это сохраняет исходное направление.
            //
            // Далее готовый Yaw AH127C больше
            // не влияет на интегрирование.

            integrated_yaw_deg_ =
                normalizeAngle(d.yaw);

            previous_gyro_rate_ =
                d.Z_rate;

            last_gyro_time_ =
                sample_time;

            gyro_initialized_ = true;

            return;
        }

        // ====================================================
        // DELTA TIME
        // ====================================================

        const double dt =
            std::chrono::duration<double>(
                sample_time - last_gyro_time_
            ).count();

        // ====================================================
        // VALIDATION
        // ====================================================

        if (dt <= 0.0)
        {
            return;
        }

        // После длительной паузы не интегрируем
        // неизвестный промежуток движения.
        //
        // При этом сохраняем предыдущий курс.
        //
        // НЕ сбрасываем его на d.yaw.

        if (dt > 0.1)
        {
            previous_gyro_rate_ =
                d.Z_rate;

            last_gyro_time_ =
                sample_time;

            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,

                "Gyro integration gap: %.3f s",
                dt
            );

            return;
        }

        // ====================================================
        // TRAPEZOIDAL INTEGRATION
        // ====================================================
        //
        // Угловые скорости в град/с.
        // Время в секундах.
        //
        // Формула:
        //
        // Yaw += (omega_prev + omega_now) / 2 * dt

        const double average_rate =
            0.5 * (
                previous_gyro_rate_ +
                d.Z_rate
            );

        integrated_yaw_deg_ +=
            average_rate * dt;

        // ====================================================
        // NORMALIZATION
        // ====================================================

        integrated_yaw_deg_ =
            normalizeAngle(
                integrated_yaw_deg_
            );

        // ====================================================
        // SAVE PREVIOUS DATA
        // ====================================================

        previous_gyro_rate_ =
            d.Z_rate;

        last_gyro_time_ =
            sample_time;
    }


    // ========================================================
    // PUBLISH GYRO DIAGNOSTICS
    // ========================================================

    void publishGyroDiagnostics(
        const DataFromAH127C &d,
        const rclcpp::Time &stamp
    )
    {
        if (!enable_diagnostics_)
        {
            return;
        }

        // Сырой курс AH127C.
        const double ahrs_yaw =
            normalizeAngle(d.yaw);

        // Курс по гироскопу.
        const double gyro_yaw =
            normalizeAngle(
                integrated_yaw_deg_
            );

        // Разница.
        const double error =
            normalizeAngle(
                ahrs_yaw - gyro_yaw
            );

        geometry_msgs::msg::Vector3Stamped msg;

        msg.header.stamp = stamp;
        msg.header.frame_id = frame_id_;

        // X = старый курс AH127C.
        msg.vector.x = ahrs_yaw;

        // Y = новый правильный курс по гироскопу.
        msg.vector.y = gyro_yaw;

        // Z = разница.
        msg.vector.z = error;

        publisher_gyro_integrated_->publish(msg);

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


    // ========================================================
    // MAIN TIMER CALLBACK
    // ========================================================

    void timer_callback()
    {
        try
        {
            // =================================================
            // READ DATA
            // =================================================

            const bool new_data =
                protocol_->readData();

            protocol_->timeoutSlot();

            if (!new_data)
            {
                return;
            }

            // =================================================
            // TIME
            // =================================================

            const auto stamp =
                this->now();

            // =================================================
            // SENSOR DATA
            // =================================================

            const DataFromAH127C &d =
                protocol_->data;

            // =================================================
            // CALCULATE CORRECTED YAW
            // =================================================
            //
            // Интегрирование выполняется ДО публикации.
            //
            // Это гарантирует, что все топики содержат
            // новый курс текущего измерения.

            updateGyroIntegration(d);

            // =================================================
            // MAIN YAW
            // =================================================
            //
            // Главная переменная!
            //
            // Вместо d.yaw используем угол гироскопа.

            const double corrected_yaw =
                integrated_yaw_deg_;

            tf2::Quaternion q;

            if (orientation_source_ == "mahony")
            {
                q = updateOrientation(d);
            }
            else
            {
                q.setRPY(
                    d.roll * DEG_TO_RAD,
                    d.pitch * DEG_TO_RAD,
                    corrected_yaw * DEG_TO_RAD
                );
                q.normalize();
            }

            double output_roll_rad = 0.0;
            double output_pitch_rad = 0.0;
            double output_yaw_rad = 0.0;
            tf2::Matrix3x3(q).getRPY(
                output_roll_rad,
                output_pitch_rad,
                output_yaw_rad
            );

            // =================================================
            // 1. SENSOR_MSGS / IMU
            // =================================================

            sensor_msgs::msg::Imu imu_msg;

            imu_msg.header.stamp = stamp;
            imu_msg.header.frame_id = frame_id_;

            imu_msg.orientation.x = q.x();
            imu_msg.orientation.y = q.y();
            imu_msg.orientation.z = q.z();
            imu_msg.orientation.w = q.w();

            // =================================================
            // ANGULAR VELOCITY
            // =================================================

            imu_msg.angular_velocity.x =
                d.X_rate * DEG_TO_RAD;

            imu_msg.angular_velocity.y =
                d.Y_rate * DEG_TO_RAD;

            imu_msg.angular_velocity.z =
                d.Z_rate * DEG_TO_RAD;

            // =================================================
            // LINEAR ACCELERATION
            // =================================================

            imu_msg.linear_acceleration.x =
                d.X_accel;

            imu_msg.linear_acceleration.y =
                d.Y_accel;

            imu_msg.linear_acceleration.z =
                d.Z_accel;

            // =================================================
            // COVARIANCE
            // =================================================
            //
            // Initial conservative estimates in SI units. These avoid an
            // overconfident EKF and must be refined from stationary logs.
            // Orientation: rad^2 (yaw is less certain due to heading drift).
            imu_msg.orientation_covariance = {
                0.01, 0.0, 0.0,
                0.0, 0.01, 0.0,
                0.0, 0.0, 0.04
            };

            // Angular velocity: (rad/s)^2.
            imu_msg.angular_velocity_covariance = {
                0.0025, 0.0, 0.0,
                0.0, 0.0025, 0.0,
                0.0, 0.0, 0.0025
            };

            // Linear acceleration: (m/s^2)^2.
            imu_msg.linear_acceleration_covariance = {
                0.04, 0.0, 0.0,
                0.0, 0.04, 0.0,
                0.0, 0.0, 0.04
            };

            // =================================================
            // PUBLISH IMU
            // =================================================

            publisher_imu_->publish(imu_msg);

            // =================================================
            // 2. EULER ANGLES
            // =================================================

            geometry_msgs::msg::Vector3Stamped euler_msg;

            euler_msg.header.stamp = stamp;
            euler_msg.header.frame_id = frame_id_;

            // Euler angles are derived from the exact quaternion published
            // above, so both topics always describe the same orientation.
            euler_msg.vector.x = output_roll_rad * RAD_TO_DEG;
            euler_msg.vector.y = output_pitch_rad * RAD_TO_DEG;
            euler_msg.vector.z = output_yaw_rad * RAD_TO_DEG;

            publisher_euler_->publish(euler_msg);

            // =================================================
            // 3. MAGNETOMETER
            // =================================================

            geometry_msgs::msg::Vector3Stamped magn_msg;

            magn_msg.header.stamp = stamp;
            magn_msg.header.frame_id = frame_id_;

            magn_msg.vector.x = d.X_magn;
            magn_msg.vector.y = d.Y_magn;
            magn_msg.vector.z = d.Z_magn;

            publisher_magnetic_->publish(magn_msg);

            // =================================================
            // 4. GYRO DIAGNOSTICS
            // =================================================

            publishGyroDiagnostics(
                d,
                stamp
            );

            // =================================================
            // 5. LOG
            // =================================================

            RCLCPP_INFO_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                200,

                "IMU: Roll=%7.2f Pitch=%7.2f "
                "Yaw_Filter=%7.2f Yaw_AH127C=%7.2f "
                "GyroZ=%7.2f deg/s",

                euler_msg.vector.x,
                euler_msg.vector.y,
                euler_msg.vector.z,
                d.yaw,
                d.Z_rate
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


private:

    // ========================================================
    // PARAMETERS
    // ========================================================

    std::string port_name_;
    std::string frame_id_;
    std::string orientation_source_;

    bool enable_diagnostics_ = true;
    double mahony_kp_ = 1.5;
    double accel_rejection_threshold_ = 2.0;
    double gyro_bias_calibration_sec_ = 3.0;

    // ========================================================
    // PROTOCOL
    // ========================================================

    std::unique_ptr<AH127Cprotocol> protocol_;

    // ========================================================
    // TIMER
    // ========================================================

    rclcpp::TimerBase::SharedPtr timer_;

    // ========================================================
    // PUBLISHERS
    // ========================================================

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

    // ========================================================
    // GYRO INTEGRATION
    // ========================================================

    bool gyro_initialized_ = false;

    double integrated_yaw_deg_ = 0.0;

    double previous_gyro_rate_ = 0.0;

    std::chrono::steady_clock::time_point
        last_gyro_time_;

    // ========================================================
    // ORIENTATION FILTER
    // ========================================================

    bool orientation_initialized_ = false;
    bool gyro_bias_ready_ = false;
    tf2::Quaternion orientation_q_{0.0, 0.0, 0.0, 1.0};
    std::chrono::steady_clock::time_point last_orientation_time_;
    std::chrono::steady_clock::time_point bias_calibration_start_;
    std::size_t gyro_bias_samples_ = 0;
    double gyro_bias_sum_x_ = 0.0;
    double gyro_bias_sum_y_ = 0.0;
    double gyro_bias_sum_z_ = 0.0;
    double gyro_bias_x_ = 0.0;
    double gyro_bias_y_ = 0.0;
    double gyro_bias_z_ = 0.0;
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
