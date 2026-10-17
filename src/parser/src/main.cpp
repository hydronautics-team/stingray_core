#include <chrono>
#include <cmath>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"

#include "AH127Cprotocol.h"

using namespace std::chrono_literals;

namespace {

constexpr double G = 9.80665;
constexpr double DEG2RAD = M_PI / 180.0;

// "x,y,z" | "y,x,-z" | "-x,y,z" ... : одно и то же преобразование для accel, gyro, mag.
struct AxisMap {
    int idx[3] = {0, 1, 2};
    float sgn[3] = {1, 1, 1};

    bool parse(const std::string& s) {
        std::stringstream ss(s);
        std::string tok;
        int i = 0;
        while (std::getline(ss, tok, ',') && i < 3) {
            float sg = 1;
            size_t p = 0;
            if (!tok.empty() && (tok[0] == '-' || tok[0] == '+')) { sg = (tok[0] == '-') ? -1 : 1; p = 1; }
            if (p >= tok.size()) return false;
            char c = tok[p];
            int id = (c == 'x') ? 0 : (c == 'y') ? 1 : (c == 'z') ? 2 : -1;
            if (id < 0) return false;
            idx[i] = id; sgn[i] = sg; i++;
        }
        return i == 3;
    }
    void apply(float& x, float& y, float& z) const {
        float in[3] = {x, y, z};
        x = sgn[0] * in[idx[0]];
        y = sgn[1] * in[idx[1]];
        z = sgn[2] * in[idx[2]];
    }
};

} // namespace

class AH127CPublisher : public rclcpp::Node {
public:
    AH127CPublisher() : Node("ah127c_publisher") {
        port_     = declare_parameter<std::string>("port", "/dev/ttyUSB0");
        baud_     = declare_parameter<int>("baud", 9600);
        rate_hz_  = declare_parameter<int>("rate_hz", 10);        // 0,5,10,20,25,50
        save_     = declare_parameter<bool>("save_settings", false);
        frame_id_ = declare_parameter<std::string>("frame_id", "imu_link");
        debug_    = declare_parameter<bool>("debug", false);
        std::string axis = declare_parameter<std::string>("axis_map", "x,y,z");
        timeout_ms_ = declare_parameter<int>("timeout_ms", 2000);

        if (!axis_.parse(axis)) {
            RCLCPP_ERROR(get_logger(), "Bad axis_map '%s', using x,y,z", axis.c_str());
            axis_ = AxisMap();
        }

        // 57 байт * 10 бит * частота должно помещаться в скорость порта.
        double need = 57.0 * 10.0 * rate_hz_;
        if (rate_hz_ > 0 && need > baud_ * 0.9) {
            RCLCPP_WARN(get_logger(),
                "rate_hz=%d requires ~%.0f bit/s, but baud=%d. Frames will be corrupted. "
                "Use rate_hz<=10 at 9600 or set sensor baud to 115200.",
                rate_hz_, need, baud_);
        }

        pub_array_ = create_publisher<std_msgs::msg::Float32MultiArray>("imu_full_data", 10);
        pub_imu_   = create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);
        pub_mag_   = create_publisher<sensor_msgs::msg::MagneticField>("imu/mag", 10);

        try {
            proto_ = std::make_unique<AH127Cprotocol>(port_, baud_);
        } catch (const std::exception& e) {
            RCLCPP_FATAL(get_logger(), "Cannot open %s @ %d: %s", port_.c_str(), baud_, e.what());
            throw;
        }
        proto_->setDebug(debug_);
        if (!proto_->configure(rate_hz_, save_)) {
            RCLCPP_ERROR(get_logger(), "Unsupported rate_hz=%d (allowed: 0,5,10,20,25,50). Using 10.", rate_hz_);
            rate_hz_ = 10;
            proto_->configure(rate_hz_, save_);
        }

        // Опрос порта каждые 2 мс: публикуем только когда пришёл новый корректный кадр.
        timer_ = create_wall_timer(2ms, std::bind(&AH127CPublisher::onTimer, this));
        if (rate_hz_ == 0) {
            // Режим "по запросу": запрашиваем кадр сами 10 раз в секунду.
            req_timer_ = create_wall_timer(100ms, [this]() { proto_->requestFrame(); });
        }
        stats_timer_ = create_wall_timer(5s, std::bind(&AH127CPublisher::onStats, this));

        RCLCPP_INFO(get_logger(), "AH127C on %s @ %d, rate_hz=%d, axis_map=%s",
                    port_.c_str(), baud_, rate_hz_, axis.c_str());
    }

private:
    void onTimer() {
        int n = 0;
        try {
            n = proto_->poll();
        } catch (const std::exception& e) {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "Serial read error: %s", e.what());
            return;
        }
        if (n > 0) publish(proto_->latest());
        if (proto_->checkTimeout(timeout_ms_)) {
            RCLCPP_WARN(get_logger(), "No data for %d ms, re-sending config", timeout_ms_);
        }
    }

    void onStats() {
        RCLCPP_INFO(get_logger(), "frames_ok=%llu crc_errors=%llu bytes_dropped=%llu",
                    (unsigned long long)proto_->framesOk(),
                    (unsigned long long)proto_->crcErrors(),
                    (unsigned long long)proto_->bytesDropped());
    }

    void publish(ImuFrame f) {
        auto stamp = now();

        // Одинаковое преобразование осей для всех трёх векторов.
        axis_.apply(f.ax, f.ay, f.az);
        axis_.apply(f.gx, f.gy, f.gz);
        axis_.apply(f.mx, f.my, f.mz);

        // --- Float32MultiArray (совместимый формат, 16 значений) ---
        // [roll, pitch, yaw (град), ax, ay, az (м/с²), gx, gy, gz (град/с),
        //  mx, my, mz (Гаусс), q_x, q_y, q_z, q_w]
        std_msgs::msg::Float32MultiArray arr;
        arr.data = {
            f.roll, f.pitch, f.yaw,
            static_cast<float>(f.ax * G), static_cast<float>(f.ay * G), static_cast<float>(f.az * G),
            f.gx, f.gy, f.gz,
            f.mx, f.my, f.mz,
            f.q1, f.q2, f.q3, f.q0   // q0 у датчика - скаляр (w)
        };
        pub_array_->publish(arr);

        // --- sensor_msgs/Imu (СИ: м/с², рад/с) ---
        sensor_msgs::msg::Imu imu;
        imu.header.stamp = stamp;
        imu.header.frame_id = frame_id_;
        imu.orientation.x = f.q1;
        imu.orientation.y = f.q2;
        imu.orientation.z = f.q3;
        imu.orientation.w = f.q0;
        imu.angular_velocity.x = f.gx * DEG2RAD;
        imu.angular_velocity.y = f.gy * DEG2RAD;
        imu.angular_velocity.z = f.gz * DEG2RAD;
        imu.linear_acceleration.x = f.ax * G;
        imu.linear_acceleration.y = f.ay * G;
        imu.linear_acceleration.z = f.az * G;
        // Ковариации: ~1° по углам (паспорт), остальное грубо.
        const double var_ang = std::pow(1.0 * DEG2RAD, 2);
        imu.orientation_covariance = {var_ang, 0, 0, 0, var_ang, 0, 0, 0, var_ang};
        imu.angular_velocity_covariance = {0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01};
        imu.linear_acceleration_covariance = {0.05, 0, 0, 0, 0.05, 0, 0, 0, 0.05};
        pub_imu_->publish(imu);

        sensor_msgs::msg::MagneticField mag;
        mag.header = imu.header;
        mag.magnetic_field.x = f.mx * 1e-4;   // Гаусс -> Тесла
        mag.magnetic_field.y = f.my * 1e-4;
        mag.magnetic_field.z = f.mz * 1e-4;
        pub_mag_->publish(mag);

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
            "R:%.2f P:%.2f Y:%.2f | a:%.3f %.3f %.3f g | w:%.1f %.1f %.1f dps",
            f.roll, f.pitch, f.yaw, f.ax, f.ay, f.az, f.gx, f.gy, f.gz);
    }

    std::string port_, frame_id_;
    int baud_, rate_hz_, timeout_ms_;
    bool save_, debug_;
    AxisMap axis_;

    std::unique_ptr<AH127Cprotocol> proto_;
    rclcpp::TimerBase::SharedPtr timer_, req_timer_, stats_timer_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_array_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr pub_mag_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    int rc = 0;
    try {
        auto node = std::make_shared<AH127CPublisher>();
        rclcpp::spin(node);
    } catch (const std::exception&) {
        rc = 1;   // причина уже выведена в лог ноды
    }
    rclcpp::shutdown();
    return rc;
}
