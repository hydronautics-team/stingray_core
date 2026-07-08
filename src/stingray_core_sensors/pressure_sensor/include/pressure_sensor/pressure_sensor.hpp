#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>

#define DEFAULT_DUMP_PARAM 1.0

namespace stingray_core::pressure_sensor {

struct PressureSensorConfig{
    PressureSensorConfig(const rclcpp::Node::SharedPtr& node)
    : dump_param(node->declare_parameter<double>("dump_param", DEFAULT_DUMP_PARAM)),
      data_topic(node->declare_parameter<std::string>("data_topic", "/data_raw"))
    {}
    const double  dump_param;
    const std::string data_topic;
};

class PressureSensor {
   public:
    explicit PressureSensor(
        rclcpp::NodeOptions options = rclcpp::NodeOptions());

    void spin() { 
        rclcpp::spin(node_);
    }

    rclcpp::Logger get_logger() const { return node_->get_logger(); }

   private:
    void data_raw_callback(const std_msgs::msg::String::ConstSharedPtr& msg);
    void publish_depth(double depth);

    void check_depth(diagnostic_updater::DiagnosticStatusWrapper& stat);
    void check_frequency(diagnostic_updater::DiagnosticStatusWrapper& stat);
    void check_parse_errors(diagnostic_updater::DiagnosticStatusWrapper& stat);

    rclcpp::Node::SharedPtr node_;
    PressureSensorConfig config_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr depth_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr data_raw_sub_;

    std::shared_ptr<diagnostic_updater::Updater> updater_;
    std::shared_ptr<diagnostic_updater::HeaderlessTopicDiagnostic> freq_diag_;
    
    std::atomic<double> last_depth_{0.0};
    std::atomic<int> parse_errors_{0};
    std::atomic<int> total_messages_{0};
    double min_freq_ = 0.5;
    double max_freq_ = 100.0;
};

}  // namespace stingray_core::pressure_sensor
