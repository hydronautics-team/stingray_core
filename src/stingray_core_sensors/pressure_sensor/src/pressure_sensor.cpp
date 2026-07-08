#include <exception>
#include <pressure_sensor/pressure_sensor.hpp>
#include <rclcpp/rclcpp.hpp>

namespace stingray_core::pressure_sensor
{

PressureSensor::PressureSensor(rclcpp::NodeOptions options)
    : node_(rclcpp::Node::make_shared("pressure_sensor", std::move(options))),
      config_(node_),
      depth_pub_(node_->create_publisher<std_msgs::msg::Float64>("depth", rclcpp::SensorDataQoS())),
      data_raw_sub_(node_->create_subscription<std_msgs::msg::String>(config_.data_topic, rclcpp::SensorDataQoS(),
                                                                      [this](const std_msgs::msg::String::ConstSharedPtr &msg)
                                                                      { this->data_raw_callback(msg); }))
{
    updater_ = std::make_shared<diagnostic_updater::Updater>(node_);
    updater_->setHardwareID("pressure_sensor_01");

    updater_->add("Depth Values", this, &PressureSensor::check_depth);
    updater_->add("Data Frequency", this, &PressureSensor::check_frequency);
    updater_->add("Parse Errors", this, &PressureSensor::check_parse_errors);

    freq_diag_ = std::make_shared<diagnostic_updater::HeaderlessTopicDiagnostic>(
        "depth", *updater_, diagnostic_updater::FrequencyStatusParam(&min_freq_, &max_freq_, 0.1, 10));

    RCLCPP_INFO(node_->get_logger(), "Pressure sensor node initialized");
    RCLCPP_INFO(node_->get_logger(), "dump_param: %.3f", config_.dump_param);
    RCLCPP_INFO(node_->get_logger(), "data_topic: %s", config_.data_topic.c_str());
}

void PressureSensor::data_raw_callback(const std_msgs::msg::String::ConstSharedPtr &msg)
{
    total_messages_++;
    try
    {
        double depth = std::stod(msg->data) * config_.dump_param / 10.0;
        if (depth > 1000)
        {
            depth = 0;
        }
        last_depth_ = depth;
        publish_depth(depth);
        freq_diag_->tick();
    }
    catch (const std::exception &e)
    {
        parse_errors_++;
        RCLCPP_WARN(node_->get_logger(), "Can't parse pressure payload '%s': %s", msg->data.c_str(), e.what());
    }
}

void PressureSensor::publish_depth(double depth)
{
    auto depth_msg = std_msgs::msg::Float64();
    depth_msg.data = depth;
    RCLCPP_DEBUG(node_->get_logger(), "Published depth: %.3f m", depth);
    depth_pub_->publish(std::move(depth_msg));
}

void PressureSensor::check_depth(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
    double depth = last_depth_.load();

    if (depth < 0)
    {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Negative depth value detected");
    }
    else if (depth > 300)
    {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Depth value unusually high");
    }
    else
    {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Depth readings normal");
    }

    stat.add("Current Depth", std::to_string(depth) + " m");
    stat.add("Total Messages", std::to_string(total_messages_.load()));
}

void PressureSensor::check_frequency(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
    int total = total_messages_.load();

    if (total == 0)
    {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "No data received yet");
    }
    else
    {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Data frequency within expected range");
    }

    stat.add("Total Messages Received", std::to_string(total));
    stat.add("Expected Frequency", std::to_string(min_freq_) + " - " + std::to_string(max_freq_) + " Hz");
}

void PressureSensor::check_parse_errors(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
    int errors = parse_errors_.load();
    int total = total_messages_.load();

    double error_rate = total > 0 ? (double)errors / total * 100.0 : 0.0;

    if (errors == 0)
    {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "No parse errors");
    }
    else if (error_rate < 5.0)
    {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Some parse errors detected");
    }
    else
    {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "High parse error rate");
    }

    stat.add("Parse Errors", std::to_string(errors));
    stat.add("Error Rate", std::to_string(error_rate) + " %");
    stat.add("Total Messages", std::to_string(total));
}

} // namespace stingray_core::pressure_sensor
