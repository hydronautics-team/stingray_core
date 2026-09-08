#include <rclcpp/rclcpp.hpp>

#include <stingray_core_localization/localization.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto sensor_node = std::make_shared<stingray_core::localization::Localization>();
    sensor_node->spin();
    rclcpp::shutdown();
    return 0;
}
