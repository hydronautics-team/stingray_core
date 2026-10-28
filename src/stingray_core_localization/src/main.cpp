#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "stingray_core_localization/localization.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    const auto node = std::make_shared<stingray_core::localization::Localization>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
