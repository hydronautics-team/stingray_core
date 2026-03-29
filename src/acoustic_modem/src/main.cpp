#include <acoustic_modem/acoustic_modem.hpp>
#include <rclcpp/rclcpp.hpp>

// int main(int argc, char *argv[])
// {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<stingray_core::acoustic_modem::AcousticModem>());
//     rclcpp::shutdown();
//     return 0;
// }

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto acoustic_modem_node = std::make_shared<stingray_core::acoustic_modem::AcousticModem>();
    acoustic_modem_node->spin();
    rclcpp::shutdown();
    return 0;
}