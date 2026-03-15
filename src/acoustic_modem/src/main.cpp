#include <acoustic_modem/acoustic_modem.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<stingray_core::acoustic_modem::AcousticModem>());
  rclcpp::shutdown();
  return 0;
}