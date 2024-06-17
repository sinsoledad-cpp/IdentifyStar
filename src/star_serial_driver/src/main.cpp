#include "star_serial_driver/star_serial_driver.hpp"

int main(int argc,const char** argv)
{
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<StarsSerialDriver>());
  rclcpp::shutdown();
  return 0;
}