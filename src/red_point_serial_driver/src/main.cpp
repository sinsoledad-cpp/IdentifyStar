#include "red_point_serial_driver/red_point_serial_driver.hpp"

int main(int argc,const char** argv)
{
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<RedPointSerialDriver>());
  rclcpp::shutdown();
  return 0;
}
