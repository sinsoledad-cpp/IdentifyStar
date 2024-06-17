#include "my_test/my_test_listener.hpp"


int main(int argc, char ** argv)
{
  //2.初始化ROS2客户端；
  rclcpp::init(argc,argv);
  //4.调用spin函数，传入自定义类对象指针；
  rclcpp::spin(std::make_shared<MyListener>());
  //5.释放资源
  rclcpp::shutdown();

  return 0;
}
