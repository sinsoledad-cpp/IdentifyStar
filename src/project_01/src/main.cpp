#include "project_01/detector.hpp"
#include "project_01/pnp_solver.hpp"
#include "project_01/detector_node.hpp"

int main(int argc, char ** argv)
{
  //2.初始化ROS2客户端；
  rclcpp::init(argc,argv);
  //4.调用spin函数，传入自定义类对象指针；
  rclcpp::spin(std::make_shared<DetectorNode>());
  //5.释放资源
  rclcpp::shutdown();

  return 0;
}

