// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.

#ifndef RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_
#define RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_



#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <serial_driver/serial_driver.hpp>
#include <std_msgs/msg/float64.hpp>


// C++ system
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <mutex>

#include "base_interfaces_demo/msg/redpoint.hpp"

class RedPointSerialDriver : public rclcpp::Node
{
public:
  RedPointSerialDriver();

  ~RedPointSerialDriver();

private:
  void getParams();

  void sendData(base_interfaces_demo::msg::Redpoint::SharedPtr msg);

  void reopenPort();

  // Serial port
  //串行端口
  std::unique_ptr<IoContext> owned_ctx_;
  std::string device_name_;
  std::unique_ptr<drivers::serial_driver::SerialPortConfig> device_config_;
  std::unique_ptr<drivers::serial_driver::SerialDriver> serial_driver_;

  rclcpp::Subscription<base_interfaces_demo::msg::Redpoint>::SharedPtr red_point_subscription_;
  
};


#endif  // RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_
