#pragma once
#include <iostream>
#include <memory>
#include "rclcpp/rclcpp.hpp"

//要包含这个头文件在install/base_interfaces_demo/include/base_interfaces_demo/msg/xxx.hpp
#include "base_interfaces_demo/msg/stars.hpp"


class MyListener:public rclcpp::Node
{
public:
    MyListener();
    void my_callback(const base_interfaces_demo::msg::Stars stars);
public:
    rclcpp::Subscription<base_interfaces_demo::msg::Stars>::SharedPtr node_subscription_stars_;


};