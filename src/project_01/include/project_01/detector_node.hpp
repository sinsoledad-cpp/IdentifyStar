#pragma once
#include <iostream>
#include <memory>
#include <opencv4/opencv2/opencv.hpp>
#include "rclcpp/rclcpp.hpp"

//要包含这个头文件在install/base_interfaces_demo/include/base_interfaces_demo/msg/xxx.hpp
#include "base_interfaces_demo/msg/stars.hpp"

#include "project_01/detector.hpp"
#include "project_01/pnp_solver.hpp"

class DetectorNode: public rclcpp::Node
{
public:
    DetectorNode();
    
private:
    void TimerCallback();

public:
    cv::VideoCapture cap;

    std::shared_ptr<Detector> node_detector_;
    std::unique_ptr<PnPSolver> node_pnp_solver_;

    // rclcpp::TimerBase::SharedPtr timer_;
    base_interfaces_demo::msg::Star node_star_;
    base_interfaces_demo::msg::Stars node_stars_;
    rclcpp::Publisher<base_interfaces_demo::msg::Stars>::SharedPtr node_publisher_stars_;
};