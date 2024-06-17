#pragma once
#include<iostream>
#include<opencv4/opencv2/opencv.hpp>

class Detector
{
    //friend class PnP_Solver;
public:
    Detector();
    bool Preprocessing(const cv::Mat &image);
    cv::Mat PreprocessingToHSV(const cv::Mat& image);
    bool GetRedPoint(const cv::Mat& image);
    bool GetContours();
    bool ClassifyPoint();
    bool DetectWork(const cv::Mat &image);
public:
    cv::Mat image_;                                 //原图像
    cv::Mat image_thresh_;                          //黑白图
    cv::Point2f center_;                            //外接圆圆心(五角星中心)
    cv::Point2f red_point_;

    float radius_;                                  //外接圆半径

    std::vector<cv::Point2f> points_;			    //五角星拐点
    std::vector<cv::Point2f> shortpoints_;		    //离中心近的拐点
    std::vector<cv::Point2f> longpoints_;		    //离中心远的拐点
    std::vector<cv::Point2f> resultpoints_;	        //结果
};