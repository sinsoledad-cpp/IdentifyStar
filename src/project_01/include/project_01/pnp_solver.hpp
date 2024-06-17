#pragma once
#include<iostream>
#include<opencv4/opencv2/opencv.hpp>
#include "project_01/detector.hpp"

class PnPSolver
{
	//friend class Detector;
public:
	PnPSolver();

	bool MakeWorldPoints();
	bool MakeImagePoints(const Detector &detector);
	bool SolvePnP();
	bool SolvePoints();
	bool PnPSolverWork(const Detector &detector);
	
public:
	bool pnp_success_;
	cv::Point2f red_point_;

	cv::Mat rvec_;
	cv::Mat tvec_;

	cv::Mat camera_matrix_;
	cv::Mat dist_coeffs_;

	std::vector<cv::Point3f> world_3d_long_points_;
	std::vector<cv::Point3f> world_3d_short_points_;

	std::vector<cv::Point2f> image_2d_long_points_;
	std::vector<cv::Point2f> image_2d_short_points_;

	std::vector<cv::Point3f> result_3d_long_points_;
	std::vector<cv::Point3f> result_3d_short_points_;

};