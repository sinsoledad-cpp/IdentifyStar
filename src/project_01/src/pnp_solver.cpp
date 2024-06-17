#include "project_01/detector.hpp"
#include "project_01/pnp_solver.hpp"

PnPSolver::PnPSolver()
{
	//std::cout<<"PnPSolver"<<std::endl;
	camera_matrix_ = (cv::Mat_<double>(3, 3) <<
        759.964380,0.00000,308.629642,
        0.000000, 759.071714, 185.190048,
        0.000000, 0.000000, 1.000000);  

	dist_coeffs_ = (cv::Mat_<double>(5, 1) << -0.507965, 0.262994, 0.000012, -0.000443, 0.000000);

	if(!MakeWorldPoints())std::cout<<"PnPSolver" << std::endl;
}

bool PnPSolver::MakeWorldPoints()
{
	world_3d_long_points_.clear();
	world_3d_short_points_.clear();

	world_3d_long_points_.emplace_back(cv::Point3f(-0.091, 0.032, 0));
	world_3d_long_points_.emplace_back(cv::Point3f(0, 0.096	, 0));
	world_3d_long_points_.emplace_back(cv::Point3f(0.091, 0.032, 0));
	world_3d_long_points_.emplace_back(cv::Point3f(0.0575, -0.075, 0));
	world_3d_long_points_.emplace_back(cv::Point3f(-0.0575, -0.075, 0));

	world_3d_short_points_.emplace_back(cv::Point3f(-0.0315, 0.046, 0));
	world_3d_short_points_.emplace_back(cv::Point3f(0.0315, 0.046, 0));
	world_3d_short_points_.emplace_back(cv::Point3f(0.053, -0.014, 0));
	world_3d_short_points_.emplace_back(cv::Point3f(0, -0.054, 0));
	world_3d_short_points_.emplace_back(cv::Point3f(-0.053, -0.014, 0));
	return true;
}

bool PnPSolver::MakeImagePoints(const Detector& detector)
{
	//std::cout<<"MakeImagePoints"<<std::endl;
	image_2d_long_points_.clear();
	//image_2d_short_points_.clear();
	if (detector.longpoints_.size() != 5 || detector.shortpoints_.size() != 5)return false;

	for (size_t i = 0; i < 5; i++)
	{
		image_2d_long_points_.emplace_back(detector.longpoints_[i]);
	}
	red_point_=detector.red_point_;
	return true;
}

bool PnPSolver::SolvePnP()
{
	if (world_3d_long_points_.size() != 5 || world_3d_short_points_.size() != 5 
		||image_2d_long_points_.size() != 5)return false;
	bool success = cv::solvePnP(world_3d_long_points_, image_2d_long_points_, camera_matrix_, dist_coeffs_, rvec_, tvec_, false, cv::SOLVEPNP_IPPE);
	//pnp_success_=success;
	return success;
}

bool PnPSolver::SolvePoints()
{
	if (rvec_.empty())return false;
	result_3d_long_points_.clear();
	result_3d_short_points_.clear();
	cv::Mat rotation_matrix;
	cv::Rodrigues(rvec_, rotation_matrix);
	//rotation_matrix=rotation_matrix.inv();
	for (size_t i = 0; i < 5; i++)
	{
		float x = rotation_matrix.at<double>(0, 0) * world_3d_long_points_[i].x
			+ rotation_matrix.at<double>(0, 1) * world_3d_long_points_[i].y
			+ rotation_matrix.at<double>(0, 2) * world_3d_long_points_[i].z + tvec_.at<double>(0);
		float y = rotation_matrix.at<double>(1, 0) * world_3d_long_points_[i].x
			+ rotation_matrix.at<double>(1, 1) * world_3d_long_points_[i].y
			+ rotation_matrix.at<double>(1, 2) * world_3d_long_points_[i].z + tvec_.at<double>(1);
		float z = rotation_matrix.at<double>(2, 0) * world_3d_long_points_[i].x
			+ rotation_matrix.at<double>(2, 1) * world_3d_long_points_[i].y
			+ rotation_matrix.at<double>(2, 2) * world_3d_long_points_[i].z + tvec_.at<double>(2);
		result_3d_long_points_.emplace_back(cv::Point3f(x, y, z));
	}
	for (size_t i = 0; i < 5; i++)
	{
		float x = rotation_matrix.at<double>(0, 0) * world_3d_short_points_[i].x
			+ rotation_matrix.at<double>(0, 1) * world_3d_short_points_[i].y
			+ rotation_matrix.at<double>(0, 2) * world_3d_short_points_[i].z + tvec_.at<double>(0);
		float y = rotation_matrix.at<double>(1, 0) * world_3d_short_points_[i].x
			+ rotation_matrix.at<double>(1, 1) * world_3d_short_points_[i].y
			+ rotation_matrix.at<double>(1, 2) * world_3d_short_points_[i].z + tvec_.at<double>(1);
		float z = rotation_matrix.at<double>(2, 0) * world_3d_short_points_[i].x
			+ rotation_matrix.at<double>(2, 1) * world_3d_short_points_[i].y
			+ rotation_matrix.at<double>(2, 2) * world_3d_short_points_[i].z + tvec_.at<double>(2);
		result_3d_short_points_.emplace_back(cv::Point3f(x, y, z));
	}
	// red_point_.x = rotation_matrix.at<double>(0, 0) * red_point_.x
	// 	+ rotation_matrix.at<double>(0, 1) * red_point_.y
	// 	+ rotation_matrix.at<double>(0, 2) * 0 + tvec_.at<double>(0);
	// red_point_.y = rotation_matrix.at<double>(1, 0) * red_point_.x
	// 	+ rotation_matrix.at<double>(1, 1) * red_point_.y
	// 	+ rotation_matrix.at<double>(1, 2) * 0 + tvec_.at<double>(1);
	return true;
}

bool PnPSolver::PnPSolverWork(const Detector& detector)
{
	MakeImagePoints(detector);
	if (!SolvePnP())
	{
		std::cout << "SolverPnP" << std::endl;
		return false;
	}
	if (rvec_.empty() || tvec_.empty())return false;
	if (!SolvePoints())return false;
	cv::putText(detector.image_,std::to_string(cv::norm(tvec_)),cv::Point2f(50,50), cv::FONT_HERSHEY_PLAIN, 3, cv::Scalar(0, 69, 255), 1.5);
	//std::cout << "result" << std::endl;
	// for (auto a : result_3d_long_points_)
	// {
	// 	std::cout << a << std::endl;
	// }

	//std::cout << cv::norm(tvec_) << std::endl;
	return true;
}