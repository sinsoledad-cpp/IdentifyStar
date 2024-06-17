#include "project_01/detector.hpp"
Detector::Detector():red_point_(0.0,0.0),radius_(0.0)
{

}
/*
预处理图像
*/
bool Detector::Preprocessing(const cv::Mat &image)
{
	if (image.empty())return false;
	image_ = image;
	cv::Mat imagegray;
	//转灰度图
	cv::cvtColor(image_, imagegray, cv::COLOR_BGR2GRAY);
	//二值化
	cv::threshold(imagegray, image_thresh_, 80, 255, cv::THRESH_BINARY);
	//std::cout<<"Preprocessing!!!"<<std::endl;
	return true;
}
// bool Detector::Preprocessing(const cv::Mat &image)
// {
// 	if (image.empty())return false;
// 	image_ = image;
// 	cv::Mat imagegray;
// 	//转灰度图
// 	cv::cvtColor(image_, imagegray, cv::COLOR_BGR2GRAY);
// 	cv::Mat imagegau;
// 	cv::GaussianBlur(imagegray, imagegau, cv::Size(1, 1), 3, 0);
// 	//二值化
// 	cv::threshold(imagegau, image_thresh_, 80, 255, cv::THRESH_BINARY);
// 	//std::cout<<"Preprocessing!!!"<<std::endl;
// 	return true;
// }
cv::Mat Detector::PreprocessingToHSV(const cv::Mat& image)
{
	image_ = image;
	std::vector<cv::Mat> channels;
	cv::split(image, channels);
	cv::Mat redimage = channels.at(2);
	cv::Mat imagethre,imagedilate;
	cv::threshold(redimage, imagethre, 244, 255, 0);
	cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
	cv::dilate(imagethre, imagedilate, kernel);
	// cv::imshow("imagedilate", imagedilate);
	// cv::imshow("redimage", redimage);
	// cv::imshow("imagethre", imagethre);
	return imagedilate;
}
bool Detector::GetRedPoint(const cv::Mat& image)
{
	cv::Mat imagethre = PreprocessingToHSV(image);
	std::vector<std::vector<cv::Point>> contours;
	std::vector<cv::Vec4i> hierarchy;
	cv::findContours(imagethre, contours, hierarchy,
		cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
	cv::drawContours(image, contours, -1, cv::Scalar(255, 0, 0), 1);
	std::vector<std::vector<cv::Point>> conPoly(contours.size());
	std::vector<cv::Rect> boundRect(contours.size());

	for (size_t i = 0; i < contours.size(); i++)
	{
		double area = cv::contourArea(contours[i]);
		cv::Point2f center;
		float radius;
		cv::minEnclosingCircle(cv::Mat(contours[i]), center, radius);
		double circlearea = 3.1415926 * std::pow(radius, 2);
		// std::cout << area << "---" << circlearea << "---" << area / circlearea << std::endl;
		if (area / circlearea < 0.5)//外接圆面积比轮廓面积
		{
			continue;
		}
		cv::circle(image, center, 3, cv::Scalar(255, 0, 0), cv::FILLED);
		red_point_ = center;
	}
	return true;
}
/*
查找五角星轮廓
通过层次关系排除
对五角星轮廓拟合一个最小外接圆(得到圆心和半径)
对五角星轮廓进行多边形拟opencv合得到10个拐点
极坐标排序
依据拐点和外接圆圆心的相对关系
建立极坐标系,向右👉
注意,上方是负,下方是正
然后依据两个点在x和y上的偏量计算弧度角
*/
bool Detector::GetContours()
{
	std::vector<std::vector<cv::Point>> contours;
	std::vector<cv::Vec4i> hierarchy;

	cv::findContours(image_thresh_, contours, hierarchy,
		cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

	std::vector<std::vector<cv::Point>> conPoly(contours.size());
	//std::vector<cv::Rect> boundRect(contours.size());

	for (size_t i = 0; i < contours.size(); i++)
	{

		//面积排除
		//double area = cv::contourArea(contours[i]);
		// if (area < 50)continue;
		//层次关系排除
		if (hierarchy[i][2] == -1 )continue;
		if (hierarchy[i][3] == -1)continue;

		//判断子轮廓个数是否为6
		int cnt = 0;
		for (size_t j = hierarchy[i][2]; j + 1; j = hierarchy[j][0])cnt++;
		//std::cout << cnt << std::endl;
		if (cnt < 6)continue;

		//多边形拟合
		double length = cv::arcLength(contours[i], true);
		cv::approxPolyDP(contours[i], conPoly[i], 0.03 * length, true);
		if (conPoly[i].size() != 10)continue;//非十个拐点排除

		//拟合外接圆
		cv::minEnclosingCircle(cv::Mat(contours[i]), center_, radius_);

		//储存拐点
		if (!points_.empty())points_.clear();
		for (size_t j = 0; j < conPoly[i].size(); j++)
		{
			points_.emplace_back(conPoly[i][j]);
		}

		//依照极坐标排序
		std::sort(points_.begin(), points_.end(), [&](const cv::Point2f& p1, const cv::Point2f& p2) {
			return atan2(p1.y - center_.y, p1.x - center_.x) < atan2(p2.y - center_.y, p2.x - center_.x);
			});
		//return true;
	}
	//return false;
	return true;
}
/*
拐点分类
依据拐点到圆心的距离分成长短点集
*/
bool Detector::ClassifyPoint()
{

	if (points_.size() != 10)return false;
	shortpoints_.clear();
	longpoints_.clear();

	cv::Point2f firstpoint, secondpoint;
	bool oneflag = true, flag = true;
	for (size_t i = 0; i < points_.size(); i += 2)
	{
		firstpoint = points_[i];	//取出前面两个点
		secondpoint = points_[i + 1];
		if (oneflag)//因为下面代码只需要运行一次(为了确定flag),为了代码简洁,多定义了一个oneflag,
		{
			oneflag = false;
			flag = cv::norm(firstpoint - center_) > cv::norm(secondpoint - center_) ? true : false;
		}
		if (flag)//flag用于两种情况的开关,因为不清楚是第一个点离圆心远还第二个,对于后续,因为拟合多边形的点位是顺时针的所以一远一近交替排列
		{
			longpoints_.emplace_back(firstpoint);
			shortpoints_.emplace_back(secondpoint);
		}
		else
		{
			longpoints_.emplace_back(secondpoint);
			shortpoints_.emplace_back(firstpoint);
		}
	}
	if (!flag)//将最前面一个点放到最后面
	{
		cv::Point2f temp = shortpoints_.front();
		shortpoints_.erase(shortpoints_.begin());
		shortpoints_.emplace_back(temp);
	}
	if (!points_.empty())points_.clear();
	return true;
}
/*
探测器工作
*/
bool Detector::DetectWork(const cv::Mat &image)
{
	//std::cout<<"DetectWork"<<std::endl;
	if (!Preprocessing(image))return false;
	if(!GetRedPoint(image))return false;
	//std::cout<<"DetectWork1"<<std::endl;
	if (!GetContours())return false;
	//std::cout<<"DetectWork2"<<std::endl;
	if (!ClassifyPoint())return false;
	//std::cout<<"DetectWork3"<<std::endl;
	//std::cout << "hua" << std::endl;
	for (size_t i = 0; i < longpoints_.size(); i++)
	{
		cv::circle(image_, longpoints_[i], 5, cv::Scalar(250, 25, 0), 5);
		cv::putText(image_, std::to_string(i), longpoints_[i], cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0, 0, 200), 2);
	}
	for (size_t i = 0; i < shortpoints_.size(); i++)
	{
		cv::circle(image_, shortpoints_[i], 5, cv::Scalar(250, 25, 0), 5);
		cv::putText(image_, std::to_string(i + 5), shortpoints_[i], cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(200, 0, 0), 2);
	}
	// if(red_point_.x!=0&&red_point_.y!=0)
	// {
	// 	cv::putText(image_, "red_point: ("  + std::to_string(red_point_.x) + "," + std::to_string(red_point_.y) + ")",
	// 		cv::Point(30, 300), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(200, 0, 0), 2);
	// }
	//std::cout<<"DetectWork!!!"<<std::endl;
	return true;
}
