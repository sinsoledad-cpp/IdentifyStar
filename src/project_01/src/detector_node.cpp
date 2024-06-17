#include "project_01/detector_node.hpp"


DetectorNode::DetectorNode():Node("Detector_Node","Stars")
{
    RCLCPP_INFO(this->get_logger(),"发布节点Detector_Node创建！");
    // cap.open(0);

    /*
    曝光值 (cv::CAP_PROP_EXPOSURE):

    范围: 通常为负数到正数，负数表示较暗的曝光，正数表示较亮的曝光。
    效果: 较低的值会导致暗曝光，而较高的值会导致过曝光。负值使图像变暗，正值使图像变亮。
    亮度 (cv::CAP_PROP_BRIGHTNESS):

    范围: 通常为0到255，其中0为最暗，255为最亮。
    效果: 较低的值使图像变暗，而较高的值使图像变亮。
    色调 (cv::CAP_PROP_HUE):

    范围: 通常为0到180，代表颜色的角度。
    效果: 控制图像的色彩，0表示红色，30表示黄色，60表示绿色，以此类推。
    饱和度 (cv::CAP_PROP_SATURATION):

    范围: 通常为0到255，其中0表示灰度图像，255表示完全饱和的图像。
    效果: 较低的值会导致图像变得灰暗和失真，而较高的值会增加图像的颜色饱和度。
    */
    cap.open("/dev/video0");
    cap.set(cv::CAP_PROP_AUTO_EXPOSURE,0.25);           // 设置自动曝光的属性标识符，值为0.25表示将自动曝光设置为手动模式，并且将曝光值设置为0.25
    cap.set(cv::CAP_PROP_EXPOSURE,-1);                  // 设置曝光值的属性标识符，值为-1表示该参数的值将由摄像头驱动程序自动确定。这意味着曝光值将由摄像头自动调整，而不是手动设置。
                                                        // 负数表示由摄像头驱动程序自动确定曝光值。非负数表示手动设置的曝光值。
    cap.set(cv::CAP_PROP_BRIGHTNESS, 170);              // 亮度 0-255
    cap.set(cv::CAP_PROP_HUE, 0);                       // 色调 0-360 0r 120g 240b
    cap.set(cv::CAP_PROP_SATURATION, 64);               // 饱和度 0-255
    node_detector_ = std::make_shared<Detector>();
    node_pnp_solver_ = std::make_unique<PnPSolver>();

    node_publisher_stars_ = this->create_publisher<base_interfaces_demo::msg::Stars>("topic_stars",rclcpp::SensorDataQoS());

    RCLCPP_INFO(this->get_logger(),"发布节点Detector_Node创建成功！");
   // timer_ = this->create_wall_timer(std::chrono::milliseconds(2000), std::bind(&DetectorNode::TimerCallback,this));
    TimerCallback();
}

void DetectorNode::TimerCallback()
{
    int flag1=0,flag2=0;
    cv::Mat image;
    while(true)
    {
        if (!rclcpp::ok())
        {
            RCLCPP_INFO(this->get_logger(),"终端退出!");
            return ;
        }

        cap.read(image);
        if(image.empty())break;
        flag1++;
        if(node_detector_->DetectWork(image))
        {
            bool flag=node_pnp_solver_->PnPSolverWork(*node_detector_);
            if(flag)
            {
                flag2++;
                for(size_t i=0;i<5;i++)
                {
                    node_star_.position=true;
                    node_star_.number=i;
                    node_star_.point.x=node_pnp_solver_->result_3d_long_points_[i].x;
                    node_star_.point.y=node_pnp_solver_->result_3d_long_points_[i].y;
                    node_star_.point.z=node_pnp_solver_->result_3d_long_points_[i].z;
                    node_stars_.stars.emplace_back(node_star_);
                    RCLCPP_INFO(this->get_logger(),"long [%ld]: [%s  %lf %lf %lf]",
                    node_star_.number,
                    node_star_.position?"ture":"false",
                    node_star_.point.x,node_star_.point.y,node_star_.point.z);
                }
                for(size_t i=0;i<5;i++)
                {
                    node_star_.position=false;
                    node_star_.number=i+5;
                    node_star_.point.x=node_pnp_solver_->result_3d_short_points_[i].x;
                    node_star_.point.y=node_pnp_solver_->result_3d_short_points_[i].y;
                    // node_star_.point.x=node_pnp_solver_->result_3d_short_points_[i].x;
                    // node_star_.point.y=node_pnp_solver_->result_3d_short_points_[i].y;
                    node_star_.point.z=node_pnp_solver_->result_3d_short_points_[i].z;
                    node_stars_.stars.emplace_back(node_star_);

                    RCLCPP_INFO(this->get_logger(), "short[%ld]: [%s %lf %lf %lf]",
                                node_star_.number,
                                node_star_.position ? "ture" : "false",
                                node_star_.point.x, node_star_.point.y, node_star_.point.z);
                }
                node_publisher_stars_->publish(node_stars_);

                RCLCPP_INFO(this->get_logger(),"%ld发布成功！！！",node_stars_.stars.size());
                RCLCPP_INFO(this->get_logger(),"%d---%d",flag1,flag2);
                node_stars_.stars.clear();
            }
            if(flag)
            {
                cv::Mat Z_end, X_end, Y_end, O;
                cv::projectPoints(std::vector<cv::Point3d>({ cv::Point3d(0.0, 0.0, 0.05) }), node_pnp_solver_->rvec_, node_pnp_solver_->tvec_, 
                node_pnp_solver_->camera_matrix_, node_pnp_solver_->dist_coeffs_, Z_end); // 投影 Z 轴的末端点
                cv::projectPoints(std::vector<cv::Point3d>({ cv::Point3d(0.05, 0.0, 0.0) }), node_pnp_solver_->rvec_, node_pnp_solver_->tvec_, 
                node_pnp_solver_->camera_matrix_, node_pnp_solver_->dist_coeffs_, X_end); // 投影 X 轴的末端点
                cv::projectPoints(std::vector<cv::Point3d>({ cv::Point3d(0.0, -0.05, 0.0) }), node_pnp_solver_->rvec_, node_pnp_solver_->tvec_, 
                node_pnp_solver_->camera_matrix_, node_pnp_solver_->dist_coeffs_, Y_end); // 投影 Y 轴的末端点
                cv::projectPoints(std::vector<cv::Point3d>({ cv::Point3d(0.0, 0.0, 0.0) }), node_pnp_solver_->rvec_, node_pnp_solver_->tvec_,
                node_pnp_solver_->camera_matrix_, node_pnp_solver_->dist_coeffs_, O); // 投影原点
                cv::Point Z_end_point(Z_end.at<cv::Point2d>(0));
                cv::Point X_end_point(X_end.at<cv::Point2d>(0));
                cv::Point Y_end_point(Y_end.at<cv::Point2d>(0));
                cv::Point O_point(O.at<cv::Point2d>(0));
                cv::line(node_detector_->image_, O_point, X_end_point, cv::Scalar(50, 255, 50), 10); // 绘制 X 轴
                cv::line(node_detector_->image_, O_point, Y_end_point, cv::Scalar(50, 50, 255), 10); // 绘制 Y 轴
                cv::line(node_detector_->image_, O_point, Z_end_point, cv::Scalar(255, 50, 50), 10); // 绘制 Z 轴
            }
        }
        // cv::imshow("thresh",node_detector_->image_thresh_);
        cv::imshow("image",node_detector_->image_);
        cv::waitKey(1);
    }
}


