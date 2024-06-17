#include "my_test/my_test_listener.hpp"

MyListener::MyListener():Node("my_test_listener","Stars")
{

    RCLCPP_INFO(this->get_logger(),"发布节点Detector_Node创建！");
    node_subscription_stars_=this->create_subscription<base_interfaces_demo::msg::Stars>(
        "topic_stars",rclcpp::SensorDataQoS(),
    std::bind(&MyListener::my_callback,this,std::placeholders::_1));


}

void MyListener::my_callback(const base_interfaces_demo::msg::Stars stars)
{
    for(size_t i=0;i<stars.stars.size();i++)
    {
        RCLCPP_INFO(this->get_logger(),"[%s %ld %lf %lf %lf]",
        stars.stars.at(i).position ? "true ":"false",
        stars.stars.at(i).number,
        stars.stars.at(i).point.x,
        stars.stars.at(i).point.y,
        stars.stars.at(i).point.z);
    }
    RCLCPP_INFO(this->get_logger(),"消息接收成功！！！");
    //RCLCPP_INFO(this->get_logger(),"%s",stars.stars.at(0).position ? "true":"false");
}
