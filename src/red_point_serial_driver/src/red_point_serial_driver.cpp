#include "red_point_serial_driver/crc.hpp"
#include "red_point_serial_driver/packet.hpp"
#include "red_point_serial_driver/red_point_serial_driver.hpp"

RedPointSerialDriver::RedPointSerialDriver():Node("red_point_serial_driver","Stars"),
  owned_ctx_{new IoContext(2)},
  serial_driver_{new drivers::serial_driver::SerialDriver(*owned_ctx_)}
{

    RCLCPP_INFO(get_logger(), "red_point_serial_driver启动！！！");

    getParams();

    try {
        //初始化了串口驱动，配置了设备名称和设备配置
        serial_driver_->init_port(device_name_, *device_config_);

        if (!serial_driver_->port()->is_open()) {
        serial_driver_->port()->open();
        //创建新线程用于接收串口数据

        }
    } catch (const std::exception & ex) {
        RCLCPP_ERROR(
        get_logger(), "Error creating serial port: %s - %s", device_name_.c_str(), ex.what());
        throw ex;
    }

    // node_subscription_stars_=this->create_subscription<base_interfaces_demo::msg::Stars>("topic_stars",rclcpp::SensorDataQoS(),
    // std::bind(&MyListener::my_callback,this,std::placeholders::_1));
    // Create Subscription 创建订阅方
    red_point_subscription_ = this->create_subscription<base_interfaces_demo::msg::Redpoint>(
        "topic_stars",rclcpp::SensorDataQoS(),//Stars/
        std::bind(&RedPointSerialDriver::sendData,this,std::placeholders::_1)
    );

}

RedPointSerialDriver::~RedPointSerialDriver()
{
    //在使用串口之后，如果串口处于打开状态，则关闭串口以释放资源和避免可能的资源泄露
    if (serial_driver_->port()->is_open()) {
        serial_driver_->port()->close();
    }
    //存在一个上下文对象 owned_ctx_，则等待该上下文退出
    if (owned_ctx_) {
        owned_ctx_->waitForExit();
    }
}

void RedPointSerialDriver::getParams()
{
    using FlowControl = drivers::serial_driver::FlowControl;
    using Parity = drivers::serial_driver::Parity;
    using StopBits = drivers::serial_driver::StopBits;


    //波特率 (baud_rate)、流控制 (fc)、奇偶校验 (pt) 和停止位 (sb)
    uint32_t baud_rate{};
    //枚举类型变量
    auto fc = FlowControl::NONE;
    auto pt = Parity::NONE;
    auto sb = StopBits::ONE;

    try {
        device_name_ = declare_parameter<std::string>("device_name", "");
    } catch (rclcpp::ParameterTypeException & ex) {
        RCLCPP_ERROR(get_logger(), "The device name provided was invalid");
        throw ex;
    }

    try {
        baud_rate = declare_parameter<int>("baud_rate", 0);
    } catch (rclcpp::ParameterTypeException & ex) {
        RCLCPP_ERROR(get_logger(), "The baud_rate provided was invalid");
        throw ex;
    }

    try {
        const auto fc_string = declare_parameter<std::string>("flow_control", "");

        if (fc_string == "none") {
        fc = FlowControl::NONE;
        } else if (fc_string == "hardware") {
        fc = FlowControl::HARDWARE;
        } else if (fc_string == "software") {
        fc = FlowControl::SOFTWARE;
        } else {
        throw std::invalid_argument{
            "The flow_control parameter must be one of: none, software, or hardware."};
        }
    } catch (rclcpp::ParameterTypeException & ex) {
        RCLCPP_ERROR(get_logger(), "The flow_control provided was invalid");
        throw ex;
    }

    try {
        const auto pt_string = declare_parameter<std::string>("parity", "");

        if (pt_string == "none") {
        pt = Parity::NONE;
        } else if (pt_string == "odd") {
        pt = Parity::ODD;
        } else if (pt_string == "even") {
        pt = Parity::EVEN;
        } else {
        throw std::invalid_argument{"The parity parameter must be one of: none, odd, or even."};
        }
    } catch (rclcpp::ParameterTypeException & ex) {
        RCLCPP_ERROR(get_logger(), "The parity provided was invalid");
        throw ex;
    }

    try {
        const auto sb_string = declare_parameter<std::string>("stop_bits", "");

        if (sb_string == "1" || sb_string == "1.0") {
        sb = StopBits::ONE;
        } else if (sb_string == "1.5") {
        sb = StopBits::ONE_POINT_FIVE;
        } else if (sb_string == "2" || sb_string == "2.0") {
        sb = StopBits::TWO;
        } else {
        throw std::invalid_argument{"The stop_bits parameter must be one of: 1, 1.5, or 2."};
        }
    } catch (rclcpp::ParameterTypeException & ex) {
        RCLCPP_ERROR(get_logger(), "The stop_bits provided was invalid");
        throw ex;
    }


    //创建了一个动态分配的 SerialPortConfig 类型的对象
    device_config_ =
        std::make_unique<drivers::serial_driver::SerialPortConfig>(baud_rate, fc, pt, sb);


}

void RedPointSerialDriver::sendData(base_interfaces_demo::msg::Redpoint::SharedPtr msg)
{
    // if (msg->stars.size() != 10)
    //     return;
    // RCLCPP_INFO(this->get_logger(), "sendData~~~~~");
    // try
    // {
    //     std::vector<SendPacket> packets;
    //     packets.reserve(10);
    //     for (size_t i = 0; i < 10; i++)
    //     {
    //         SendPacket packet;
    //         // packet.star_number=msg->stars[i].number;
    //         packet.star_x = msg->stars[i].point.x;
    //         packet.star_y = msg->stars[i].point.y;
    //         packet.star_z = msg->stars[i].point.z;
    //         // packet.star_position=msg->stars[i].position;
    //         // 计算 CRC16 校验和
    //         // Append_CRC16_Check_Sum(reinterpret_cast<uint8_t *>(&packet), sizeof(packet));
    //         packets.push_back(packet);
    //         RCLCPP_INFO(this->get_logger(), "[%ld!] %lf,%lf,%lf", i, msg->stars[i].point.x, msg->stars[i].point.y, msg->stars[i].point.z);
    //         RCLCPP_INFO(this->get_logger(), "[%ld~] %f,%f,%f", i, packet.star_x, packet.star_y, packet.star_z);
    //     }
    //     //  将数据包转换为字节向量，并通过串口发送该字节向量
    //     std::vector<uint8_t> all_data;
    //     size_t total_size = sizeof(SendPacket) * packets.size();
    //     all_data.reserve(total_size);
    //     for (const auto &packet : packets)
    //     {
    //         std::vector<uint8_t> data = toVector(packet);
    //         all_data.insert(all_data.end(), data.begin(), data.end());
    //     }
    //     // for(size_t j=0;j<all_data.size();j++)
    //     // {
    //     //     printf("0x%02X",all_data[j]);
    //     // }
    //     // std::cout<<std::endl;
    //     packets.clear();
    //     serial_driver_->port()->send(all_data);
    //     all_data.clear();
    // }
    // catch (const std::exception &ex)
    // {
    //     RCLCPP_ERROR(get_logger(), "Error while sending data: %s", ex.what());
    //     reopenPort();
    // }
}
void RedPointSerialDriver::reopenPort()
{
    RCLCPP_WARN(get_logger(), "Attempting to reopen port");

    try {
        //重新打开port（）
        if (serial_driver_->port()->is_open()) {
        serial_driver_->port()->close();
        }
        serial_driver_->port()->open();
        RCLCPP_INFO(get_logger(), "Successfully reopened port");
    } catch (const std::exception & ex) {
        RCLCPP_ERROR(get_logger(), "Error while reopening port: %s", ex.what());
        if (rclcpp::ok()) {
        //让当前执行此代码的线程休眠1秒钟
        rclcpp::sleep_for(std::chrono::seconds(1));
        reopenPort();
        }
    }
}


