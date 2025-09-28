#ifndef CAMERA_DATA_HANDLE_RECEIVE_OPENMV_DATA_H
#define CAMERA_DATA_HANDLE_RECEIVE_OPENMV_DATA_H

#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include <cstdint>
#include <termios.h>
#include <fcntl.h>
#include <memory>
#include <unistd.h>
#include <sys/ioctl.h>
#include <opencv2/opencv.hpp>
#include "nlohmann/json.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <Eigen/Eigen>
#include "cmath"


/// 读取openmv的数据流
class ReceiveOpenMVData {
public:
    explicit ReceiveOpenMVData(std::string port = "/dev/ttyACM0", speed_t baudRate = B115200);

    std::string Receive_Openmv_Data();

private:
    const std::string port_;
    const speed_t baudRate_;

    static int openSerialPort(const std::string &port, speed_t baudRate);

    std::string readLine(int fd);
};

///进行解算
class CalculateTarget {
public:
    CalculateTarget();

    cv::Point2d Handle_Openmv_Data(const geometry_msgs::msg::TransformStamped& pose);

private:
    // 创建接受信息的ReceiveOpenMVData指针
    std::shared_ptr<ReceiveOpenMVData> receive_openmv_data_;

    // 将受到的字符串解码
    static cv::Point2d Decode_Openmv_Data(std::string &input_str);

    // 将得到的像素坐标解算(主要算法)
    static cv::Point2d Transform_Image_TO_Real(cv::Point2d &image_point, geometry_msgs::msg::TransformStamped pose);
};

#endif //CAMERA_DATA_HANDLE_RECEIVE_OPENMV_DATA_H
