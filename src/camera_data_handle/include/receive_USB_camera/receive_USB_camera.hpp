#ifndef CAMERA_DATA_HANDLE_RECEIVE_USB_CAMERA_HPP
#define CAMERA_DATA_HANDLE_RECEIVE_USB_CAMERA_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <Eigen/Dense>
#include "Camera_driver.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <opencv2/opencv.hpp>
#include "../../include/Yolov8Detector/Yolov8Detector.h"

//调试时发送出图像信息
class receive_USB : public rclcpp::Node {
public:
    receive_USB();

    cv::Point2f Receive_Keypoint();

private:
    void callback();

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<usb_camera::USBCamera> usb_camera_;
    cv::Mat image_;
    cv::Scalar color_lower_;
    cv::Scalar color_upper_;
};


//得到图像进行筛选以得到结果
namespace usb_camera {
    inline std::vector<std::string> ClassNames = {
        "tank", "Red_cross"
    };

    class USBFactor {
    public:
        explicit USBFactor();

        cv::Point2d Receive_Keypoint();

        cv::Point2d Transform_Image_TO_Real(cv::Point2d &image_point,
                                            geometry_msgs::msg::TransformStamped pose);

    private:
        std::shared_ptr<usb_camera::USBCamera> usb_camera_;
        cv::Mat image_;
        cv::Scalar color_lower_;
        cv::Scalar color_upper_;
#ifdef USB_USE_YOLO
        Yolov8::YOLOv8Detector detector_;
        int times;
#endif
    };
}

#endif //CAMERA_DATA_HANDLE_RECEIVE_USB_CAMERA_HPP
