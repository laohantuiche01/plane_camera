#ifndef CAMERA_DATA_HANDLE_RECEIVE_USB_CAMERA_HPP
#define CAMERA_DATA_HANDLE_RECEIVE_USB_CAMERA_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <Eigen/Dense>
#include "Camera_driver.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <opencv2/opencv.hpp>

// class receive_USB : public rclcpp::Node {
// public:
//     receive_USB();
//
//     cv::Point2f Receive_Keypoint();
//
// private:
//     void callback();
//
//     rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
//     rclcpp::TimerBase::SharedPtr timer_;
//     std::shared_ptr<usb_camera::USBCamera> usb_camera_;
//     cv::Mat image_;
//     cv::Scalar color_lower_;
//     cv::Scalar color_upper_;
// };
namespace usb_camera {
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
    };
}

#endif //CAMERA_DATA_HANDLE_RECEIVE_USB_CAMERA_HPP
