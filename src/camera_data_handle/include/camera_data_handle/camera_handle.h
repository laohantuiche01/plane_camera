#ifndef CAMERA_HANDLE_H
#define CAMERA_HANDLE_H

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "opencv4/opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"

#include "../Yolov8Detector/Yolov8Detector.h"


class ReceiveData : public rclcpp::Node {
public:
    explicit ReceiveData();

private:
    void imageCallback(sensor_msgs::msg::Image::ConstSharedPtr msg) ;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<std::string> output_names_;
    std::vector<std::string> class_names_;

    cv::dnn::Net net_;

    std::string model_path_;

    Yolov8::YOLOv8Detector detector;

    double confidence_threshold_;
    double nms_threshold_;
    bool has_received_{false};
};



#endif //CAMERA_HANDLE_H
