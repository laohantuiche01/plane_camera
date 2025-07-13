#ifndef CAMERA_HANDLE_H
#define CAMERA_HANDLE_H

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "opencv4/opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "../Yolov8Detector/Yolov8Detector.h"

namespace camera {
    class ReceiveData : public rclcpp::Node {
    public:
        explicit ReceiveData();

    private:
        void imageCallback(sensor_msgs::msg::Image::ConstSharedPtr msg);

        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr position_pub;
        rclcpp::TimerBase::SharedPtr timer_;

        std::vector<std::string> output_names_;
        std::vector<std::string> class_names_;

        cv::dnn::Net net_;

        std::string model_path_;

        Yolov8::YOLOv8Detector detector;

        double confidence_threshold_;
        double nms_threshold_;
        bool has_received_{false};

        cv::TickMeter fps_timer_;
        double fps_time_sum;
        long fps_sum;
    };
}


#endif //CAMERA_HANDLE_H
