#ifndef CAMERA_HANDLE_H
#define CAMERA_HANDLE_H

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "opencv4/opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "opencv2/videoio.hpp"

#include "../Yolov8Detector/Yolov8Detector.h"
#include "../kalman/kalmanbox.h"
#include "target_predict.h"
#include "../point_tracking/point_tracking.h"
#include "robot_interfaces/msg/image_location.hpp"


namespace camera {
    class ReceiveData : public rclcpp::Node {
    public:
        explicit ReceiveData();



    private:
        void imageCallback(sensor_msgs::msg::Image::ConstSharedPtr msg);

        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;

        rclcpp::Publisher<robot_interfaces::msg::ImageLocation>::SharedPtr position_pub_;
        rclcpp::TimerBase::SharedPtr timer_;

        std::vector<std::string> output_names_;
        std::vector<std::string> class_names_;

        //camera::TargetPredictFactory target_predict_factory_;

        camera::KalmanBoxTracker *tracker_{nullptr};
        int tracking_num_{0};

        //如果检测或者跟踪都失效时设置为false
        bool if_do_tracking_{false};

        cv::dnn::Net net_;

        std::string model_path_;

#ifndef KEY_POINT_TRACKING
        Yolov8::YOLOv8Detector detector_;
#endif

#ifdef KEY_POINT_TRACKING
        Yolov8::YOLOv8Tracker detector_;
#endif

#ifdef  VIDEO_WRITE
        int frameWidth_ = 640;
        int frameHeight_ = 480;

        const string video_name_ = "output.avi";

        cv::VideoWriter writer_;

#endif
        double confidence_threshold_{0.0};
        double nms_threshold_{0.0};
        bool has_received_{false};

        cv::TickMeter fps_timer_;
        double fps_time_sum_{0.0};
        int fps_sum_{0};
    };
}


#endif //CAMERA_HANDLE_H
