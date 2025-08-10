#include "../include/camera_data_handle/camera_handle.h"

#include "rclcpp/rclcpp.hpp"
#include "yaml-cpp/yaml.h"
#include "sensor_msgs/msg/image.hpp"
#include "opencv4/opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"
#include <openvino/openvino.hpp>


#define STR(s) #s
#define MACRO_TO_STR(s) STR(s)

camera::ReceiveData::ReceiveData() : Node("receive_data"),
                                     detector_(
                                         MACRO_TO_STR(PROJECT_PATH)"/model/best.onnx") {
    RCLCPP_INFO(this->get_logger(), "Receive Data");
    RCLCPP_INFO(this->get_logger(), MACRO_TO_STR(PROJECT_PATH)"/model/best.onnx");

    this->declare_parameter("nms_threshold_", 0.4);
    this->declare_parameter("confidence_threshold_", 0.5);

    this->get_parameter("confidence_threshold_", confidence_threshold_);
    this->get_parameter("nms_threshold_", nms_threshold_);

    position_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/camera/target/position",
        10
    );

    image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/camera/color/image_raw",
        10,
        std::bind(&ReceiveData::imageCallback, this, std::placeholders::_1)
    );

    timer_ = this->create_wall_timer(
        std::chrono::seconds(1), [this]() {
            if (!has_received_) {
                RCLCPP_WARN(this->get_logger(), "No Image Receive");
            } else {
                RCLCPP_INFO(this->get_logger(), "Has been receive!");
                timer_->cancel();
            }
        });
}

void camera::ReceiveData::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg) {
    has_received_ = true;
    try {
        fps_sum_++;
        if (msg->width <= 0 || msg->height <= 0) {
            RCLCPP_WARN(this->get_logger(), "Invalid image dimensions: %dx%d", msg->width, msg->height);
            return;
        }

#ifdef FPS_VISABLE_OPEN
        fps_timer_.start(); //计时以计算帧率
#endif

        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
        cv::Mat temp_image = cv_ptr->image;
        cv::Mat image;
        cv::cvtColor(temp_image, image, cv::COLOR_BGR2RGB);

        //目标检测接口
        std::vector<Yolov8::Detection> detections = detector_.detect(image);
#ifdef KEY_POINT_TRACKING
        if (!detections.empty()) if_do_tracking_ = true;

        if (if_do_tracking_ && detections.empty()) {
            detections = detector_.track(image);
            if (detections.empty()) if_do_tracking_ = false;
        }
#endif

        std::vector<std::vector<double> > positions = detector_.drawDetections(image, detections);


#ifdef PREDICT_OPEN
        //卡尔曼滤波器接口
        if (!detections.empty()) {
            tracking_num_ = 1;
        }
        try {
            if (tracker_ == nullptr && tracking_num_) {
                tracker_ = new KalmanBoxTracker(detections.front().box);
            } else if (tracking_num_) {
                if (!detections.empty()) {
                    //RCLCPP_ERROR(this->get_logger(), "-----------------------------!!!!!");
                    Rect input_rect = detections.front().box;
                    tracker_->update(input_rect);
                }
                Point2f predictCenter = tracker_->predict();
                Rect estimatedBox = tracker_->get_state();
                //RCLCPP_ERROR(this->get_logger(), "-----------------------------");
                circle(image, predictCenter, 5, Scalar(200, 0, 120), 2);
                cv::rectangle(image, estimatedBox, Scalar(255, 0, 0), 2);
            }
        } catch (cv::Exception e) {
            RCLCPP_WARN(this->get_logger(), "%s", e.what());
        }

        if (detections.empty() && tracking_num_) {
            tracking_num_++;
            if (tracking_num_ == 10) {
                tracking_num_ = 0;
            }
        }

#endif

        //信息发送变量
        std_msgs::msg::Float64MultiArray position_msg;

        if (positions.empty()) {
            RCLCPP_WARN(this->get_logger(), "No detections found");
            position_msg.data = std::vector<double>(0, 0);
            position_pub_->publish(position_msg);
        } else {
            while (!positions.empty()) {
                position_msg.data = positions.front();
                positions.pop_back();
                position_pub_->publish(position_msg);
            }
        }
        cv::circle(image, cv::Point(320, 240), 207, cv::Scalar(0, 255, 0), 1);
        cv::line(image, cv::Point(0, 240), cv::Point(640, 240), cv::Scalar(0, 255, 0), 1);
        cv::line(image, cv::Point(320, 0), cv::Point(320, 480), cv::Scalar(0, 255, 0), 1);

#ifdef FPS_VISABLE_OPEN
        //计算fps的
        {
            fps_timer_.stop();
            double currentTime = fps_timer_.getTimeSec();
            fps_time_sum_ += currentTime;
            double fps = fps_sum_ / fps_time_sum_;
            std::stringstream ss;
            ss << "FPS: " << std::fixed << std::setprecision(2) << fps;
            cv::putText(image, ss.str(), cv::Point(10, 30),
                        cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);
        }
#endif

        namedWindow("image", cv::WINDOW_NORMAL);
        cv::resizeWindow("image", 2000, 1500);

#ifdef FPS_VISABLE_OPEN
        fps_timer_.reset();

        if (fps_sum_ == 20) {
            fps_sum_ = 0;
            fps_time_sum_ = 0;
        }
#endif

        cv::imshow("image", image);
        cv::waitKey(1);
    } catch (cv_bridge::Exception &e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Exception in image callback: %s", e.what());
    } catch (...) {
        RCLCPP_ERROR(this->get_logger(), "Unknown exception in image callback");
    }
}
