#include "../include/camera_data_handle/camera_handle.h"

#include "rclcpp/rclcpp.hpp"
#include "yaml-cpp/yaml.h"
#include "sensor_msgs/msg/image.hpp"
#include "opencv4/opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"

camera::ReceiveData::ReceiveData(): Node("receive_data"), confidence_threshold_(0.5),
                                    nms_threshold_(0.4),
                                    detector(
                                        "/home/zxk/桌面/Unmanned_Aerial_Vehicle_Workspace/camera_handle/src/camera_data_handle/model/best.onnx"
                                    ) {
    RCLCPP_INFO(this->get_logger(), "Receive Data");

    this->declare_parameter("nms_threshold_", 0.4);
    this->declare_parameter("confidence_threshold_", 0.5);

    this->get_parameter("confidence_threshold_", confidence_threshold_);
    this->get_parameter("nms_threshold_", nms_threshold_);

    position_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/camera/target/position",
        10
    );

    image_subscription = this->create_subscription<sensor_msgs::msg::Image>(
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
        if (msg->width <= 0 || msg->height <= 0) {
            RCLCPP_WARN(this->get_logger(), "Invalid image dimensions: %dx%d", msg->width, msg->height);
            return;
        }

        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
        cv::Mat temp_image = cv_ptr->image;
        cv::Mat image;
        cv::cvtColor(temp_image, image, cv::COLOR_BGR2RGB);

        std::vector<std::vector<double> > positions;

        std::vector<Yolov8::Detection> detections = detector.detect(image);
        positions = detector.drawDetections(image, detections);

        std_msgs::msg::Float64MultiArray position_msg;

        if (positions.empty()) {
            RCLCPP_WARN(this->get_logger(), "No detections found");
            position_msg.data = std::vector<double>(2,10000);
            position_pub->publish(position_msg);
        }
        else {
            while (!positions.empty()) {
                position_msg.data = positions.front();
                positions.pop_back();
                position_pub->publish(position_msg);
            }
        }

        cv::line(image,cv::Point(0,240),cv::Point(640,240),cv::Scalar(0, 255, 0), 1);
        cv::line(image,cv::Point(320,0),cv::Point(320,480),cv::Scalar(0, 255, 0), 1);

        namedWindow("image", cv::WINDOW_NORMAL);
        cv::resizeWindow("image", 1600, 1200);

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
