#include <yaml-cpp/node/node.h>

#include "../../include/tf_publish/tf_publish.h"

#ifdef DETECTION_OPENVINO_OPEN
#warning "DETECTION_OPENVINO_OPEN is defined"
#else
#warning "DETECTION_OPENVINO_OPEN is NOT defined"
#endif

#ifndef OPENMV_NULL_ERROR
#define OPENMV_NULL_ERROR 321
#endif

camera::TF_Publisher_Base::TF_Publisher_Base() : height_(0), receive_height_(0), transform_initialized(false) {
}

void camera::TF_Publisher_Base::publish_transform() {
}

void camera::TF_Publisher_Base::initialize_transform(geometry_msgs::msg::TransformStamped &msg_loader,
                                                     const std::string &header_id,
                                                     const std::string &child_id) {
    msg_loader.header.frame_id = header_id;
    msg_loader.child_frame_id = child_id;
    msg_loader.transform.translation.x = 0;
    msg_loader.transform.translation.y = 0;
    msg_loader.transform.translation.z = 0;

    msg_loader.transform.rotation.x = 0;
    msg_loader.transform.rotation.y = 0;
    msg_loader.transform.rotation.z = 0;
    msg_loader.transform.rotation.w = 1;
}

///继承的目标检测的类
///---------------------------------------------------------------------------------------------------------------------
camera::Detect_Publisher::Detect_Publisher() : Node("Detect_Publisher"), tf2_reflash_(0), tf2_reflash_num_(0) {
    RCLCPP_INFO(this->get_logger(), "TF_Publisher");

    this->declare_parameter("height", 1.5);
    this->declare_parameter("tf2_reflash_num", 1);

    this->get_parameter("tf2_reflash_num", tf2_reflash_num_);
    this->get_parameter("height", height_);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    position_subscription = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/camera/target/position",
        10,
        std::bind(&Detect_Publisher::position_callback, this, std::placeholders::_1)
    );

    send_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&Detect_Publisher::publish_transform, this));
}

void camera::Detect_Publisher::position_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    //std::cout<<tf2_reflash_num<<std::endl;
    if (msg.get()->data.empty()) //处理为空的情况
    {
        tf2_reflash_++;
        if (tf2_reflash_ >= tf2_reflash_num_) {
            transform_.transform.translation.x = 0;
            transform_.transform.translation.y = 0;
            transform_.transform.translation.z = 0;
            tf2_reflash_ = 0;
        }
        return;
    }

    tf2_reflash_ = 0; // 更新

    double x, y, z;
    x = msg->data[0];
    y = msg->data[1];
    z = height_;
    //std::cout << x << " " << y << " " << z << std::endl;
#ifdef THE_TRANSFORM_USE_PREDICT
    transform_.transform.translation.x = x * 42 / 20700;
    transform_.transform.translation.y = y * 42 / 20700;
    transform_.transform.translation.z = z;

    RCLCPP_INFO(this->get_logger(), "x: %f, y: %f, z: %f", transform_.transform.translation.x,
                transform_.transform.translation.y, transform_.transform.translation.z);
#endif

#ifdef THE_TRANSFORM_USE_ACCELERATE
    constexpr double param = 0;

    transform_.transform.translation.x = 0;
    transform_.transform.translation.y = 0;
    transform_.transform.translation.z = 0;

#endif
}

void camera::Detect_Publisher::publish_transform() {
    if (!transform_initialized) {
        std::string header_frame_id = "camera_color_frame";
        std::string child_frame_id = "target_position";
        initialize_transform(transform_, header_frame_id, child_frame_id);
        transform_initialized = true;
    }

    transform_.header.stamp = this->get_clock()->now();
    tf_broadcaster_->sendTransform(transform_);
}

///继承的猜测openmv的类
///---------------------------------------------------------------------------------------------------------------------
camera::Calculate_Publisher::Calculate_Publisher() : Node("Calculate_Publisher") {
    RCLCPP_INFO(this->get_logger(), "TF_Publisher");

    calculate_target_class_ = std::make_shared<CalculateTarget>();

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    send_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&Calculate_Publisher::publish_transform, this));
}

void camera::Calculate_Publisher::publish_transform() {
    if (!transform_initialized) {
        std::string header_frame_id = "camera_color_frame";
        std::string child_frame_id = "openmv_target_position";
        initialize_transform(transform_openmv_, header_frame_id, child_frame_id);
        transform_initialized = true;
    }

    transform_openmv_.header.stamp = this->get_clock()->now();
    cv::Point2d guess_point_ = calculate_target_class_.get()->Handle_Openmv_Data();
    if (guess_point_.x == OPENMV_NULL_ERROR && guess_point_.y == OPENMV_NULL_ERROR) {
        guess_point_.x = 0;
        guess_point_.y = 0;
    }
    transform_openmv_.transform.translation.x = guess_point_.x;
    transform_openmv_.transform.translation.y = guess_point_.y;
    transform_openmv_.transform.translation.z = -height_;
    transform_openmv_.transform.rotation.x = 0;
    transform_openmv_.transform.rotation.y = 0;
    transform_openmv_.transform.rotation.z = 0;
    transform_openmv_.transform.rotation.w = 1;
    std::cout << guess_point_.x << " " << guess_point_.y << std::endl;
    tf_broadcaster_->sendTransform(transform_openmv_);
}


