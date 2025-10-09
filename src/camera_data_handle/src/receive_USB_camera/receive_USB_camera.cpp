#include "../../include/receive_USB_camera/receive_USB_camera.hpp"

receive_USB::receive_USB() : Node("USB_pub") {
    usb_camera_ = std::make_shared<usb_camera::USBCamera>(2);
    usb_camera_->OpenCameraDevice();
    usb_camera_->SetExposure(150);
    pub_ = this->create_publisher<sensor_msgs::msg::Image>("/camera/camera/color/image_raw", 10);
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(30),
        std::bind(&receive_USB::callback, this)
    );
}

void receive_USB::callback() {
    sensor_msgs::msg::Image img_msg;
    cv::Mat frame;

    frame = usb_camera_->GetFrame();
    cv::cvtColor(frame, frame, cv::COLOR_BGR2RGB);
    img_msg.encoding = "rgb8";
    img_msg.header.frame_id = "camera";
    img_msg.width = frame.cols;
    img_msg.height = frame.rows;
    img_msg.step = frame.step;

    size_t size = frame.step * frame.rows;
    img_msg.data.resize(size);
    memcpy(&img_msg.data[0], frame.data, size);

    img_msg.header.stamp = rclcpp::Clock().now();

    pub_->publish(img_msg);
}
