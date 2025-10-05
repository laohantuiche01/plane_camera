#include "../../include/camera_data_handle/camera_debug.h"

#include "rclcpp/rclcpp.hpp"

using namespace cv;
using namespace std;

camera::CameraDebug::CameraDebug() :Node("camera_debug"){

    pub_=this->create_publisher<sensor_msgs::msg::Image>("/camera/camera/color/image_raw", 10);

    imageSend();
}

void camera::CameraDebug::imageSend() {

    cv::VideoCapture camera("../video/第六次.avi");
    Mat frame;
    while (true) {
        camera.read(frame);

        sensor_msgs::msg::Image img_msg;

        cvtColor(frame,frame,cv::COLOR_BGR2RGB);

        img_msg.encoding = "rgb8";
        img_msg.header.frame_id = "camera";
        img_msg.width = frame.cols;
        img_msg.height = frame.rows;
        img_msg.step = frame.step;

        size_t size = frame.step * frame.rows ;
        img_msg.data.resize(size);
        memcpy(&img_msg.data[0], frame.data, size);

        img_msg.header.stamp = rclcpp::Clock().now();

        pub_->publish(img_msg);

        //imshow("camera", frame);
        waitKey(30);
    }

}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<camera::CameraDebug>());
    rclcpp::shutdown();
    cv::destroyAllWindows();
    return 0;
}
