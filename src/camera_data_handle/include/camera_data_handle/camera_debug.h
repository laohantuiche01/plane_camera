
#ifndef CAMERA_DATA_HANDLE_CAMERA_DEBUG_H
#define CAMERA_DATA_HANDLE_CAMERA_DEBUG_H

#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/image.hpp"
#include "opencv4/opencv2/opencv.hpp"
namespace camera {

    class CameraDebug : public rclcpp::Node {
    public:
        explicit CameraDebug();

    private:
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;

        void imageSend();
    };


}



#endif //CAMERA_DATA_HANDLE_CAMERA_DEBUG_H