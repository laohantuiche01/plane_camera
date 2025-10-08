#ifndef CAMERA_DATA_HANDLE_RECEIVE_USB_CAMERA_HPP
#define CAMERA_DATA_HANDLE_RECEIVE_USB_CAMERA_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "Camera_driver.hpp"

class receive_USB : public rclcpp::Node {
public:
    receive_USB();
private:
    void callback();
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<usb_camera::USBCamera> usb_camera_;
};

#endif //CAMERA_DATA_HANDLE_RECEIVE_USB_CAMERA_HPP
