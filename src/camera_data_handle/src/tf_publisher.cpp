#include "../include/tf_publish/tf_publish.h"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<camera::TF_Publisher>());
    rclcpp::shutdown();
    return 0;
}