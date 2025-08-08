#include "../include/tf_publish/tf_publish.h"

#ifdef DETECTION_OPENVINO_OPEN
    #warning "DETECTION_OPENVINO_OPEN is defined"  // 编译时会显示此警告
#else
    #warning "DETECTION_OPENVINO_OPEN is NOT defined"
#endif

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<camera::TF_Publisher>());
    rclcpp::shutdown();
    return 0;
}