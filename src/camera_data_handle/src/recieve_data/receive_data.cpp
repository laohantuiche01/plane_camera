#include "../../include/camera_data_handle/camera_handle.h"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
#ifdef KALMAN_OPEN_DEBUG
    rclcpp::executors::MultiThreadedExecutor executor;
    auto kalman_node = std::make_shared<kalman::TopicPublisher>(
        "/kalman_node",
        "/kalman"
    );
    auto node = std::make_shared<camera::ReceiveData>(kalman_node);
    executor.add_node(node);
    executor.add_node(kalman_node);
    executor.spin();
#else
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<camera::ReceiveData>());
#endif
    rclcpp::shutdown();
    cv::destroyAllWindows();
    return 0;
}
