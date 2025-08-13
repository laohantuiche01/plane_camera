#ifndef TF_PUBLISH_H
#define TF_PUBLISH_H

#define THE_TRANSFORM_USE_PREDICT
//#define THE_TRANSFORM_USE_ACCELERATE

#ifdef THE_TRANSFORM_USE_PREDICT
#ifdef THE_TRANSFORM_USE_ACCELERATE
#error "You cannot define both ! ! !"
#endif
#endif

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include "tf2_ros/transform_broadcaster.h"


namespace camera {
    class TF_Publisher : public rclcpp::Node {
    public:
        TF_Publisher() : Node("TF_publish"), height(1.5), tf2_reflash(0), tf2_reflash_num(0) {

            RCLCPP_INFO(this->get_logger(), "TF_Publisher");

            this->declare_parameter("height", 1.5);
            this->declare_parameter("tf2_reflash_num", 1);

            this->get_parameter("tf2_reflash_num", tf2_reflash_num);
            this->get_parameter("height", height);

            tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
            position_subscription = this->create_subscription<std_msgs::msg::Float64MultiArray>(
                "/camera/target/position",
                10,
                std::bind(&TF_Publisher::position_callback, this, std::placeholders::_1)
            );

            send_timer_ = this->create_wall_timer(
                std::chrono::milliseconds(10),
                std::bind(&TF_Publisher::publish_transform, this));
        }

    private:
        void position_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
            //std::cout<<tf2_reflash_num<<std::endl;
            if (msg.get()->data.empty()) //处理为空的情况
            {
                tf2_reflash++;
                if (tf2_reflash >= tf2_reflash_num) {
                    transform_.transform.translation.x = 0;
                    transform_.transform.translation.y = 0;
                    transform_.transform.translation.z = 0;
                    tf2_reflash = 0;
                }
                return;
            }

            tf2_reflash = 0; // 更新

            double x, y, z;
            x = msg->data[0];
            y = msg->data[1];
            z = height;
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

        void publish_transform() {
            if (!transform_initialized) {
                initialize_transform();
                transform_initialized = true;
            }

            transform_.header.stamp = this->get_clock()->now();
            tf_broadcaster->sendTransform(transform_);
        }

        void initialize_transform() {
            transform_.header.frame_id = "camera_color_frame";
            transform_.child_frame_id = "target_position";
            transform_.transform.translation.x = 0;
            transform_.transform.translation.y = 0;
            transform_.transform.translation.z = 0;

            transform_.transform.rotation.x = 0;
            transform_.transform.rotation.y = 0;
            transform_.transform.rotation.z = 0;
            transform_.transform.rotation.w = 1;
        }

        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr position_subscription;
        rclcpp::TimerBase::SharedPtr send_timer_;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
        geometry_msgs::msg::TransformStamped transform_;
        bool transform_initialized = false;

        long tf2_reflash;
        int tf2_reflash_num;

        double height;
    };
}


#endif //TF_PUBLISH_H
