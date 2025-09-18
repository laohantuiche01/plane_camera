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
#include "../receive_openmv_data/receive_openmv_data.h"


namespace camera {
    class TF_Publisher : public rclcpp::Node {
    public:
        TF_Publisher();

    private:
        void position_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

        void publish_transform();

        void initialize_transform(geometry_msgs::msg::TransformStamped &msg_loader, const std::string &header_id,
                                  const std::string &child_id);

        //图像解算的类
        std::shared_ptr<CalculateTarget> calculate_target_class_;

        //接受高度信息

        //接受识别的图像信息
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr position_subscription;

        //隔断时间
        rclcpp::TimerBase::SharedPtr send_timer_;

        //识别坐标的tf广播器
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

        //猜测大概目标的tf广播器
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_guess_;

        //发送的信息(这个是具体的识别信息)
        geometry_msgs::msg::TransformStamped transform_;

        //openmv的信息
        geometry_msgs::msg::TransformStamped transform_openmv_;

        bool transform_initialized = false;

        //更新计数器（有时候丢失目标，能够延时）
        long tf2_reflash_;

        //最大更新次数
        int tf2_reflash_num_;

        //测试用的高度
        double height_;

        //接收到的高度
        double receive_height_{0};
    };
}


#endif //TF_PUBLISH_H
