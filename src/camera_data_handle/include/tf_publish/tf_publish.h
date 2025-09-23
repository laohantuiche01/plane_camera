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

#include "tf2_ros/transform_broadcaster.h"
#include "../receive_openmv_data/receive_openmv_data.h"
#include "robot_interfaces/msg/image_location.hpp"

namespace camera {
    enum Target {
        H = 0,
        TENT = 1,
        CAR = 2,
        BRIDGE = 3,
        PILLBOX = 4,
        TANK = 5,
        RED_CROSS = 6,
        UNKNOW = 7,
        INITIALIZER = 8,
    };


    ///基类--------------------------------------------------------------------------------
    class TF_Publisher_Base {
    public:
        virtual ~TF_Publisher_Base() = default;

        TF_Publisher_Base();

    protected:
        virtual void publish_transform();

        void initialize_transform(robot_interfaces::msg::ImageLocation &msg_loader);

        //隔断时间
        rclcpp::TimerBase::SharedPtr send_timer_;

        //发布信息的容器
        robot_interfaces::msg::ImageLocation pub_pos_;

        //识别坐标的tf广播器
        //std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

        //发送的信息(这个是具体地识别信息)
        geometry_msgs::msg::TransformStamped transform_;

        bool transform_initialized;

        //测试用的高度
        double height_;

    };

    ///继承的目标检测的类-------------------------------------------------------------------------------------
    class Detect_Publisher : public TF_Publisher_Base, public rclcpp::Node {
    public:
        Detect_Publisher();

        ~Detect_Publisher() override = default;

    private:
        void publish_transform() override;

        void position_callback(const robot_interfaces::msg::ImageLocation::SharedPtr msg);

#ifndef HIGHT_DEBUG

        //接受高度

#endif

        //自定义接口信息发送
        rclcpp::Publisher<robot_interfaces::msg::ImageLocation>::SharedPtr position_pub_;

        //接受点的信息
        rclcpp::Subscription<robot_interfaces::msg::ImageLocation>::SharedPtr position_subscription;

        //更新计数器（有时候丢失目标，能够延时）
        long tf2_reflash_;

        //最大更新次数
        int tf2_reflash_num_;
    };

    ///继承的猜测openmv的类----------------------------------------------------------------------
    class Calculate_Publisher : public TF_Publisher_Base, public rclcpp::Node {
    public:
        Calculate_Publisher();

        ~Calculate_Publisher() override = default;

    private:
        void publish_transform() override;

        //自定义接口信息发送
        rclcpp::Publisher<robot_interfaces::msg::ImageLocation>::SharedPtr position_pub_;

        std::shared_ptr<CalculateTarget> calculate_target_class_;

        //openmv的信息
        geometry_msgs::msg::TransformStamped transform_openmv_;
    };
}


#endif //TF_PUBLISH_H
