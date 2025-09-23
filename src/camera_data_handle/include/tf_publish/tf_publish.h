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
#include "camera_info/msg/position.hpp"

namespace camera {
    enum Target {
        H = 0,
        TENT = 1,
        CAR = 2,
        BRIDGE = 3,
        PILLBOX = 4,
        TANK = 5,
        RED_CROSS = 6,
    };


    ///基类--------------------------------------------------------------------------------
    class TF_Publisher_Base {
    public:
        virtual ~TF_Publisher_Base() = default;

        TF_Publisher_Base();

    protected:
        virtual void publish_transform();

        void initialize_transform(geometry_msgs::msg::TransformStamped &msg_loader, const std::string &header_id,
                                  const std::string &child_id);

        //隔断时间
        rclcpp::TimerBase::SharedPtr send_timer_;

        //识别坐标的tf广播器
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

        //发送的信息(这个是具体地识别信息)
        geometry_msgs::msg::TransformStamped transform_;

        bool transform_initialized;

        //测试用的高度
        double height_;

        //接收到的高度
        double receive_height_;
    };

    ///继承的目标检测的类-------------------------------------------------------------------------------------
    class Detect_Publisher : public TF_Publisher_Base, public rclcpp::Node {
    public:
        Detect_Publisher();

        ~Detect_Publisher() override = default;

    private:
        void publish_transform() override;

        void position_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

        //自定义接口信息发送
        rclcpp::Publisher<camera_info::msg::Position>::SharedPtr position_pub_;

        //接受点的信息
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr position_subscription;

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

        std::shared_ptr<CalculateTarget> calculate_target_class_;
        //openmv的信息
        geometry_msgs::msg::TransformStamped transform_openmv_;
    };
}


#endif //TF_PUBLISH_H
