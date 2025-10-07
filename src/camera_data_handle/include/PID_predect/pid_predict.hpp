#ifndef CAMERA_DATA_HANDLE_PID_PREDICT_HPP
#define CAMERA_DATA_HANDLE_PID_PREDICT_HPP

#include <vector>
#include <chrono>
#include <tuple>
#include <opencv2/opencv.hpp>

#include <vector>
#include <chrono>
#include <tuple>
#include <Eigen/Dense>
#include <geometry_msgs/msg/detail/point_stamped__builder.hpp>
#include <geometry_msgs/msg/detail/transform_stamped__struct.hpp>
#include <geometry_msgs/msg/detail/twist__struct.hpp>

struct TargetPosition {
    double x; // X像素坐标
    double y; // Y像素坐标
    std::chrono::high_resolution_clock::time_point timestamp;

    TargetPosition(double x_, double y_)
        : x(x_), y(y_), timestamp(std::chrono::high_resolution_clock::now()) {
    }

    TargetPosition(double x_, double y_, std::chrono::high_resolution_clock::time_point ts)
        : x(x_), y(y_), timestamp(ts) {
    }
};

class TargetSpeedEstimator {
private:
    const int img_width;
    const int img_height;
    // 存储历史位置的缓冲区
    std::vector<TargetPosition> position_history;
    size_t max_history_size; // 最大历史记录数量
    // 速度计算参数
    double min_time_interval; // 计算速度的最小时间间隔
    double max_position_jump; // 认为是异常值的最大位置跳变
    // 滤波参数
    double smoothing_factor; // 0-1，值越大越信任新数据
    // 内部状态
    double current_vx; // 当前X方向速度(像素/秒)
    double current_vy; // 当前Y方向速度(像素/秒)
    // 计算两个时间戳之间的秒数
    double calculate_time_diff(const std::chrono::high_resolution_clock::time_point &t1,
                               const std::chrono::high_resolution_clock::time_point &t2);

    // 检查位置是否为异常值
    bool is_outlier(const TargetPosition &new_pos);

    // 使用指数平滑过滤速度
    void smooth_speed(double &current, double new_value);

public:
    // 构造函数
    explicit TargetSpeedEstimator(int width = 640, int height = 480);

    // 设置参数
    void set_max_history_size(size_t size);

    void set_min_time_interval(double interval);

    void set_max_position_jump(double jump);

    void set_smoothing_factor(double factor);

    // 添加新的目标位置并更新速度估计
    void update_position(double x, double y);

    // 获取当前估计的速度
    std::tuple<double, double> get_speed() const;

    // 获取历史位置记录
    const std::vector<TargetPosition> &get_history() const;

    // 重置估计器
    void reset();
};

class V_Predict {
private:
    TargetSpeedEstimator estimator_;
    geometry_msgs::msg::Twist plane_velocity_; //无人机的速度向量
    geometry_msgs::msg::TransformStamped camera_pose_; //可以得到高度
    std::vector<TargetPosition> position_history_; //储存之前的数据

    std::tuple<double, double> Camera_Speed_To_Real(double height, double x, double y);

    std::tuple<double, double> Calculate_dv(double dx_1, double dy_1, double dx_2, double dy_2);

public:
    V_Predict();

    cv::Point2f Predict(int x, int y,
                        geometry_msgs::msg::Twist twist,
                        geometry_msgs::msg::TransformStamped pose);

    cv::Point2f Predict(int x, int y);
};


#endif //CAMERA_DATA_HANDLE_PID_PREDICT_HPP
