#include "../../include/PID_predect/pid_predict.hpp"

#include <cmath>
#include <algorithm>
#include <stdexcept>

TargetSpeedEstimator::TargetSpeedEstimator(int width, int height)
    : img_width(width),
      img_height(height),
      max_history_size(10),
      min_time_interval(0.01), // 10毫秒
      max_position_jump(100.0), // 100像素
      smoothing_factor(0.3), // 平滑因子
      current_vx(0.0),
      current_vy(0.0) {
}

void TargetSpeedEstimator::set_max_history_size(size_t size) {
    if (size < 2) {
        throw std::invalid_argument("历史记录数量必须至少为2");
    }
    max_history_size = size;
    // 截断历史记录到新的最大尺寸
    while (position_history.size() > max_history_size) {
        position_history.erase(position_history.begin());
    }
}

void TargetSpeedEstimator::set_min_time_interval(double interval) {
    if (interval <= 0) {
        throw std::invalid_argument("时间间隔必须为正数");
    }
    min_time_interval = interval;
}

void TargetSpeedEstimator::set_max_position_jump(double jump) {
    if (jump <= 0) {
        throw std::invalid_argument("位置跳变阈值必须为正数");
    }
    max_position_jump = jump;
}

void TargetSpeedEstimator::set_smoothing_factor(double factor) {
    if (factor < 0 || factor > 1) {
        throw std::invalid_argument("平滑因子必须在0到1之间");
    }
    smoothing_factor = factor;
}

double TargetSpeedEstimator::calculate_time_diff(
    const std::chrono::high_resolution_clock::time_point &t1,
    const std::chrono::high_resolution_clock::time_point &t2) {
    auto duration = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1);
    return duration.count();
}

bool TargetSpeedEstimator::is_outlier(const TargetPosition &new_pos) {
    if (position_history.empty()) {
        return false; // 没有历史数据，无法判断
    }

    // 与最后一个位置比较
    const auto &last_pos = position_history.back();
    double dx = new_pos.x - last_pos.x;
    double dy = new_pos.y - last_pos.y;
    double distance = std::sqrt(dx * dx + dy * dy);

    // 计算时间差
    double dt = calculate_time_diff(last_pos.timestamp, new_pos.timestamp);

    // 如果时间差很小但位置变化很大，则认为是异常值
    if (dt < min_time_interval && distance > max_position_jump) {
        return true;
    }

    return false;
}

void TargetSpeedEstimator::smooth_speed(double &current, double new_value) {
    current = (1 - smoothing_factor) * current + smoothing_factor * new_value;
}

void TargetSpeedEstimator::update_position(double x, double y) {
    // 检查坐标是否在图像范围内
    if (x < 0 || x >= img_width || y < 0 || y >= img_height) {
        throw std::invalid_argument("目标位置超出图像范围");
    }

    // 创建新位置记录
    TargetPosition new_pos(x, y);

    // 检查是否为异常值
    if (is_outlier(new_pos)) {
        // 是异常值，不添加到历史记录，也不更新速度
        return;
    }

    // 添加到历史记录
    position_history.push_back(new_pos);

    // 保持历史记录大小
    while (position_history.size() > max_history_size) {
        position_history.erase(position_history.begin());
    }

    // 计算速度（需要至少两个位置）
    if (position_history.size() >= 2) {
        // 使用最近的两个位置计算原始速度
        const auto &prev_pos = position_history[position_history.size() - 2];
        double dt = calculate_time_diff(prev_pos.timestamp, new_pos.timestamp);

        // 确保时间差有效
        if (dt >= min_time_interval) {
            double raw_vx = (new_pos.x - prev_pos.x) / dt;
            double raw_vy = (new_pos.y - prev_pos.y) / dt;

            // 应用平滑
            smooth_speed(current_vx, raw_vx);
            smooth_speed(current_vy, raw_vy);
        }
    }
}

std::tuple<double, double> TargetSpeedEstimator::get_speed() const {
    return std::make_tuple(current_vx, current_vy);
}

const std::vector<TargetPosition> &TargetSpeedEstimator::get_history() const {
    return position_history;
}

void TargetSpeedEstimator::reset() {
    position_history.clear();
    current_vx = 0.0;
    current_vy = 0.0;
}

V_Predict::V_Predict() {
    estimator_.set_max_history_size(8);
    estimator_.set_smoothing_factor(0.1);
    estimator_.set_max_position_jump(30.0);
}

std::tuple<double, double> V_Predict::Calculate_dv(double dx_1, double dy_1, double dx_2, double dy_2) {
    return std::make_tuple(dx_1 - dx_2, dy_1 - dy_2);
}

std::tuple<double, double> V_Predict::Camera_Speed_To_Real(double height, double x, double y) {
    double rate = 0.5;
    height += 0.15;
    double x1 = height * rate * x * 35 / (12 * 207);
    double y1 = height * rate * y * 35 / (12 * 207);
    std::cout << x1 << "    " << y1 << std::endl;
    return std::make_tuple(x1, y1); //h:105   x:36
}


cv::Point2f V_Predict::Predict(int x, int y, geometry_msgs::msg::Twist twist,
                               geometry_msgs::msg::TransformStamped pose) {
    double dt = 10;
    estimator_.update_position(x, y);
    camera_pose_.transform = pose.transform;
    plane_velocity_ = twist;
    auto [camera_dx,camera_dy] = estimator_.get_speed();
    auto [plane_dx,plane_dy] = Camera_Speed_To_Real(camera_pose_.transform.translation.z, camera_dx, camera_dy);
    auto [dx,dy] = Calculate_dv(plane_dx, plane_dy, twist.linear.x, twist.linear.y);
    return {static_cast<float>(dx * dt), static_cast<float>(dy * dt)};
}

cv::Point2f V_Predict::Predict(int x, int y) {
    double dt = 10;
    estimator_.update_position(x, y);
    auto [camera_dx,camera_dy] = estimator_.get_speed();
    auto [plane_dx,plane_dy] = Camera_Speed_To_Real(1, camera_dx, camera_dy);
    auto [dx,dy] = Calculate_dv(plane_dx, plane_dy, 10, 10);
    return {static_cast<float>(dx * dt), static_cast<float>(dy * dt)};
}
