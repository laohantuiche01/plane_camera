#include "../../include/camera_data_handle/camera_handle.h"
#include "../../include/camera_data_handle/target_predict.h"
#include "../../include/tf_publish/tf_publish.h"
#include "../../include/kalman_moving_target/kalman.hpp"

#include "rclcpp/rclcpp.hpp"
#include "yaml-cpp/yaml.h"
#include "sensor_msgs/msg/image.hpp"
#include "opencv4/opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"
#include <openvino/openvino.hpp>

#define STR(s) #s
#define MACRO_TO_STR(s) STR(s)

constexpr uint8_t KALMAN_MODEL = kalman::Kalman::CVMODE; // 选择模型：CVMODE(8维状态)/CAMODE(9维状态)
constexpr uint16_t MAX_PREDICT_STEP = 3; // 最大预测步数（无观测时最多预测5次）
constexpr double KALMAN_PERIOD = 0.05; // 卡尔曼运行周期（50ms，即20Hz，匹配传感器频率）
constexpr double PROCESS_NOISE = 0.0001; // 过程噪声（模型不确定性，值越小越信任模型）
constexpr double MEAS_NOISE = 0.001; // 测量噪声（观测不确定性，值越小越信任传感器）


kalman::KalmanInput camera::transform_to_karman_input(const Yolov8::Detection &detection) {
    kalman::KalmanInput kalman{};
    kalman.x = detection.box.x;
    kalman.y = detection.box.y;
    kalman.w = detection.box.width;
    kalman.h = detection.box.height;
    return kalman;
}

Eigen::VectorXd camera::transform_to_eigen_vector(const kalman::KalmanInput &input) {
    Eigen::VectorXd eigen_vector;
    eigen_vector = Eigen::VectorXd::Zero(4);
    eigen_vector << input.x, input.y, input.w, input.h;
    return eigen_vector;
}

camera::ReceiveData::ReceiveData() : Node("receive_data"),
                                     detector_(
                                         MACRO_TO_STR(PROJECT_PATH)"/model/best.onnx") {
    RCLCPP_INFO(this->get_logger(), "Receive Data");
    RCLCPP_INFO(this->get_logger(), MACRO_TO_STR(PROJECT_PATH)"/model/best.onnx");

#ifdef VIDEO_WRITE
    writer_.open(video_name_,
                 cv::VideoWriter::fourcc('M', 'J', 'P', 'G'),
                 10.0,
                 Size(640, 480),
                 true
    );
#endif

#ifdef KALMAN_OPEN
    kf_ = std::make_shared<kalman::Kalman>(KALMAN_MODEL, MAX_PREDICT_STEP); //创建对象
    kf_->T_set(KALMAN_PERIOD); //设置运行周期
    kf_->Q_set_2d_CV(PROCESS_NOISE); //设置过程噪声
    kf_->R_set_2d(MEAS_NOISE); //设置观测噪声
    Eigen::MatrixXd init_P(8, 8);
    init_P.setIdentity();
    init_P *= 8.0;
    kf_->P_init(init_P);
#ifdef KALMAN_OPEN_DEBUG
    measure_pub_ = this->create_publisher<Measure>("measure", 10);
    kalman_pub_ = this->create_publisher<KalmanOutput>("kalman", 10);
#endif
#endif

    // this->declare_parameter("nms_threshold_", 0.4);
    // this->declare_parameter("confidence_threshold_", 0.5);
    //
    // this->get_parameter("confidence_threshold_", confidence_threshold_);
    // this->get_parameter("nms_threshold_", nms_threshold_);

    position_pub_ = this->create_publisher<robot_interfaces::msg::ImageLocation>(
        "/camera/target/position",
        10
    );
    image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/camera/color/image_raw",
        10,
        std::bind(&ReceiveData::imageCallback, this, std::placeholders::_1)
    );
    timer_ = this->create_wall_timer(
        std::chrono::seconds(1), [this]() {
            if (!has_received_) {
                RCLCPP_WARN(this->get_logger(), "No Image Receive");
            } else {
                RCLCPP_INFO(this->get_logger(), "Has been receive!");
                timer_->cancel();
            }
        });
}

void camera::ReceiveData::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg) {
    has_received_ = true;
    try {
        fps_sum_++;
        if (msg->width <= 0 || msg->height <= 0) {
            RCLCPP_WARN(this->get_logger(), "Invalid image dimensions: %dx%d", msg->width, msg->height);
            return;
        }

#ifdef FPS_VISABLE_OPEN
        fps_timer_.start(); //计时以计算帧率
#endif

        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
        cv::Mat temp_image = cv_ptr->image;
        cv::Mat image;
        cv::cvtColor(temp_image, image, cv::COLOR_BGR2RGB);

#ifdef VIDEO_WRITE
        writer_.write(image);
#endif

#ifndef YOLOV8_DETECTOR_OFF
        //目标检测接口
        std::vector<Yolov8::Detection> detections = detector_.detect(image);
#ifdef KEY_POINT_TRACKING
        if (!detections.empty()) if_do_tracking_ = true;

        if (if_do_tracking_ && detections.empty()) {
            detections = detector_.track(image);
            if (detections.empty()) if_do_tracking_ = false;
        }
#endif

#ifdef KALMAN_OPEN
        if (!detections.empty() || kalman_step_ != 0) {
            kalman::KalmanInput kalman = transform_to_karman_input(detections.at(0));
            current_meas_ = transform_to_eigen_vector(kalman);
            if (!detections.empty()) is_meas_unsuccessful_ = false; //观测成功
            else {
                is_meas_unsuccessful_ = true; //观测失败
                kalman_step_++;
                if (kalman_step_ == 5) kalman_step_ = 0;
            }

            Measure measure;
            measure.x = current_meas_[0];
            measure.y = current_meas_[1];
            measure.w = current_meas_[2];
            measure.h = current_meas_[3];
#ifdef KALMAN_OPEN_DEBUG
            measure_pub_->publish(measure);
#endif
            kf_result_ = kf_->kalman_filter(
                is_meas_unsuccessful_,
                current_meas_,
                std::nullopt
            );

            KalmanOutput output;
            output.x = kf_result_.input.x;
            output.y = kf_result_.input.y;
            output.w = kf_result_.input.w;
            output.h = kf_result_.input.h;
            output.vx = kf_result_.v_x;
            output.vy = kf_result_.v_y;
            output.vw = kf_result_.v_w;
            output.vh = kf_result_.v_h;
#ifdef KALMAN_OPEN_DEBUG
            kalman_pub_->publish(output);
#endif

            Rect kalman_box;
            kalman_box.x = static_cast<int>(kf_result_.input.x);
            kalman_box.y = static_cast<int>(kf_result_.input.y);
            kalman_box.width = static_cast<int>(kf_result_.input.w);
            kalman_box.height = static_cast<int>(kf_result_.input.h);
            cv::rectangle(image, kalman_box, Scalar(255, 0, 0), 2);
            std::cout << "  状态（x,y,w,h）：" << kf_result_.input.x << ", "
                    << kf_result_.input.y << ", " << kf_result_.input.w << ", " << kf_result_.input.h << std::endl;
            std::cout << "  速度（vx,vy,vw,vh）：" << kf_result_.v_x << ", "
                    << kf_result_.v_y << ", " << kf_result_.v_w << ", " << kf_result_.v_h << std::endl;
            std::cout << "  噪声指标（sigma）：" << kf_result_.sigma << " | 滤波有效？"
                    << (kf_result_.is_success ? "是" : "否") << std::endl;
        }

#endif
        std::vector<std::vector<double> > positions = detector_.drawDetections(image, detections);
#endif


#ifdef YOLOV8_DETECTOR_OFF

        std::vector<std::vector<double> > positions{};
#endif

#ifdef PREDICT_OPEN
        //卡尔曼滤波器接口
        if (!detections.empty()) {
            tracking_num_ = 1;
        }
        try {
            if (tracker_ == nullptr && tracking_num_) {
                tracker_ = new KalmanBoxTracker(detections.front().box);
            } else if (tracking_num_) {
                if (!detections.empty()) {
                    //RCLCPP_ERROR(this->get_logger(), "-----------------------------!!!!!");
                    Rect input_rect = detections.front().box;
                    tracker_->update(input_rect);
                }
                Point2f predictCenter = tracker_->predict();
                Rect estimatedBox = tracker_->get_state();
        //RCLCPP_ERROR(this->get_logger(), "-----------------------------");

#ifndef NO_IMAGE
        circle(image, predictCenter, 5, Scalar(200, 0, 120), 2);
        cv::rectangle(image, estimatedBox, Scalar(255, 0, 0), 2);
#endif

        // cv::Point2d new_center = target_predict_factory_.StartPredict({predictCenter.x, predictCenter.y},
        //                                                               std::chrono::system_clock::now().
        //                                                               time_since_epoch().count());
        // circle(image, new_center, 5, Scalar(200, 20, 120), 2);
            }
        } catch (cv::Exception e) {
            RCLCPP_WARN(this->get_logger(), "%s", e.what());
        }

        if (detections.empty() && tracking_num_) {
            tracking_num_++;
            if (tracking_num_ == 10) {
                tracking_num_ = 0;
            }
        }

#endif

        if (!detections.empty()) {
            RCLCPP_INFO(this->get_logger(), "Received Image %s",
                        Yolov8::defaultClassNames[detections.front().classId].c_str());
        }

        //信息发送变量
        robot_interfaces::msg::ImageLocation position_msg;

        if (positions.empty()) {
            RCLCPP_WARN(this->get_logger(), "No detections found");
            position_msg.image_x = 0;
            position_msg.image_y = 0;
            position_msg.id = UNKNOW;
            position_pub_->publish(position_msg);
        } else {
            while (!positions.empty()) {
                position_msg.image_x = static_cast<float>(detections.front().box.x);
                position_msg.image_y = static_cast<float>(detections.front().box.y);
                position_msg.id = detections.front().classId;
                positions.pop_back();
                position_pub_->publish(position_msg);
            }
        }
#ifndef NO_IMAGE
        // cv::circle(image, cv::Point(320, 240), 207, cv::Scalar(0, 255, 0), 1);
        // cv::line(image, cv::Point(0, 240), cv::Point(640, 240), cv::Scalar(0, 255, 0), 1);
        // cv::line(image, cv::Point(320, 0), cv::Point(320, 480), cv::Scalar(0, 255, 0), 1);
        cv::circle(image, cv::Point(320, 240), 207, cv::Scalar(0, 255, 0), 1);
        cv::line(image, cv::Point(0, 240), cv::Point(640, 240), cv::Scalar(0, 255, 0), 1);
        cv::line(image, cv::Point(320, 0), cv::Point(320, 480), cv::Scalar(0, 255, 0), 1);
#endif

#ifdef FPS_VISABLE_OPEN
        //计算fps的
        {
            fps_timer_.stop();
            double currentTime = fps_timer_.getTimeSec();
            fps_time_sum_ += currentTime;
            double fps = fps_sum_ / fps_time_sum_;
            std::stringstream ss;
            ss << "FPS: " << std::fixed << std::setprecision(2) << fps;
            cv::putText(image, ss.str(), cv::Point(10, 30),
                        cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);
        }
#endif

#ifndef NO_IMAGE
        namedWindow("image", cv::WINDOW_NORMAL);
        cv::resizeWindow("image", 2000, 1500);
#endif

#ifdef FPS_VISABLE_OPEN
        fps_timer_.reset();

        if (fps_sum_ == 20) {
            fps_sum_ = 0;
            fps_time_sum_ = 0;
        }
#endif

#ifndef NO_IMAGE
        cv::imshow("image", image);
        cv::waitKey(1);
#endif
    } catch (cv_bridge::Exception &e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Exception in image callback: %s", e.what());
    } catch (...) {
        RCLCPP_ERROR(this->get_logger(), "Unknown exception in image callback");
    }
}
