#include "../../include/receive_USB_camera/receive_USB_camera.hpp"

#define OPENMV_NULL_ERROR 321

#define IMAGE_WIDTH 640
#define IMAGE_HEIGHT 480

#define HORIZON_X 23.5
#define HORIZON_Z 18.4

#define INPUT_ANGLE(s) ((s)*M_PI/180)

constexpr double HORIZONTAL_ANGLE = 60.0;
constexpr double HORIZONTAL_FOV = 60.0 * M_PI / 180.0;
constexpr double VERTICAL_FOV = HORIZONTAL_FOV * (IMAGE_HEIGHT / (double) IMAGE_WIDTH);

using namespace std;
using namespace cv;


receive_USB::receive_USB() : Node("USB_pub") {
    color_lower_ = Scalar(0, 0, 0);
    color_upper_ = Scalar(180, 255, 255);
    usb_camera_ = std::make_shared<usb_camera::USBCamera>(2);
    usb_camera_->OpenCameraDevice();
    usb_camera_->SetExposure(300);
    pub_ = this->create_publisher<sensor_msgs::msg::Image>("/camera/camera/color/image_raw", 10);
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(30),
        std::bind(&receive_USB::callback, this)
    );
}

void receive_USB::callback() {
    sensor_msgs::msg::Image img_msg;
    cv::Mat frame;

    frame = usb_camera_->GetFrame();
    image_ = frame.clone();
    Receive_Keypoint();
    cv::cvtColor(frame, frame, cv::COLOR_BGR2RGB);
    img_msg.encoding = "rgb8";
    img_msg.header.frame_id = "camera";
    img_msg.width = frame.cols;
    img_msg.height = frame.rows;
    img_msg.step = frame.step;

    size_t size = frame.step * frame.rows;
    img_msg.data.resize(size);
    memcpy(&img_msg.data[0], frame.data, size);

    img_msg.header.stamp = rclcpp::Clock().now();

    pub_->publish(img_msg);
}

cv::Point2f receive_USB::Receive_Keypoint() {
    Mat hsv, mask;
    image_ = usb_camera_->GetFrame();
    cvtColor(image_, hsv, COLOR_BGR2HSV);
    inRange(hsv, color_lower_, color_upper_, mask);

    Mat kernel = getStructuringElement(MORPH_ELLIPSE, Size(5, 5));
    erode(mask, mask, kernel);
    dilate(mask, mask, kernel);

    vector<vector<Point> > contours;
    findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    vector<Point> max_contours;
    double max_contour_score = 200;

    for (size_t i = 0; i < contours.size(); i++) {
        if (contourArea(contours[i]) > max_contour_score) {
            max_contour_score = contours[i].size();
            max_contours = contours[i];
        }

        Rect bounding_rect = boundingRect(max_contours);

        rectangle(image_, bounding_rect, Scalar(0, 255, 0), 2);

        putText(image_, "Blue Object", Point(bounding_rect.x, bounding_rect.y - 10),
                FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0), 2);
    }

    Point2f output;
    output.x = (max_contours.at(0).x + max_contours.at(2).x) / 2;
    output.y = (max_contours.at(0).y + max_contours.at(1).y) / 2;
    imshow("11111", image_);
    waitKey(30);
    return output;
}

///--------------------------------------------------------------------------------------------------------------
usb_camera::USBFactor::USBFactor() {
    // color_lower_ = Scalar(168, 68, 82);
    // color_upper_ = Scalar(180, 196, 210);
    color_lower_ = Scalar(68, 73, 112);
    color_upper_ = Scalar(86, 181, 255);
    usb_camera_ = std::make_shared<usb_camera::USBCamera>(2);
    usb_camera_->OpenCameraDevice();
    usb_camera_->SetExposure(300);
}

cv::Point2d usb_camera::USBFactor::Receive_Keypoint() {
    cv::Mat hsv, mask;
    image_ = usb_camera_->GetFrame();
    cvtColor(image_, hsv, COLOR_BGR2HSV);
    inRange(hsv, color_lower_, color_upper_, mask);

    Mat kernel = getStructuringElement(MORPH_ELLIPSE, Size(8, 5));
    erode(mask, mask, kernel);
    dilate(mask, mask, kernel);

    vector<vector<Point> > contours;
    findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    vector<Point> max_contours;
    double max_contour_score = 200;

    for (const auto &contour: contours) {
        if (contourArea(contour) > max_contour_score) {
            max_contour_score = contour.size();
            max_contours = contour;
        }

        Rect bounding_rect = boundingRect(max_contours);

        rectangle(image_, bounding_rect, Scalar(0, 255, 0), 2);

        putText(image_, "Red_cross", Point(bounding_rect.x, bounding_rect.y - 10),
                FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0), 2);
    }

    Point2d output;
    if (max_contours.empty()) {
        output.x = 0;
        output.y = 0;
    } else {
        output.x = static_cast<double>(max_contours.at(0).x + max_contours.at(2).x) / 2;
        output.y = static_cast<double>(max_contours.at(0).y + max_contours.at(1).y) / 2;
    }
    imshow("hsv_", mask);
    imshow("11111", image_);
    waitKey(30);
    return output;
}


Point2d usb_camera::USBFactor::Transform_Image_TO_Real(cv::Point2d &image_point,
                                                       geometry_msgs::msg::TransformStamped pose) {
    double height = pose.transform.translation.z + 0.39; // 无人机高度 + 相机安装高度
    double theta_x, theta_z;
    double length;
    double cam_pitch;
    double real_x, real_y;

/// --------------------------------转换在这里转的--------------------------------------------
    double w = pose.transform.rotation.w;
    double z = pose.transform.rotation.x;
    double x = pose.transform.rotation.y;
    double y = pose.transform.rotation.z;

    std::cerr << "w=" << w << "  x=" << x << "  y=" << y << "  z=" << z << std::endl;

    std::cerr << "X=" << pose.transform.translation.x << "  Y=" << pose.transform.translation.y << "  Z=" << pose.
            transform.translation.z << std::endl;

    Eigen::Vector3d r_C; //相机系的向量
    Eigen::Quaterniond q(w, x, y, z); //构造的四元数
    Eigen::Matrix3d R_WD; //由四元数得到的旋转矩阵
    Eigen::Matrix3d R_DC; //相机对于无人机系的偏执
    Eigen::Matrix3d R_WC; //总的偏执矩阵
    Eigen::Vector3d r_W; //世界系中的相机向量
    Eigen::Vector3d target_W; //目标的世界向量

    cv::Point image_center(IMAGE_WIDTH / 2, IMAGE_HEIGHT / 2);
    theta_x = INPUT_ANGLE(HORIZON_X) * (image_point.x - image_center.x) / image_center.x;
    theta_z = INPUT_ANGLE(HORIZON_Z) * (image_center.y - image_point.y) / image_center.y;

    //相机系方向向量
    r_C.x() = tan(theta_x);
    r_C.y() = tan(theta_z);
    r_C.z() = 1.0; //这里的 1.0 代表以1为单位高度
    r_C.normalize();

    //旋转矩阵（无人机系到世界系）
    R_WD = q.normalized().toRotationMatrix();

    //相机相对于无人机的固定旋转
    //绕X轴旋转
    cam_pitch = -M_PI / 4; // -30度
    //cam_pitch = 0;

    R_DC = Eigen::AngleAxisd(cam_pitch, Eigen::Vector3d::UnitX()) // roll
           * Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) // pitch
           * Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ()); // yaw

    //TFDebug tf2("camera","plane",R_DC);

    //相机系到世界系的变换
    R_WC = R_WD * R_DC;

#ifdef RVIZ_DEBUG
    tf_debug_ptr_->SetTransform(R_WC);
    tf_debug_ptr_->broadcast_transform();
#endif

    //方向向量转换到世界系
    r_W = R_WC * r_C;

    //计算目标在世界系中的位置
    double t = height / r_W.z();
    // if (t < 0) {
    //     return {OPENMV_NULL_ERROR,OPENMV_NULL_ERROR};
    // }
    target_W.x() = t * r_W.x();
    target_W.y() = t * r_W.y();
    target_W.z() = 0.0; //地面高度为0

    //转换到无人机系
    // Eigen::Vector3d drone_pos(pose.transform.translation.x,
    //                           pose.transform.translation.y,
    //                           pose.transform.translation.z);

    //Eigen::Vector3d target_D = R_WD.transpose() * (target_W - drone_pos);

    Eigen::Vector3d target_D = R_WD.transpose() * target_W;

    real_x = target_D.x();
    real_y = target_D.y();

    return {target_W.x(), target_W.y()};
}
