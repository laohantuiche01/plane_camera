#include "../../include/receive_openmv_data/receive_openmv_data.h"

#define OPENMV_NULL_ERROR 321

#define IMAGE_WIDTH 320
#define IMAGE_HEIGHT 240

#define INPUT_ANGLE(s) ((s)*M_PI/180)

const double HORIZONTAL_ANGLE = 60.0;
const double HORIZONTAL_FOV = 60.0 * M_PI / 180.0;
const double VERTICAL_FOV = HORIZONTAL_FOV * (IMAGE_HEIGHT / (double) IMAGE_WIDTH);

ReceiveOpenMVData::ReceiveOpenMVData(const std::string port,
                                     const speed_t baudRate) : port_(port), baudRate_(baudRate) {
}

std::string ReceiveOpenMVData::Receive_Openmv_Data() {
    int fd = openSerialPort(port_, baudRate_);;

    int num = 0;

    try {
        while (true) {
            std::string line = readLine(fd);

            uint8_t byte_data;
            ssize_t n = read(fd, &byte_data, 1);

            // if (n > 0) {
            //     std::cout << "收到字节: 0x" << std::hex << static_cast<int>(byte_data) << std::dec << std::endl;
            //
            //     std::string bit_string;
            //     for (int i = 7; i >= 0; --i) {
            //         bit_string += ((byte_data >> i) & 1) ? '1' : '0';
            //     }
            //     std::cout << "按位解码: " << bit_string << std::endl;
            //     std::cout << "---------------------------" << std::endl;
            // }

            if (!line.empty()) {
                std::cout << "收到信息: " << line << std::endl;
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
                return line;
            }
            //num++;
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    } catch (const std::exception &e) {
        std::cerr << "发生错误: " << e.what() << std::endl;
    } catch (...) {
        std::cerr << "程序已停止" << std::endl;
    }

    close(fd);
    std::cout << "串口已关闭" << std::endl;

    return "0";
}

int ReceiveOpenMVData::openSerialPort(const std::string &port, speed_t baudRate) {
    struct termios tty;
    int fd = -1;
    int retryCount = 0;
    const int maxRetries = 10;

    while (retryCount < maxRetries) {
        fd = open(port.c_str(), O_RDWR | O_NOCTTY);
        if (fd == -1) {
            std::cerr << "无法打开串口: " << port << "，错误: " << strerror(errno) << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            retryCount++;
            continue;
        }

        if (!isatty(fd)) {
            std::cerr << port << " 不是一个终端设备" << std::endl;
            close(fd);
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            retryCount++;
            continue;
        }

        if (tcgetattr(fd, &tty) == -1) {
            std::cerr << "无法获取串口属性，错误: " << strerror(errno) << std::endl;
            close(fd);
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            retryCount++;
            continue;
        }

        break;
    }

    if (fd == -1 || retryCount >= maxRetries) {
        std::cerr << "达到最大重试次数，无法打开串口" << std::endl;
        return -1;
    }

    memset(&tty, 0, sizeof(tty));

    if (cfsetospeed(&tty, baudRate) == -1 || cfsetispeed(&tty, baudRate) == -1) {
        std::cerr << "无法设置波特率，错误: " << strerror(errno) << std::endl;
        close(fd);
        return -1;
    }

    tty.c_cflag &= ~PARENB; // 禁用奇偶校验
    tty.c_cflag &= ~CSTOPB; // 1停止位
    tty.c_cflag &= ~CSIZE; // 清除数据位设置
    tty.c_cflag |= CS8; // 8位数据位

    // 禁用硬件流控制
    tty.c_cflag &= ~CRTSCTS;

    // 启用接收器，设置本地模式
    tty.c_cflag |= (CLOCAL | CREAD);

    // 禁用软件流控制
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);

    // 禁用输入处理（不转换回车换行等）
    tty.c_iflag &= ~(ICRNL | INLCR | IGNCR);

    // 设置原始输入模式
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

    // 设置原始输出模式
    tty.c_oflag &= ~OPOST;
    tty.c_oflag &= ~(ONLCR | OCRNL);

    // 调整超时设置 - 等待至少1个字符，超时1秒
    tty.c_cc[VTIME] = 10; // 1秒超时 (10 * 0.1秒)
    tty.c_cc[VMIN] = 1; // 至少读取1个字符

    // 清空输入输出缓冲区
    tcflush(fd, TCIFLUSH);

    // 应用配置
    if (tcsetattr(fd, TCSANOW, &tty) == -1) {
        std::cerr << "无法设置串口属性，错误: " << strerror(errno) << std::endl;
        close(fd);
        return -1;
    }

    return fd;
}

std::string ReceiveOpenMVData::readLine(int fd) {
    if (fd < 0) {
        return "";
    }

    std::string line;
    char c;
    ssize_t n;

    while (true) {
        n = read(fd, &c, 1);

        if (n == -1) {
            // 处理读取错误
            if (errno != EAGAIN && errno != EWOULDBLOCK) {
                std::cerr << "读取串口错误: " << strerror(errno) << std::endl;
            }
            break;
        } else if (n == 0) {
            // 没有读取到数据，超时
            break;
        } else {
            // 处理换行符和回车符
            if (c == '\n' || c == '\r') {
                if (!line.empty()) {
                    // 避免空行
                    break;
                }
                // 如果是空行则继续读取
                continue;
            }
            line += c;
        }
    }

    return line;
}


CalculateTarget::CalculateTarget() {
    receive_openmv_data_ = std::make_shared<ReceiveOpenMVData>();
}

cv::Point2d CalculateTarget::Decode_Openmv_Data(std::string &input_str) {
    double point_x = 0, point_y = 0;
    try {
        if (input_str == "null") {
            return {OPENMV_NULL_ERROR, OPENMV_NULL_ERROR};
        }
        nlohmann::json j = nlohmann::json::parse(input_str);
        point_x = j["cx"];
        point_y = j["cy"];
    } catch (const std::exception &e) {
        std::cerr << "发生错误" << e.what() << std::endl;
    }
    return {point_x, point_y};
}

cv::Point2d CalculateTarget::Transform_Image_TO_Real(cv::Point2d &image_point, double height) {
    // double the_camera_center = 0.05; //焦点距离影响平面的距离
    // double the_camera_length = 0.05; //相机下边界长度
    // double real_x{0} ; //实际的x
    // double real_y{0} ; //实际的y
    // double theta_below{0}; //下边界角度
    // double theta_above{0}; //上边界角度
    // double length_below{0};
    // double length_above{0};
    //
    // length_below = the_camera_length*(the_camera_center+height)/(sin(theta_below)*the_camera_center);
    // length_above = the_camera_length*(the_camera_center+height)/(sin(theta_above)*the_camera_center);
    //
    // // 把相机高度转化成焦点高度 （这个比例要调the_camera_center）
    // double height_center = height + the_camera_center;
    // cv::Point2d image_point_c = cv::Point2d(image_point.x - 160, image_point.y - 120);
    //
    // real_x = image_point_c.x * height_center / 100;
    // real_y = (1 + image_point_c.y / 100) * height_center;
    //
    // return {real_x, real_y};
    //cv::Point image_center(IMAGE_WIDTH / 2, IMAGE_HEIGHT / 2);

    //计算垂直方向像素偏移比例
    // double vertical_ratio = (image_point.y - image_center.y) / (double) (IMAGE_HEIGHT / 2);
    //
    // //距离=中心距离/cos(垂直角度)
    // double vertical_angle = vertical_ratio * (VERTICAL_FOV / 2.0);
    // double distance = height / cos(vertical_angle);
    //
    // //计算水平方向像素偏移比例
    // double horizontal_ratio = (image_point.x - image_center.x) / (double) (IMAGE_WIDTH / 2);
    //
    // double horizontal_angle = horizontal_ratio * (HORIZONTAL_FOV / 2.0);
    //
    // //计算实际x,y坐标
    // double x = distance * sin(horizontal_angle);
    // double y = distance * sin(vertical_angle);


    //上下的视场与视觉中心大概差20～30度
    //左右的视场与视觉中心大概差30～35度

    double x_min;
    double x_max;
    double y_min;
    double y_max;
    double temp_x;
    double temp_y;
    double x = 0;
    double y = 0;

    cv::Point image_center(IMAGE_WIDTH / 2, IMAGE_HEIGHT / 2);

    y_min = tan(INPUT_ANGLE(HORIZONTAL_ANGLE - 23)) * height;
    y_max = tan(INPUT_ANGLE(HORIZONTAL_ANGLE + 23)) * height;

    x_min = height * tan(INPUT_ANGLE(32)) / cos(INPUT_ANGLE(HORIZONTAL_ANGLE - 23));
    x_max = height * tan(INPUT_ANGLE(32)) / cos(INPUT_ANGLE(HORIZONTAL_ANGLE + 23));

    temp_x = x_max - x_min;

    x = (x_min + temp_x * ((IMAGE_HEIGHT - image_point.y) / IMAGE_HEIGHT)) * (
            (image_point.x - IMAGE_WIDTH / 2) / (IMAGE_WIDTH / 2)) / 2;


    temp_y = y_max - y_min;

    y = y_min + temp_y * ((IMAGE_HEIGHT - image_point.y) / IMAGE_HEIGHT);

    return {x, y};
}

cv::Point2d CalculateTarget::Handle_Openmv_Data(double height) {
    while (true) {
        std::string input_str = receive_openmv_data_.get()->Receive_Openmv_Data();
        cv::Point2d temp_point = Decode_Openmv_Data(input_str);

        if (temp_point == cv::Point2d(OPENMV_NULL_ERROR, OPENMV_NULL_ERROR)) {
            //std::cout << temp_point.x << " " << temp_point.y << std::endl;
            return temp_point;
            continue;
        }

        cv::Point2d output_point = Transform_Image_TO_Real(temp_point, height);
        //std::cout << output_point.x << " " << output_point.y << std::endl;
        return output_point;
    }
    return {0, 0};
}
