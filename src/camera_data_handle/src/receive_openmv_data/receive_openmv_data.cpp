#include "../../include/receive_openmv_data/receive_openmv_data.h"

#define OPENMV_NULL_ERROR 321

ReceiveOpenMVData::ReceiveOpenMVData(const std::string port,
                                     const speed_t baudRate) : port_(port), baudRate_(baudRate) {
}

std::string ReceiveOpenMVData::Receive_Openmv_Data() {
    int fd = openSerialPort(port_, baudRate_);
    if (fd == -1) {
        return "1";
    }

    int num = 0;

    try {
        while (num < 5) {
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
                return line;
            }
            num++;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
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
    int fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd == -1) {
        std::cerr << "无法打开串口: " << port << std::endl;
        return -1;
    }

    struct termios tty;
    if (tcgetattr(fd, &tty) == -1) {
        std::cerr << "无法获取串口属性" << std::endl;
        close(fd);
        return -1;
    }

    //配置输入输出波特率
    cfsetospeed(&tty, baudRate);
    cfsetispeed(&tty, baudRate);

    tty.c_cflag &= ~PARENB; //禁用奇偶校验
    tty.c_cflag &= ~CSTOPB; //1停止位
    tty.c_cflag &= ~CSIZE; //清除数据位设置
    tty.c_cflag |= CS8; //8位数据位

    //禁用硬件流控制
    tty.c_cflag &= ~CRTSCTS;

    //启用接收器，设置本地模式
    tty.c_cflag |= (CLOCAL | CREAD);

    //禁用软件流控制
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);

    //设置原始输入模式
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

    //设置原始输出模式
    tty.c_oflag &= ~OPOST;

    //设置超时
    tty.c_cc[VTIME] = 10; //1秒超时
    tty.c_cc[VMIN] = 0;

    //应用配置
    if (tcsetattr(fd, TCSANOW, &tty) == -1) {
        std::cerr << "无法设置串口属性" << std::endl;
        close(fd);
        return -1;
    }

    return fd;
}

// 读取一行数据
std::string ReceiveOpenMVData::readLine(int fd) {
    std::string line;
    char c;
    ssize_t n;

    while ((n = read(fd, &c, 1)) == 1) {
        if (c == '\n') {
            break;
        }
        line += c;
    }

    return line;
}


CalculateTarget::CalculateTarget() {
    receive_openmv_data_ = std::make_shared<ReceiveOpenMVData>();
}

cv::Point2d CalculateTarget::Decode_Openmv_Data(std::string &input_str) {
    if (input_str == "null") {
        return {OPENMV_NULL_ERROR, OPENMV_NULL_ERROR};
    }
    nlohmann::json j = nlohmann::json::parse(input_str);
    double point_x = j["cx"];
    double point_y = j["cy"];
    return {point_x, point_y};
}

cv::Point2d CalculateTarget::Transform_Image_TO_Real(cv::Point2d &image_point, double height) {
    double the_camera_center = 0.05;

    double height_center = height + the_camera_center; // 把相机高度转化成焦点高度 （这个比例要调the_camera_center）
    cv::Point2d image_point_c = cv::Point2d(image_point.x - 160, image_point.y - 120);

    double real_x = image_point_c.x * height_center / 100;
    double real_y = (1 + image_point_c.y / 100) * height_center;
    return {real_x, real_y};
}

cv::Point2d CalculateTarget::Handle_Openmv_Data() {
    while (true) {
        std::string input_str = receive_openmv_data_.get()->Receive_Openmv_Data();
        cv::Point2d temp_point = Decode_Openmv_Data(input_str);

        if (temp_point == cv::Point2d(OPENMV_NULL_ERROR, OPENMV_NULL_ERROR)) {
            //std::cout << temp_point.x << " " << temp_point.y << std::endl;
            return temp_point;
            continue;
        }

        cv::Point2d output_point = Transform_Image_TO_Real(temp_point);
        //std::cout << output_point.x << " " << output_point.y << std::endl;
        return output_point;
    }
    return {0, 0};
}

// int main() {
//     CalculateTarget calculate_target;
//     calculate_target.Handle_Openmv_Data();
//     return 0;
// }
