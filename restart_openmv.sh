#!/bin/bash

# 检查 root 权限
if [ "$(id -u)" -ne 0 ]; then
    echo "错误：此脚本需要root权限，请使用sudo运行"
    exit 1
fi

OPENMV_VIDPID="37c5:1204"

echo "正在查找OpenMV设备（VID:PID=$OPENMV_VIDPID）..."

# 使用 lsusb 确认设备存在
openmv_info=$(lsusb | grep -i "$OPENMV_VIDPID")
if [ -z "$openmv_info" ]; then
    echo "错误：未找到OpenMV设备"
    echo "当前连接的USB设备："
    lsusb
    exit 1
fi

echo "找到设备：$openmv_info"

# 提取总线号和设备号
bus=$(echo "$openmv_info" | awk '{print $2}')
device=$(echo "$openmv_info" | awk '{print $4}' | sed 's/://')
echo "设备位置：总线 $bus，设备 $device"

# 方法四：通过遍历/sys/bus/usb/devices/目录查找设备路径
echo "正在搜索设备路径..."
usb_device_path=""

# 遍历所有USB设备目录
for dev_path in /sys/bus/usb/devices/*; do
    # 检查是否是目录且包含供应商和产品ID文件
    if [ -d "$dev_path" ] && [ -f "$dev_path/idVendor" ] && [ -f "$dev_path/idProduct" ]; then
        vendor=$(cat "$dev_path/idVendor")
        product=$(cat "$dev_path/idProduct")

        # 检查是否匹配OpenMV的VID和PID
        if [ "$vendor" = "37c5" ] && [ "$product" = "1204" ]; then
            usb_device_path="$dev_path"
            echo "找到匹配的设备路径: $usb_device_path"
            break
        fi
    fi
done

# 验证设备路径是否存在
if [ -z "$usb_device_path" ] || [ ! -d "$usb_device_path" ]; then
    echo "错误：无法找到设备的sysfs路径"
    echo "尝试通过总线号和设备号查找路径..."

    # 尝试构建可能的路径格式
    possible_paths=(
        "/sys/bus/usb/devices/$bus-$device"
        "/sys/bus/usb/devices/usb$bus/$bus-$device"
    )

    for path in "${possible_paths[@]}"; do
        if [ -d "$path" ]; then
            usb_device_path="$path"
            echo "找到备用路径: $usb_device_path"
            break
        fi
    done

    # 如果仍然找不到，退出
    if [ -z "$usb_device_path" ]; then
        echo "错误：无法确定设备路径"
        exit 1
    fi
fi

echo "设备sysfs路径：$usb_device_path"

# 检查authorized文件是否存在
authorized_file="$usb_device_path/authorized"
if [ ! -f "$authorized_file" ]; then
    echo "警告：未找到authorized控制文件，尝试使用power/control方法..."

    # 尝试使用power/control方法
    power_control="$usb_device_path/power/control"
    if [ -f "$power_control" ]; then
        echo "使用power/control方法重置设备..."
        echo "suspend" > "$power_control"
        sleep 2
        echo "on" > "$power_control"
        sleep 3
    else
        echo "错误：找不到任何可用的重置方法"
        echo "路径内容："
        ls -l "$usb_device_path"
        exit 1
    fi
else
    # 使用authorized文件方法
    echo "正在重置OpenMV设备..."
    echo "1. 禁用设备（模拟断电）..."
    echo 0 > "$authorized_file"

    # 等待设备完全关闭
    echo "等待2秒..."
    sleep 2

    echo "2. 启用设备（模拟上电）..."
    echo 1 > "$authorized_file"

    # 等待设备重新初始化
    echo "等待3秒让设备重新启动..."
    sleep 3
fi

# 验证操作结果
echo "验证设备状态..."
if lsusb | grep -i "$OPENMV_VIDPID" &> /dev/null; then
    echo "成功：OpenMV设备已重新上电并识别"
else
    echo "警告：操作完成，但未检测到设备。可能需要手动拔插。"
    exit 1
fi

sleep 4

# 启动节点（如果需要）
echo "启动ROS2节点..."
gnome-terminal --tab -- bash -c "source install/setup.bash;ros2 run camera_data_handle calculate_publish"

sleep 4

echo "OpenMV重新上电操作完成"
exit 0