from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="camera_data_handle",
            executable='receive_data',
            name='receive_data',
            parameters=[
                '/home/zxk/桌面/Unmanned_Aerial_Vehicle_Workspace/camera_handle/src/camera_data_handle/config/param.yaml']
        )
    ])
