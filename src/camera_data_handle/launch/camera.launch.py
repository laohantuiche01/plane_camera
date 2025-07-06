from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="camera_data_handle",
            executable='receive_data',
            name='receive_data',
            # parameters=[
            #     'path/to/param.yaml']
        )
    ])
