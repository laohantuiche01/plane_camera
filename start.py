

import subprocess

tasks=[
    {
        'source_path':'install/setup.bash',
        'ros_cmd':'ros2 launch realsense2_camera rs_launch.py '
    },
    {
        'source_path':'install/setup.bash',
        'ros_cmd':'ros2 run camera_data_handle receive_data'
    },
    {
        'source_path':'install/setup.bash',
        'ros_cmd':'ros2 run camera_data_handle tf_publish'

    }
]

for task in tasks:
    full_cmd=f'source {task["source_path"]} && {task["ros_cmd"]} '

    subprocess.Popen(['gnome-terminal','--','bash','-c',full_cmd])