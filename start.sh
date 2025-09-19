#!/bin/bash

gnome-terminal --tab -- bash -c "source install/setup.bash;ros2 launch realsense2_camera rs_launch.py "

gnome-terminal --tab -- bash -c "source install/setup.bash;ros2 run camera_data_handle receive_data  "

gnome-terminal --tab -- bash -c "source install/setup.bash;ros2 run camera_data_handle detect_publish  "

gnome-terminal --tab -- bash -c "source install/setup.bash;ros2 run camera_data_handle calculate_publish  "
