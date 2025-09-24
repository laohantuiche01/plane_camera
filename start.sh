#!/bin/bash

gnome-terminal --tab -- bash -c "source install/setup.bash;ros2 launch realsense2_camera rs_launch.py rgb_camera.color_profile:=640x480x30"

gnome-terminal --tab -- bash -c "source install/setup.bash;ros2 run camera_data_handle receive_data  "

gnome-terminal --tab -- bash -c "source install/setup.bash;ros2 run camera_data_handle detect_publish  "

gnome-terminal --tab -- bash -c "source install/setup.bash;ros2 run camera_data_handle calculate_publish  " 
