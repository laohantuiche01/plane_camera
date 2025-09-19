#!/bin/bash

gnome-terminal --tab -- bash -c "source install/setup.bash;ros2 run camera_data_handle calculate_publish"