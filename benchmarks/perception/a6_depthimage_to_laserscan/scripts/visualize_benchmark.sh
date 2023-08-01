#!/bin/bash

# Source the workspace as an overlay
source install/setup.bash

# Open a new terminal window and run rqt_graph
gnome-terminal -- rqt_graph

# Open a new terminal window and run rviz2
gnome-terminal -- rviz2 -d src/benchmarks/benchmarks/perception/a3_stereo_image_proc/config/camera_bot.rviz

# Open a new terminal window and view the stereo image disparity map
gnome-terminal -- ros2 run image_view disparity_view image:=/benchmark/disparity