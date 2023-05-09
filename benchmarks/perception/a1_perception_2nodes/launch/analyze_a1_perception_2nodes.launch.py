#
#    @@@@@@@@@@@@@@@@@@@@
#    @@@@@@@@@&@@@&&@@@@@
#    @@@@@ @@  @@    @@@@
#    @@@@@ @@  @@    @@@@
#    @@@@@ @@  @@    @@@@ Copyright (c) 2023, Acceleration Robotics®
#    @@@@@ @@  @@    @@@@ Author: Martiño Crespo <martinho@accelerationrobotics.com>
#    @@@@@ @@  @@    @@@@
#    @@@@@@@@@&@@@@@@@@@@
#    @@@@@@@@@@@@@@@@@@@@
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from benchmark_utilities.analysis import BenchmarkAnalyzer
from launch import LaunchDescription

def generate_launch_description():
    return LaunchDescription()

# targeted chain of messages for tracing
# NOTE: there're not "publish" tracepoints because
# graph's using inter-process communications
#
target_chain = [
    "ros2:callback_start",
    "robotperf_benchmarks:robotperf_image_input_cb_init",
    "robotperf_benchmarks:robotperf_image_input_cb_fini",
    "ros2:callback_end",
    "ros2:callback_start",
    "ros2_image_pipeline:image_proc_rectify_cb_init",
    "ros2_image_pipeline:image_proc_rectify_init",
    "ros2_image_pipeline:image_proc_rectify_fini",
    # "ros2:rclcpp_publish",
    # "ros2:rcl_publish",
    # "ros2:rmw_publish",
    "ros2_image_pipeline:image_proc_rectify_cb_fini",
    "ros2:callback_end",
    "ros2:callback_start",
    "ros2_image_pipeline:image_proc_resize_cb_init",
    "ros2_image_pipeline:image_proc_resize_init",
    "ros2_image_pipeline:image_proc_resize_fini",
    # "ros2:rclcpp_publish",
    # "ros2:rcl_publish",
    # "ros2:rmw_publish",
    "ros2_image_pipeline:image_proc_resize_cb_fini",
    "ros2:callback_end",
    "ros2:callback_start",
    "robotperf_benchmarks:robotperf_image_output_cb_init",
    "robotperf_benchmarks:robotperf_image_output_cb_fini",
    "ros2:callback_end",
]
target_chain_dissambiguous = [
    "ros2:callback_start",
    "robotperf_benchmarks:robotperf_image_input_cb_init",
    "robotperf_benchmarks:robotperf_image_input_cb_fini",
    "ros2:callback_end",
    "ros2:callback_start (2)",
    "ros2_image_pipeline:image_proc_rectify_cb_init",
    "ros2_image_pipeline:image_proc_rectify_init",
    "ros2_image_pipeline:image_proc_rectify_fini",
    # "ros2:rclcpp_publish",
    # "ros2:rcl_publish",
    # "ros2:rmw_publish",
    "ros2_image_pipeline:image_proc_rectify_cb_fini",
    "ros2:callback_end (2)",
    "ros2:callback_start (3)",
    "ros2_image_pipeline:image_proc_resize_cb_init",
    "ros2_image_pipeline:image_proc_resize_init",
    "ros2_image_pipeline:image_proc_resize_fini",
    # "ros2:rclcpp_publish (2)",
    # "ros2:rcl_publish (2)",
    # "ros2:rmw_publish (2)",
    "ros2_image_pipeline:image_proc_resize_cb_fini",
    "ros2:callback_end (3)",
    "ros2:callback_start (4)",
    "robotperf_benchmarks:robotperf_image_output_cb_init",
    "robotperf_benchmarks:robotperf_image_output_cb_fini",
    "ros2:callback_end (4)",
]
target_chain_colors_fg = [
    "blue",
    "blue",
    "blue",
    "blue",
    "blue",
    "yellow",
    "red",
    "red",
    # "blue",
    # "blue",
    # "blue",
    "yellow",
    "blue",
    "blue",
    "yellow",
    "red",
    "red",
    # "blue",
    # "blue",
    # "blue",
    "yellow",
    "blue",
    "blue",
    "blue",
    "blue",
    "blue",    
]
target_chain_colors_fg_bokeh = [
    "lightgray",
    "silver",
    "darkgray",
    "gray",
    "lightsalmon",
    "salmon",
    "darksalmon",
    "lightcoral",
    # "indianred",
    # "crimson",
    # "firebrick",
    "darkred",
    "red",
    "lavender",
    "thistle",
    "plum",
    "fuchsia",
    # "mediumorchid",
    # "mediumpurple",
    # "darkmagenta",
    "indigo",
    "mediumslateblue",
    "chartreuse",
    "chocolate",
    "coral",
    "cornflowerblue",    
]
target_chain_layer = [
    "rclcpp",
    "userland",
    "benchmark",
    "rclcpp",
    "rclcpp",
    "userland",
    "userland",
    "userland",
    # "rclcpp",
    # "rcl",
    # "rmw",
    "userland",
    "rclcpp",
    "rclcpp",
    "userland",
    "userland",
    "userland",
    # "rclcpp",
    # "rcl",
    # "rmw",
    "userland",
    "rclcpp",
    "rclcpp",
    "benchmark",
    "userland",
    "rclcpp",    
]
target_chain_label_layer = [  # associated with the layer
    3,
    4,
    5,
    3,
    3,
    4,
    4,
    4,
    # 3,
    # 2,
    # 1,
    4,
    3,
    3,
    4,
    4,
    4,
    # 3,
    # 2,
    # 1,
    4,
    3,
    3,
    5,
    4,
    3,
]
target_chain_marker = [
    "diamond",
    "plus",
    "plus",
    "diamond",
    "diamond",
    "plus",
    "plus",
    "plus",
    # "plus",
    # "plus",
    # "plus",
    "plus",
    "diamond",
    "diamond",
    "plus",
    "plus",
    "plus",
    # "plus",
    # "plus",
    # "plus",
    "plus",
    "diamond",
    "diamond",
    "plus",
    "plus",
    "diamond",    
]

ba = BenchmarkAnalyzer("a1_perception_2nodes", target_chain, target_chain_dissambiguous, target_chain_colors_fg, target_chain_colors_fg_bokeh, target_chain_layer, target_chain_label_layer, target_chain_marker)
ba.analyze_traces()
