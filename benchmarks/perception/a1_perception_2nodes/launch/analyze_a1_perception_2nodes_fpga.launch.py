#
#    @@@@@@@@@@@@@@@@@@@@
#    @@@@@@@@@&@@@&&@@@@@
#    @@@@@ @@  @@    @@@@
#    @@@@@ @@  @@    @@@@
#    @@@@@ @@  @@    @@@@ Copyright (c) 2023, Acceleration Robotics®
#    @@@@@ @@  @@    @@@@ Author: Víctor Mayoral Vilches <victor@accelerationrobotics.com>
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

ba = BenchmarkAnalyzer("a1_perception_2nodes_fpga", hardware_device_type="fpga")

# add parameters for analyzing the traces
ba.add_target({"name": "ros2:callback_start", "name_disambiguous": "ros2:callback_start", "colors_fg": "blue", "colors_fg_bokeh": "lightgray", "layer": "rclcpp", "label_layer": 3, "marker": "diamond"})
ba.add_target({"name": "robotperf_benchmarks:robotperf_image_input_cb_init", "name_disambiguous": "robotperf_benchmarks:robotperf_image_input_cb_init", "colors_fg": "blue", "colors_fg_bokeh": "silver", "layer": "userland", "label_layer": 4, "marker": "plus"})
ba.add_target({"name": "robotperf_benchmarks:robotperf_image_input_cb_fini", "name_disambiguous": "robotperf_benchmarks:robotperf_image_input_cb_fini", "colors_fg": "blue", "colors_fg_bokeh": "darkgray", "layer": "benchmark", "label_layer": 5, "marker": "plus"})
ba.add_target({"name": "ros2:callback_end", "name_disambiguous": "ros2:callback_end", "colors_fg": "blue", "colors_fg_bokeh": "gray", "layer": "rclcpp", "label_layer": 3, "marker": "diamond"})
ba.add_target({"name": "ros2:callback_start", "name_disambiguous": "ros2:callback_start (2)", "colors_fg": "blue", "colors_fg_bokeh": "lightsalmon", "layer": "rclcpp", "label_layer": 3, "marker": "diamond"})
ba.add_target({"name": "ros2_image_pipeline:image_proc_rectify_cb_init", "name_disambiguous": "ros2_image_pipeline:image_proc_rectify_cb_init", "colors_fg": "yellow", "colors_fg_bokeh": "salmon", "layer": "userland", "label_layer": 4, "marker": "plus"})
ba.add_target({"name": "ros2_image_pipeline:image_proc_rectify_init", "name_disambiguous": "ros2_image_pipeline:image_proc_rectify_init", "colors_fg": "red", "colors_fg_bokeh": "darksalmon", "layer": "userland", "label_layer": 4, "marker": "plus"})
ba.add_target({"name": "ros2:vitis_profiler:kernel_enqueue", "name_disambiguous": "ros2:kernel_enqueue:rectify_init", "colors_fg": "green", "colors_fg_bokeh": "indianred", "kernel", 1, "marker": "plus"})
ba.add_target({"name": "ros2:vitis_profiler:kernel_enqueue", "name_disambiguous": "ros2:kernel_enqueue:rectify_fini", "colors_fg": "green", "colors_fg_bokeh": "crimson", "kernel", 1, "marker": "plus"})
ba.add_target({"name": "ros2_image_pipeline:image_proc_rectify_fini", "name_disambiguous": "ros2_image_pipeline:image_proc_rectify_fini", "colors_fg": "red", "colors_fg_bokeh": "lightcoral", "layer": "userland", "label_layer": 4, "marker": "plus"})
# ba.add_target({"name": "ros2:rclcpp_publish", "name_disambiguous": "ros2:rclcpp_publish", "colors_fg": "blue", "colors_fg_bokeh": "indianred", "layer": "rclcpp", "label_layer": 3, "marker": "plus"})
# ba.add_target({"name": "ros2:rcl_publish", "name_disambiguous": "ros2:rcl_publish", "colors_fg": "blue", "colors_fg_bokeh": "crimson" , "layer": "rcl", "label_layer": 2, "marker": "plus"})
# ba.add_target({"name": "ros2:rmw_publish", "name_disambiguous": "ros2:rmw_publish", "colors_fg": "blue", "colors_fg_bokeh": "firebrick" , "layer": "rmw", "label_layer": 1, "marker": "plus"})
ba.add_target({"name": "ros2_image_pipeline:image_proc_rectify_cb_fini", "name_disambiguous": "ros2_image_pipeline:image_proc_rectify_cb_fini", "colors_fg": "yellow", "colors_fg_bokeh": "darkred", "layer": "userland", "label_layer": 4, "marker": "plus"})
ba.add_target({"name": "ros2:callback_end", "name_disambiguous": "ros2:callback_end (2)", "colors_fg": "blue", "colors_fg_bokeh": "red", "layer": "rclcpp", "label_layer": 3, "marker": "diamond"})
ba.add_target({"name": "ros2:callback_start", "name_disambiguous": "ros2:callback_start (3)", "colors_fg": "blue", "colors_fg_bokeh": "lavender", "layer": "rclcpp", "label_layer": 3, "marker": "diamond"})
ba.add_target({"name": "ros2_image_pipeline:image_proc_resize_cb_init", "name_disambiguous": "ros2_image_pipeline:image_proc_resize_cb_init", "colors_fg": "yellow", "colors_fg_bokeh": "thistle", "layer": "userland", "label_layer": 4, "marker": "plus"})
ba.add_target({"name": "ros2_image_pipeline:image_proc_resize_init", "name_disambiguous": "ros2_image_pipeline:image_proc_resize_init", "colors_fg": "red", "colors_fg_bokeh": "plum", "layer": "userland", "label_layer": 4, "marker": "plus"})
ba.add_target({"name": "ros2:vitis_profiler:kernel_enqueue", "name_disambiguous": "ros2:kernel_enqueue:resize_init", "colors_fg": "green", "colors_fg_bokeh": "fuchsia",	"kernel",	1,	"marker": "plus"})
ba.add_target({"name": "ros2:vitis_profiler:kernel_enqueue", "name_disambiguous": "ros2:kernel_enqueue:resize_finit", "colors_fg": "green", "colors_fg_bokeh": "colors_fg":"darkmagenta", "kernel",	1,	"marker": "plus"})
ba.add_target({"name": "ros2_image_pipeline:image_proc_resize_fini", "name_disambiguous": "ros2_image_pipeline:image_proc_resize_fini", "colors_fg": "red", "colors_fg_bokeh": "fuchsia", "layer": "userland", "label_layer": 4, "marker": "plus"})
# ba.add_target({"name": "ros2:rclcpp_publish", "name_disambiguous": "ros2:rclcpp_publish (2)", "colors_fg": "blue", "colors_fg_bokeh": "mediumorchid", "layer": "rclcpp", "label_layer": 3, "marker": "plus"})
# ba.add_target({"name": "ros2:rcl_publish", "name_disambiguous": "ros2:rcl_publish (2)", "colors_fg": "blue", "colors_fg_bokeh": "mediumpurple", "layer": "rcl", "label_layer": 2, "marker": "plus"})
# ba.add_target({"name": "ros2:rmw_publish", "name_disambiguous": "ros2:rmw_publish (2)", "colors_fg": "blue", "colors_fg_bokeh": "darkmagenta", "layer": "rmw", "label_layer": 1, "marker": "plus"})
ba.add_target({"name": "ros2_image_pipeline:image_proc_resize_cb_fini", "name_disambiguous": "ros2_image_pipeline:image_proc_resize_cb_fini", "colors_fg": "yellow", "colors_fg_bokeh": "indigo", "layer": "userland", "label_layer": 4, "marker": "plus"})
ba.add_target({"name": "ros2:callback_end", "name_disambiguous": "ros2:callback_end (3)", "colors_fg": "blue", "colors_fg_bokeh": "mediumslateblue", "layer": "rclcpp", "label_layer": 3, "marker": "diamond"})
ba.add_target({"name": "ros2:callback_start", "name_disambiguous": "ros2:callback_start (4)", "colors_fg": "blue", "colors_fg_bokeh": "chartreuse", "layer": "rclcpp", "label_layer": 3, "marker": "diamond"})
ba.add_target({"name": "robotperf_benchmarks:robotperf_image_output_cb_init", "name_disambiguous": "robotperf_benchmarks:robotperf_image_output_cb_init", "colors_fg": "blue", "colors_fg_bokeh": "chocolate", "layer": "benchmark", "label_layer": 5, "marker": "plus"})
ba.add_target({"name": "robotperf_benchmarks:robotperf_image_output_cb_fini", "name_disambiguous": "robotperf_benchmarks:robotperf_image_output_cb_fini", "colors_fg": "blue", "colors_fg_bokeh": "coral", "layer": "userland", "label_layer": 4, "marker": "plus"})
ba.add_target({"name": "ros2:callback_end", "name_disambiguous": "ros2:callback_end (4)", "colors_fg": "blue", "colors_fg_bokeh": "cornflowerblue", "layer": "rclcpp", "label_layer": 3, "marker": "diamond"})

ba.analyze_traces()
