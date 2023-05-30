#    @@@@@@@@@@@@@@@@@@@@
#    @@@@@@@@@&@@@&&@@@@@
#    @@@@@ @@  @@    @@@@
#    @@@@@ @@  @@    @@@@
#    @@@@@ @@  @@    @@@@ Copyright (c) 2023, Acceleration Robotics®
#    @@@@@ @@  @@    @@@@ Author: Víctor Mayoral Vilches <victor@accelerationrobotics.com>
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


ba = BenchmarkAnalyzer("a4_depth_image_proc")

target_chain = [
    # "ros2:callback_start",
    'robotperf_benchmarks:robotperf_image_input_cb_init',
    'robotperf_benchmarks:robotperf_image_input_cb_fini',
    # 'ros2:callback_end',
    # 'ros2:callback_start',
    'robotperf_benchmarks:robotperf_image_input_cb_init',
    'robotperf_benchmarks:robotperf_image_input_cb_fini',
    # 'ros2:callback_end',
    # "ros2:callback_start",
    'ros2_image_pipeline:depth_image_proc_transform_to_pointcloud_cb_init',
    'ros2_image_pipeline:depth_image_proc_transform_to_pointcloud_init',
    'ros2_image_pipeline:depth_image_proc_transform_to_pointcloud_fini',
    'ros2_image_pipeline:depth_image_proc_transform_to_pointcloud_cb_fini',
    # "ros2:callback_end",
    # "ros2:callback_start",
    'robotperf_benchmarks:robotperf_pointcloud_output_cb_init',
    'robotperf_benchmarks:robotperf_pointcloud_output_cb_fini',
    # "ros2:callback_end",
]

# add parameters for analyzing the traces

# ba.add_target(
#     {
#         "name": "ros2:callback_start",
#         "name_disambiguous": "ros2:callback_start",
#         "colors_fg": "blue",
#         "colors_fg_bokeh": "lightgray",
#         "layer": "rclcpp",
#         "label_layer": 3,
#         "marker": "diamond",
#     }
# )
ba.add_target(
    {
        "name": "robotperf_benchmarks:robotperf_image_input_cb_init",
        "name_disambiguous": "robotperf_benchmarks:robotperf_image_input_cb_init",
        "colors_fg": "blue",
        "colors_fg_bokeh": "silver",
        "layer": "userland",
        "label_layer": 4,
        "marker": "plus",
    }
)
ba.add_target(
    {
        "name": "robotperf_benchmarks:robotperf_image_input_cb_fini",
        "name_disambiguous": "robotperf_benchmarks:robotperf_image_input_cb_fini",
        "colors_fg": "blue",
        "colors_fg_bokeh": "darkgray",
        "layer": "benchmark",
        "label_layer": 5,
        "marker": "plus",
    }
)
# ba.add_target(
#     {
#         "name": "ros2:callback_end",
#         "name_disambiguous": "ros2:callback_end",
#         "colors_fg": "blue",
#         "colors_fg_bokeh": "gray",
#         "layer": "rclcpp",
#         "label_layer": 3,
#         "marker": "diamond",
#     }
# )

# ba.add_target(
#     {
#         "name": "ros2:callback_start",
#         "name_disambiguous": "ros2:callback_start",
#         "colors_fg": "blue",
#         "colors_fg_bokeh": "lightgray",
#         "layer": "rclcpp",
#         "label_layer": 3,
#         "marker": "diamond",
#     }
# )
ba.add_target(
    {
        "name": "robotperf_benchmarks:robotperf_image_input_cb_init",
        "name_disambiguous": "robotperf_benchmarks:robotperf_image_input_cb_init (2)",
        "colors_fg": "blue",
        "colors_fg_bokeh": "silver",
        "layer": "userland",
        "label_layer": 4,
        "marker": "plus",
    }
)
ba.add_target(
    {
        "name": "robotperf_benchmarks:robotperf_image_input_cb_fini",
        "name_disambiguous": "robotperf_benchmarks:robotperf_image_input_cb_fini (2)",
        "colors_fg": "blue",
        "colors_fg_bokeh": "darkgray",
        "layer": "benchmark",
        "label_layer": 5,
        "marker": "plus",
    }
)
# ba.add_target(
#     {
#         "name": "ros2:callback_end",
#         "name_disambiguous": "ros2:callback_end",
#         "colors_fg": "blue",
#         "colors_fg_bokeh": "gray",
#         "layer": "rclcpp",
#         "label_layer": 3,
#         "marker": "diamond",
#     }
# )
# ba.add_target(
#     {
#         "name": "ros2:callback_start",
#         "name_disambiguous": "ros2:callback_start (3)",
#         "colors_fg": "blue",
#         "colors_fg_bokeh": "chartreuse",
#         "layer": "rclcpp",
#         "label_layer": 3,
#         "marker": "diamond",
#     }
# )
ba.add_target(
    {
        "name": "ros2_image_pipeline:depth_image_proc_transform_to_pointcloud_cb_init",
        "name_disambiguous": "ros2_image_pipeline:depth_image_proc_transform_to_pointcloud_cb_init",
        "colors_fg": "yellow",
        "colors_fg_bokeh": "salmon",
        "layer": "userland",
        "label_layer": 4,
        "marker": "plus",
    }
)
ba.add_target(
    {
        "name": "ros2_image_pipeline:depth_image_proc_transform_to_pointcloud_init",
        "name_disambiguous": "ros2_image_pipeline:depth_image_proc_transform_to_pointcloud_init",
        "colors_fg": "red",
        "colors_fg_bokeh": "darksalmon",
        "layer": "userland",
        "label_layer": 4,
        "marker": "plus",
    }
)
ba.add_target(
    {
        "name": "ros2_image_pipeline:depth_image_proc_transform_to_pointcloud_fini",
        "name_disambiguous": "ros2_image_pipeline:depth_image_proc_transform_to_pointcloud_fini",
        "colors_fg": "red",
        "colors_fg_bokeh": "lightcoral",
        "layer": "userland",
        "label_layer": 4,
        "marker": "plus",
    }
)
ba.add_target(
    {
        "name": "ros2_image_pipeline:depth_image_proc_transform_to_pointcloud_cb_fini",
        "name_disambiguous": "ros2_image_pipeline:depth_image_proc_transform_to_pointcloud_cb_fini",
        "colors_fg": "yellow",
        "colors_fg_bokeh": "darkred",
        "layer": "userland",
        "label_layer": 4,
        "marker": "plus",
    }
)
# ba.add_target(
#     {
#         "name": "ros2:callback_end",
#         "name_disambiguous": "ros2:callback_end",
#         "colors_fg": "blue",
#         "colors_fg_bokeh": "gray",
#         "layer": "rclcpp",
#         "label_layer": 3,
#         "marker": "diamond",
#     }
# )
# ba.add_target(
#     {
#         "name": "ros2:callback_start",
#         "name_disambiguous": "ros2:callback_start (3)",
#         "colors_fg": "blue",
#         "colors_fg_bokeh": "chartreuse",
#         "layer": "rclcpp",
#         "label_layer": 3,
#         "marker": "diamond",
#     }
# )
ba.add_target(
    {
        "name": "robotperf_benchmarks:robotperf_pointcloud_output_cb_init",
        "name_disambiguous": "robotperf_benchmarks:robotperf_pointcloud_output_cb_init",
        "colors_fg": "blue",
        "colors_fg_bokeh": "chocolate",
        "layer": "benchmark",
        "label_layer": 5,
        "marker": "plus",
    }
)
ba.add_target(
    {
        "name": "robotperf_benchmarks:robotperf_pointcloud_output_cb_fini",
        "name_disambiguous": "robotperf_benchmarks:robotperf_pointcloud_output_cb_fini",
        "colors_fg": "blue",
        "colors_fg_bokeh": "coral",
        "layer": "userland",
        "label_layer": 4,
        "marker": "plus",
    }
)
# ba.add_target(
#     {
#         "name": "ros2:callback_end",
#         "name_disambiguous": "ros2:callback_end ()",
#         "colors_fg": "blue",
#         "colors_fg_bokeh": "cornflowerblue",
#         "layer": "rclcpp",
#         "label_layer": 3,
#         "marker": "diamond",
#     }
# )

ba.analyze_latency()
ba.analyze_throughput()