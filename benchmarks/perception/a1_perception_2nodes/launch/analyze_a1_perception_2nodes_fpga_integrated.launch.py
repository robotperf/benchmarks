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


ba = BenchmarkAnalyzer("a1_perception_2nodes_fpga_integrated")

# add parameters for analyzing the traces
## using message header id
target_chain = [
    "robotperf_benchmarks:robotperf_image_input_cb_init",  # 0
    "robotperf_benchmarks:robotperf_image_input_cb_fini",  # 1
    "ros2_image_pipeline:image_proc_rectify_cb_init",  # 2
    "ros2_image_pipeline:image_proc_rectify_init",  # 3
    "ros2_image_pipeline:image_proc_rectify_fini",  # 4
    "ros2_image_pipeline:image_proc_rectify_cb_fini",  # 5
    "robotperf_benchmarks:robotperf_image_output_cb_init",  # 6
    "robotperf_benchmarks:robotperf_image_output_cb_fini",  # 7
]

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
ba.add_target(
    {
        "name": "ros2_image_pipeline:image_proc_rectify_cb_init",
        "name_disambiguous": "ros2_image_pipeline:image_proc_rectify_cb_init",
        "colors_fg": "yellow",
        "colors_fg_bokeh": "salmon",
        "layer": "userland",
        "label_layer": 4,
        "marker": "plus",
    }
)
ba.add_target(
    {
        "name": "ros2_image_pipeline:image_proc_rectify_init",
        "name_disambiguous": "ros2_image_pipeline:image_proc_rectify_init",
        "colors_fg": "red",
        "colors_fg_bokeh": "darksalmon",
        "layer": "userland",
        "label_layer": 4,
        "marker": "plus",
    }
)
ba.add_target(
    {
        "name": "ros2_image_pipeline:image_proc_rectify_fini",
        "name_disambiguous": "ros2_image_pipeline:image_proc_rectify_fini",
        "colors_fg": "red",
        "colors_fg_bokeh": "lightcoral",
        "layer": "userland",
        "label_layer": 4,
        "marker": "plus",
    }
)
ba.add_target(
    {
        "name": "ros2_image_pipeline:image_proc_rectify_cb_fini",
        "name_disambiguous": "ros2_image_pipeline:image_proc_rectify_cb_fini",
        "colors_fg": "yellow",
        "colors_fg_bokeh": "darkred",
        "layer": "userland",
        "label_layer": 4,
        "marker": "plus",
    }
)
ba.add_target(
    {
        "name": "robotperf_benchmarks:robotperf_image_output_cb_init",
        "name_disambiguous": "robotperf_benchmarks:robotperf_image_output_cb_init",
        "colors_fg": "blue",
        "colors_fg_bokeh": "chocolate",
        "layer": "benchmark",
        "label_layer": 5,
        "marker": "plus",
    }
)
ba.add_target(
    {
        "name": "robotperf_benchmarks:robotperf_image_output_cb_fini",
        "name_disambiguous": "robotperf_benchmarks:robotperf_image_output_cb_fini",
        "colors_fg": "blue",
        "colors_fg_bokeh": "coral",
        "layer": "userland",
        "label_layer": 4,
        "marker": "plus",
    }
)

ba.analyze_latency(tracepath="/tmp/benchmark_ws/src/benchmarks/trace_offload/trace_cpu_ctf")
