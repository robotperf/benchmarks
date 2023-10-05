#
#    @@@@@@@@@@@@@@@@@@@@
#    @@@@@@@@@&@@@&&@@@@@
#    @@@@@ @@  @@    @@@@
#    @@@@@ @@  @@    @@@@
#    @@@@@ @@  @@    @@@@ Copyright (c) 2023, Acceleration Robotics®
#    @@@@@ @@  @@    @@@@ Author: Víctor Mayoral Vilches <victor@accelerationrobotics.com>
#    @@@@@ @@  @@    @@@@ Author: Martiño Crespo <martinho@accelerationrobotics.com>
#    @@@@@ @@  @@    @@@@ Author: Alejandra Martínez Fariña <alex@accelerationrobotics.com>
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
from benchmark_utilities.analysis import BenchmarkAnalyzer, FrameHierarchy
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

import sys
import argparse
import json


def plot_urdf_and_distance_calculation(argv):
    
    # Parse the command-line arguments
    parser = argparse.ArgumentParser()
    parser.add_argument('--hardware_device_type', type=str, help='Hardware Device Type (e.g. cpu or fpga)', default ='cpu')
    parser.add_argument('--trace_path', type=str, help='Path to trace files (e.g. /tmp/analysis/trace)', default = '/tmp/analysis/trace')
    parser.add_argument('--metrics', type=str, help='List of metrics to be analyzed (e.g. latency and/or throughput)', default = ['latency'])
    parser.add_argument('--integrated', type=str, help='Integrated or separated version of the Resize and Rectify nodes (only for fpga now)', default='false') 
    args = parser.parse_args(argv)

    # Get the values of the arguments
    hardware_device_type = args.hardware_device_type
    trace_path = args.trace_path
    metrics_string = args.metrics
    metrics_elements = [element.strip() for element in metrics_string.strip("[]").split(",")]
    metrics = json.loads(json.dumps(metrics_elements))
    integrated = args.integrated

    # Instantiate the class
    ba = BenchmarkAnalyzer('d7_dual_arm_static_avoidance', hardware_device_type)

    # Manipulation traces cannot be identified, since no ID is being stored in the tracepoints
    ba.set_trace_sets_filter_type('name')

    if hardware_device_type == 'cpu':
        # add parameters for analyzing the traces
        ## using message header id
        target_chain = [
            "realtime_urdf_filter:urdf_filter_cb_init",                                 # 0
            "realtime_urdf_filter:urdf_filter_init",                                    # 1
            "realtime_urdf_filter:urdf_filter_fini",                                    # 2
            "realtime_urdf_filter:urdf_filter_cb_fini",                                 # 3
            "dual_arm_static_avoidance:dual_arm_distance_calculation_cb_init:",         # 4
            "dual_arm_static_avoidance:dual_arm_distance_calculation_init",             # 5
            "dual_arm_static_avoidance:dual_arm_distance_calculation_fini",             # 6
            "dual_arm_static_avoidance:dual_arm_distance_calculation_cb_fini",          # 7
        ]

        ba.add_target(
            {
                "name": "realtime_urdf_filter:urdf_filter_cb_init",
                "name_disambiguous": "realtime_urdf_filter:urdf_filter_cb_init",
                "colors_fg": "yellow",
                "colors_fg_bokeh": "salmon",
                "layer": "vision pipeline",
                "label_layer": 4,
                "marker": "plus",
            }
        )
        ba.add_target(
            {
                "name": "realtime_urdf_filter:urdf_filter_init",
                "name_disambiguous": "realtime_urdf_filter:urdf_filter_init",
                "colors_fg": "red",
                "colors_fg_bokeh": "darksalmon",
                "layer": "vision pipeline",
                "label_layer": 4,
                "marker": "plus",
            }
        )
        ba.add_target(
            {
                "name": "realtime_urdf_filter:urdf_filter_fini",
                "name_disambiguous": "realtime_urdf_filter:urdf_filter_fini",
                "colors_fg": "red",
                "colors_fg_bokeh": "lightcoral",
                "layer": "vision pipeline",
                "label_layer": 4,
                "marker": "plus",
            }
        )
        ba.add_target(
            {
                "name": "realtime_urdf_filter:urdf_filter_cb_fini",
                "name_disambiguous": "realtime_urdf_filter:urdf_filter_cb_fini",
                "colors_fg": "yellow",
                "colors_fg_bokeh": "darkred",
                "layer": "vision pipeline",
                "label_layer": 4,
                "marker": "plus",
            }
        )

        ba.add_target(
            {
                "name": "dual_arm_static_avoidance:dual_arm_distance_calculation_cb_init",
                "name_disambiguous": "dual_arm_static_avoidance:dual_arm_distance_calculation_cb_init",
                "colors_fg": "yellow",
                "colors_fg_bokeh": "salmon",
                "layer": "vision pipeline",
                "label_layer": 4,
                "marker": "plus",
            }
        )
        ba.add_target(
            {
                "name": "dual_arm_static_avoidance:dual_arm_distance_calculation_init",
                "name_disambiguous": "dual_arm_static_avoidance:dual_arm_distance_calculation_init",
                "colors_fg": "red",
                "colors_fg_bokeh": "darksalmon",
                "layer": "vision pipeline",
                "label_layer": 4,
                "marker": "plus",
            }
        )
        ba.add_target(
            {
                "name": "dual_arm_static_avoidance:dual_arm_distance_calculation_fini",
                "name_disambiguous": "dual_arm_static_avoidance:dual_arm_distance_calculation_fini",
                "colors_fg": "red",
                "colors_fg_bokeh": "lightcoral",
                "layer": "vision pipeline",
                "label_layer": 4,
                "marker": "plus",
            }
        )
        ba.add_target(
            {
                "name": "dual_arm_static_avoidance:dual_arm_distance_calculation_cb_fini",
                "name_disambiguous": "dual_arm_static_avoidance:dual_arm_distance_calculation_cb_fini",
                "colors_fg": "yellow",
                "colors_fg_bokeh": "darkred",
                "layer": "vision pipeline",
                "label_layer": 4,
                "marker": "plus",
            }
        ) 

    ba.draw_tracepoints(trace_path)

def generate_launch_description():
    # Declare the launch arguments
    hardware_device_type_arg = DeclareLaunchArgument(
        'hardware_device_type',
        default_value='cpu',
        description='Hardware Device Type (e.g. cpu or fpga)'
    )

    trace_path_arg = DeclareLaunchArgument(
        'trace_path',
        default_value='/tmp/analysis/trace',
        description='Path to trace files (e.g. /tmp/analysis/trace)'
    )

    metrics_arg = DeclareLaunchArgument(
        'metrics',
        default_value=['latency'],
        description='List of metrics to be analyzed (e.g. latency and/or throughput)'
    )
    
    integrated_arg = DeclareLaunchArgument(
        'integrated',
        default_value="false",
        description='Integrated or separated version of the Resize and Rectify nodes (only for fpga now)'
    )

    # Create the launch description
    ld = LaunchDescription()
    
    # Define the ExecuteProcess action to run the Python script
    analyzer = ExecuteProcess(
        cmd=[
            'python3', "src/benchmarks/benchmarks/manipulation/d7_dual_arm_static_avoidance/launch/plot_d7_dual_arm_static_avoidance.launch.py",
            '--hardware_device_type', LaunchConfiguration('hardware_device_type'),
            '--trace_path', LaunchConfiguration('trace_path'),
            '--metrics', LaunchConfiguration('metrics'),
            '--integrated', LaunchConfiguration('integrated')],
        output='screen'
    )

    # Add the declared launch arguments to the launch description
    ld.add_action(hardware_device_type_arg)
    ld.add_action(trace_path_arg)
    ld.add_action(metrics_arg)
    ld.add_action(integrated_arg)
    
    # Add the ExecuteProcess action to the launch description
    ld.add_action(analyzer)

    return ld

if __name__ == '__main__':

    plot_urdf_and_distance_calculation(sys.argv[1:])
