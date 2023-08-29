#
#    @@@@@@@@@@@@@@@@@@@@
#    @@@@@@@@@&@@@&&@@@@@
#    @@@@@ @@  @@    @@@@
#    @@@@@ @@  @@    @@@@
#    @@@@@ @@  @@    @@@@ Copyright (c) 2023, Acceleration Robotics®
#    @@@@@ @@  @@    @@@@ Author: Víctor Mayoral Vilches <victor@accelerationrobotics.com>
#    @@@@@ @@  @@    @@@@ Author: Alejandra Martínez Fariña <alex@accelerationrobotics.com>
#    @@@@@ @@  @@    @@@@ Author: Martiño Crespo <martinho@accelerationrobotics.com>
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
from benchmark_utilities.analysis import BenchmarkAnalyzer, FrameHierarchy()
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

import sys
import argparse
import json

tf_tree = FrameHierarchy()
tf_tree.add_frame("world", "turtle1")
tf_tree.add_frame("world", "turtle2")

def main(argv):
    
    # Parse the command-line arguments
    parser = argparse.ArgumentParser()
    parser.add_argument('--hardware_device_type', type=str, help='Hardware Device Type (e.g. cpu or fpga)', default ='cpu')
    parser.add_argument('--trace_path', type=str, help='Path to trace files (e.g. /tmp/analysis/trace)', default = '/tmp/analysis/trace')
    parser.add_argument('--metrics', type=str, help='List of metrics to be analyzed (e.g. latency and/or throughput)', default = ['latency'])
    args = parser.parse_args(argv)

    # Get the values of the arguments
    hardware_device_type = args.hardware_device_type
    trace_path = args.trace_path
    metrics_string = args.metrics
    metrics_elements = [element.strip() for element in metrics_string.strip("[]").split(",")]
    metrics = json.loads(json.dumps(metrics_elements))
 
    # Instantiate the class
    ba = BenchmarkAnalyzer('m1_turtle_tf2_demo', hardware_device_type, tf_tree)

    if hardware_device_type == 'cpu':
        target_chain = [
        # "ros2:callback_start",
        "robotperf_benchmarks:robotcore_tf2_lookup_cb_init",
        "robotperf_benchmarks:robotcore_tf2_lookup_cb_fini",
        # "ros2:callback_end",
        # "ros2:callback_start",
        # "ros2:callback_end",
        # "ros2:callback_start",
        "robotperf_benchmarks:robotcore_tf2_set_cb_init",
        "robotperf_benchmarks:robotcore_tf2_set_cb_fini",
        # "ros2:callback_end",
        ]

        # # add parameters for analyzing the traces
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
                "name": "robotperf_benchmarks:robotcore_tf2_lookup_cb_init",
                "name_disambiguous": "robotperf_benchmarks:robotcore_tf2_lookup_cb_init",
                "colors_fg": "blue",
                "colors_fg_bokeh": "silver",
                "layer": "userland",
                "label_layer": 4,
                "marker": "plus",
            }
        )
        ba.add_target(
            {
                "name": "robotperf_benchmarks:robotcore_tf2_lookup_cb_fini",
                "name_disambiguous": "robotperf_benchmarks:robotcore_tf2_lookup_cb_fini",
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
        #         "name_disambiguous": "ros2:callback_start (2)",
        #         "colors_fg": "blue",
        #         "colors_fg_bokeh": "lightsalmon",
        #         "layer": "rclcpp",
        #         "label_layer": 3,
        #         "marker": "diamond",
        #     }
        # )
        # ba.add_target(
        #     {
        #         "name": "ros2:callback_end",
        #         "name_disambiguous": "ros2:callback_end (2)",
        #         "colors_fg": "blue",
        #         "colors_fg_bokeh": "red",
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
                "name": "robotperf_benchmarks:robotcore_tf2_set_cb_init",
                "name_disambiguous": "robotperf_benchmarks:robotcore_tf2_set_cb_init",
                "colors_fg": "blue",
                "colors_fg_bokeh": "chocolate",
                "layer": "benchmark",
                "label_layer": 5,
                "marker": "plus",
            }
        )
        ba.add_target(
            {
                "name": "robotperf_benchmarks:robotcore_tf2_set_cb_fini",
                "name_disambiguous": "robotperf_benchmarks:robotcore_tf2_set_cb_fini",
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
        
    else:
        print('The hardware device type ' + hardware_device_type + ' is not yet implemented\n')

    num_metrics = 0 # initialize the metric count
    add_power = False # initialize the boolean
    for metric in metrics:
        if metric == 'power':
            add_power = True
            ba.add_power(
            {
                "name": "robotcore_power:robotcore_power_output_cb_fini",
                "name_disambiguous": "robotcore_power:robotcore_power_output_cb_fini",
                "colors_fg": "blue",
                "colors_fg_bokeh": "silver",
                "layer": "userland",
                "label_layer": 4,
                "marker": "plus",
            }
            )
        else:
            num_metrics += 1 # it will be larger than 0 if other metrics besides power are desired
    
    for metric in metrics:
        if metric == 'latency':
            ba.analyze_latency(trace_path, add_power)
        elif metric == 'throughput':
            ba.analyze_throughput(trace_path, add_power)
        elif metric == 'power': 
            if num_metrics == 0: # launch independently iff no other metric is requested
                total_consumption = ba.analyze_power(trace_path)
                print("The average consumption is {} W".format(total_consumption))
        else:
            print('The metric ' + metric + ' is not yet implemented\n')
    
  
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
    
    # Create the launch description
    ld = LaunchDescription()
    
    # Define the ExecuteProcess action to run the Python script
    analyzer = ExecuteProcess(
        cmd=[
            'python3', "src/benchmarks/benchmarks/meta/m1_turtle_tf2_demo/launch/analyze_m1_turtle_tf2_demo.launch.py",
            '--hardware_device_type', LaunchConfiguration('hardware_device_type'),
            '--trace_path', LaunchConfiguration('trace_path'),
            '--metrics', LaunchConfiguration('metrics')],
        output='screen'
    )

    # Add the declared launch arguments to the launch description
    ld.add_action(hardware_device_type_arg)
    ld.add_action(trace_path_arg)
    ld.add_action(metrics_arg)
    
    # Add the ExecuteProcess action to the launch description
    ld.add_action(analyzer)

    return ld

if __name__ == '__main__':
    main(sys.argv[1:])
    