#
#    @@@@@@@@@@@@@@@@@@@@
#    @@@@@@@@@&@@@&&@@@@@
#    @@@@@ @@  @@    @@@@
#    @@@@@ @@  @@    @@@@
#    @@@@@ @@  @@    @@@@ Copyright (c) 2023, Acceleration Robotics®
#    @@@@@ @@  @@    @@@@ Author: Víctor Mayoral Vilches <victor@accelerationrobotics.com>
#    @@@@@ @@  @@    @@@@ Author: Martiño Crespo <martinho@accelerationrobotics.com>
#    @@@@@ @@  @@    @@@@ Author: Juan Manuel Reina Muñoz <juanma@accelerationrobotics.com>
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

class Trace_Plot:
    def __init__(self, name, layer, label_layer, colors_fg="red", colors_fg_bokeh="darkred",  marker="plus"):
        self.name = name
        self.name_disambiguous = name
        self.colors_fg = colors_fg
        self.colors_fg_bokeh = colors_fg_bokeh
        self.layer = layer
        self.label_layer = label_layer
        self.marker = marker
    def add_target_ba(self, ba):
        target_placeholder = {
            "name": self.name,
            "name_disambiguous": self.name_disambiguous,
            "colors_fg": self.colors_fg,
            "colors_fg_bokeh": self.colors_fg_bokeh,
            "layer": self.layer,
            "label_layer": self.label_layer,
            "marker": self.marker,    
        }
        ba.add_target(target_placeholder)        

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
    ba = BenchmarkAnalyzer('d8_single_arm_static_avoidance_perception', hardware_device_type)

    # Manipulation traces cannot be identified, since no ID is being stored in the tracepoints
    ba.set_trace_sets_filter_type('name')

    trace_plot_list = []
    # RealSense2 Frame
    trace_plot_list.append(Trace_Plot(name="robotperf_benchmarks:realsense2_frame_cb_init", layer="RealSense2 Frame", label_layer=1))
    trace_plot_list.append(Trace_Plot(name="robotperf_benchmarks:realsense2_frame_cb_fini", layer="RealSense2 Frame", label_layer=1))
    # URDF Filter
    trace_plot_list.append(Trace_Plot(name="realtime_urdf_filter:urdf_filter_cb_init", layer="URDF Filter", label_layer=2))
    trace_plot_list.append(Trace_Plot(name="realtime_urdf_filter:urdf_filter_init", layer="URDF Filter", label_layer=2))
    trace_plot_list.append(Trace_Plot(name="realtime_urdf_filter:urdf_filter_fini", layer="URDF Filter", label_layer=2))
    trace_plot_list.append(Trace_Plot(name="realtime_urdf_filter:urdf_filter_cb_fini", layer="URDF Filter", label_layer=2))
    # Distance Calculation
    trace_plot_list.append(Trace_Plot(name="dual_arm_static_avoidance:dual_arm_distance_calculation_cb_init", layer="Distance Calculation", label_layer=3))
    trace_plot_list.append(Trace_Plot(name="dual_arm_static_avoidance:dual_arm_distance_calculation_init", layer="Distance Calculation", label_layer=3))
    trace_plot_list.append(Trace_Plot(name="dual_arm_static_avoidance:distance_calculation_process_cp_fpga_part1_init", layer="Distance Calculation", label_layer=3))
    trace_plot_list.append(Trace_Plot(name="dual_arm_static_avoidance:distance_calculation_process_cp_fpga_part1_fini", layer="Distance Calculation", label_layer=3))
    trace_plot_list.append(Trace_Plot(name="dual_arm_static_avoidance:distance_calculation_send_data_to_driver_init", layer="Distance Calculation", label_layer=3))
    trace_plot_list.append(Trace_Plot(name="dual_arm_static_avoidance:distance_calculation_send_data_to_driver_fini", layer="Distance Calculation", label_layer=3))
    trace_plot_list.append(Trace_Plot(name="dual_arm_static_avoidance:distance_calculation_send_data_to_fpga_init", layer="Distance Calculation", label_layer=3))
    trace_plot_list.append(Trace_Plot(name="dual_arm_static_avoidance:distance_calculation_send_data_to_fpga_fini", layer="Distance Calculation", label_layer=3))
    trace_plot_list.append(Trace_Plot(name="dual_arm_static_avoidance:distance_calculation_fpga_operation_init", layer="Distance Calculation", label_layer=3))
    trace_plot_list.append(Trace_Plot(name="dual_arm_static_avoidance:distance_calculation_fpga_operation_fini", layer="Distance Calculation", label_layer=3))
    trace_plot_list.append(Trace_Plot(name="dual_arm_static_avoidance:distance_calculation_read_from_fpga_init", layer="Distance Calculation", label_layer=3))
    trace_plot_list.append(Trace_Plot(name="dual_arm_static_avoidance:distance_calculation_read_from_fpga_fini", layer="Distance Calculation", label_layer=3))
    trace_plot_list.append(Trace_Plot(name="dual_arm_static_avoidance:distance_calculation_read_from_driver_init", layer="Distance Calculation", label_layer=3))
    trace_plot_list.append(Trace_Plot(name="dual_arm_static_avoidance:distance_calculation_read_from_driver_fini", layer="Distance Calculation", label_layer=3))
    trace_plot_list.append(Trace_Plot(name="dual_arm_static_avoidance:dual_arm_distance_calculation_fini", layer="Distance Calculation", label_layer=3))
    trace_plot_list.append(Trace_Plot(name="dual_arm_static_avoidance:dual_arm_distance_calculation_cb_fini", layer="Distance Calculation", label_layer=3))
    # Distance Evaluation       
    trace_plot_list.append(Trace_Plot(name="dual_arm_static_avoidance:dual_arm_distance_evaluation_cb_init", layer="Distance Evaluation", label_layer=4))
    trace_plot_list.append(Trace_Plot(name="dual_arm_static_avoidance:dual_arm_distance_evaluation_init", layer="Distance Evaluation", label_layer=4))
    trace_plot_list.append(Trace_Plot(name="dual_arm_static_avoidance:dual_arm_distance_evaluation_fini", layer="Distance Evaluation", label_layer=4))
    trace_plot_list.append(Trace_Plot(name="dual_arm_static_avoidance:dual_arm_distance_evaluation_cb_fini", layer="Distance Evaluation", label_layer=4))
    # Control Update
    trace_plot_list.append(Trace_Plot(name="dual_arm_static_avoidance:dual_arm_control_update_cb_init", layer="Control Update", label_layer=5))
    trace_plot_list.append(Trace_Plot(name="dual_arm_static_avoidance:dual_arm_control_update_init", layer="Control Update", label_layer=5))
    trace_plot_list.append(Trace_Plot(name="dual_arm_static_avoidance:dual_arm_control_update_fini", layer="Control Update", label_layer=5))
    trace_plot_list.append(Trace_Plot(name="dual_arm_static_avoidance:dual_arm_control_update_cb_fini", layer="Control Update", label_layer=5))       

    if hardware_device_type == 'cpu':
        # add parameters for analyzing the traces
        ## using message header id
        target_chain = []
        for trace in trace_plot_list:
            target_chain.append(trace.name)
        
        for trace in trace_plot_list:
            trace.add_target_ba(ba)
            
    ba.draw_tracepoints_fpga(trace_path)

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
            'python3', os.path.abspath(__file__),
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
