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

def analyze_planning(argv):
    
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
    ba = BenchmarkAnalyzer('d1_xarm6_planning_and_traj_execution', hardware_device_type)

    # Manipulation traces cannot be identified, since no ID is being stored in the tracepoints
    ba.set_trace_sets_filter_type('name')

    if hardware_device_type == 'cpu':
        # add parameters for analyzing the traces
        ## using message header id
        target_chain = [
            "robotcore_manipulation:robotcore_moveit2_planning_cb_init",        # 0
            "robotcore_manipulation:robotcore_moveit2_planning_init",           # 1
            "robotcore_manipulation:robotcore_moveit2_planning_fini",           # 2
            "robotcore_manipulation:robotcore_moveit2_planning_cb_fini",        # 3
            "robotcore_manipulation:robotcore_moveit2_traj_execution_cb_init",  # 4
            "robotcore_manipulation:robotcore_moveit2_traj_execution_init",     # 5
            "robotcore_manipulation:robotcore_moveit2_traj_execution_fini",     # 6
            "robotcore_manipulation:robotcore_moveit2_traj_execution_cb_fini"   # 7
        ]

        ba.add_target(
            {
                "name": "robotcore_manipulation:robotcore_moveit2_planning_cb_init",
                "name_disambiguous": "robotcore_manipulation:robotcore_moveit2_planning_cb_init",
                "colors_fg": "yellow",
                "colors_fg_bokeh": "salmon",
                "layer": "userland",
                "label_layer": 4,
                "marker": "plus",
            }
        )
        ba.add_target(
            {
                "name": "robotcore_manipulation:robotcore_moveit2_planning_init",
                "name_disambiguous": "robotcore_manipulation:robotcore_moveit2_planning_init",
                "colors_fg": "red",
                "colors_fg_bokeh": "darksalmon",
                "layer": "userland",
                "label_layer": 4,
                "marker": "plus",
            }
        )
        ba.add_target(
            {
                "name": "robotcore_manipulation:robotcore_moveit2_planning_fini",
                "name_disambiguous": "robotcore_manipulation:robotcore_moveit2_planning_fini",
                "colors_fg": "red",
                "colors_fg_bokeh": "lightcoral",
                "layer": "userland",
                "label_layer": 4,
                "marker": "plus",
            }
        )
        ba.add_target(
            {
                "name": "robotcore_manipulation:robotcore_moveit2_planning_cb_fini",
                "name_disambiguous": "robotcore_manipulation:robotcore_moveit2_planning_cb_fini",
                "colors_fg": "yellow",
                "colors_fg_bokeh": "darkred",
                "layer": "userland",
                "label_layer": 4,
                "marker": "plus",
            }
        )
        
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
            ba.analyze_latency(trace_path, add_power, debug=False)
        elif metric == 'throughput':
            ba.analyze_throughput(trace_path, add_power, debug=False)
        elif metric == 'power': 
            if num_metrics == 0: # launch independently iff no other metric is requested
                total_consumption = ba.analyze_power(trace_path)
                print("The average consumption is {} W".format(total_consumption))
        else:
            print('The metric ' + metric + ' is not yet implemented\n')

def analyze_traj_execution(argv):
    
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
    ba = BenchmarkAnalyzer('d1_xarm6_planning_and_traj_execution', hardware_device_type)

    # Manipulation traces cannot be identified, since no ID is being stored in the tracepoints
    ba.set_trace_sets_filter_type('name')

    if hardware_device_type == 'cpu':
        # add parameters for analyzing the traces
        ## using message header id
        target_chain = [        # 2
            "robotcore_manipulation:robotcore_moveit2_traj_execution_cb_init",  # 0
            "robotcore_manipulation:robotcore_moveit2_traj_execution_init",     # 1
            "robotcore_manipulation:robotcore_moveit2_traj_execution_fini",     # 2
            "robotcore_manipulation:robotcore_moveit2_traj_execution_cb_fini"   # 3
        ]

        ba.add_target(
            {
                "name": "robotcore_manipulation:robotcore_moveit2_traj_execution_cb_init",
                "name_disambiguous": "robotcore_manipulation:robotcore_moveit2_traj_execution_cb_init",
                "colors_fg": "yellow",
                "colors_fg_bokeh": "salmon",
                "layer": "userland",
                "label_layer": 4,
                "marker": "plus",
            }
        )
        ba.add_target(
            {
                "name": "robotcore_manipulation:robotcore_moveit2_traj_execution_init",
                "name_disambiguous": "robotcore_manipulation:robotcore_moveit2_traj_execution_init",
                "colors_fg": "red",
                "colors_fg_bokeh": "darksalmon",
                "layer": "userland",
                "label_layer": 4,
                "marker": "plus",
            }
        )
        ba.add_target(
            {
                "name": "robotcore_manipulation:robotcore_moveit2_traj_execution_fini",
                "name_disambiguous": "robotcore_manipulation:robotcore_moveit2_traj_execution_fini",
                "colors_fg": "red",
                "colors_fg_bokeh": "lightcoral",
                "layer": "userland",
                "label_layer": 4,
                "marker": "plus",
            }
        )
        ba.add_target(
            {
                "name": "robotcore_manipulation:robotcore_moveit2_traj_execution_cb_fini",
                "name_disambiguous": "robotcore_manipulation:robotcore_moveit2_traj_execution_cb_fini",
                "colors_fg": "yellow",
                "colors_fg_bokeh": "darkred",
                "layer": "userland",
                "label_layer": 4,
                "marker": "plus",
            }
        )
        
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
            ba.analyze_latency(trace_path, add_power, debug=False)
        elif metric == 'throughput':
            ba.analyze_throughput(trace_path, add_power, debug=False)
        elif metric == 'power': 
            if num_metrics == 0: # launch independently iff no other metric is requested
                total_consumption = ba.analyze_power(trace_path)
                print("The average consumption is {} W".format(total_consumption))
        else:
            print('The metric ' + metric + ' is not yet implemented\n')
    
def analyze_collision_checking(argv):
    
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
    ba = BenchmarkAnalyzer('d1_xarm6_planning_and_traj_execution', hardware_device_type)

    # Manipulation traces cannot be identified, since no ID is being stored in the tracepoints
    ba.set_trace_sets_filter_type('name')

    if hardware_device_type == 'cpu':
        # add parameters for analyzing the traces
        ## using message header id
        target_chain = [
            "robotcore_manipulation:robotcore_moveit2_fcl_check_robot_collision_cb_init",       # 0
            "robotcore_manipulation:robotcore_moveit2_fcl_check_robot_collision_init",          # 1
            "robotcore_manipulation:robotcore_moveit2_fcl_check_robot_collision_fini",          # 2
            "robotcore_manipulation:robotcore_moveit2_fcl_check_robot_collision_cb_fini",       # 3
            "robotcore_manipulation:robotcore_moveit2_fcl_check_self_collision_cb_init",        # 4
            "robotcore_manipulation:robotcore_moveit2_fcl_check_self_collision_init",           # 5
            "robotcore_manipulation:robotcore_moveit2_fcl_check_self_collision_fini",           # 6
            "robotcore_manipulation:robotcore_moveit2_fcl_check_self_collision_cb_fini"         # 7
        ]

        ba.add_target(
            {
                "name": "robotcore_manipulation:robotcore_moveit2_fcl_check_robot_collision_cb_init",
                "name_disambiguous": "robotcore_manipulation:robotcore_moveit2_fcl_check_robot_collision_cb_init",
                "colors_fg": "yellow",
                "colors_fg_bokeh": "salmon",
                "layer": "userland",
                "label_layer": 4,
                "marker": "plus",
            }
        )
        ba.add_target(
            {
                "name": "robotcore_manipulation:robotcore_moveit2_fcl_check_robot_collision_init",
                "name_disambiguous": "robotcore_manipulation:robotcore_moveit2_fcl_check_robot_collision_init",
                "colors_fg": "red",
                "colors_fg_bokeh": "darksalmon",
                "layer": "userland",
                "label_layer": 4,
                "marker": "plus",
            }
        )
        ba.add_target(
            {
                "name": "robotcore_manipulation:robotcore_moveit2_fcl_check_robot_collision_fini",
                "name_disambiguous": "robotcore_manipulation:robotcore_moveit2_fcl_check_robot_collision_fini",
                "colors_fg": "red",
                "colors_fg_bokeh": "lightcoral",
                "layer": "userland",
                "label_layer": 4,
                "marker": "plus",
            }
        )
        ba.add_target(
            {
                "name": "robotcore_manipulation:robotcore_moveit2_fcl_check_robot_collision_cb_fini",
                "name_disambiguous": "robotcore_manipulation:robotcore_moveit2_fcl_check_robot_collision_cb_fini",
                "colors_fg": "yellow",
                "colors_fg_bokeh": "darkred",
                "layer": "userland",
                "label_layer": 4,
                "marker": "plus",
            }
        )
        
        ba.add_target(
            {
                "name": "robotcore_manipulation:robotcore_moveit2_fcl_check_self_collision_cb_init",
                "name_disambiguous": "robotcore_manipulation:robotcore_moveit2_fcl_check_self_collision_cb_init",
                "colors_fg": "yellow",
                "colors_fg_bokeh": "salmon",
                "layer": "userland",
                "label_layer": 4,
                "marker": "plus",
            }
        )
        ba.add_target(
            {
                "name": "robotcore_manipulation:robotcore_moveit2_fcl_check_self_collision_init",
                "name_disambiguous": "robotcore_manipulation:robotcore_moveit2_fcl_check_self_collision_init",
                "colors_fg": "red",
                "colors_fg_bokeh": "darksalmon",
                "layer": "userland",
                "label_layer": 4,
                "marker": "plus",
            }
        )
        ba.add_target(
            {
                "name": "robotcore_manipulation:robotcore_moveit2_fcl_check_self_collision_fini",
                "name_disambiguous": "robotcore_manipulation:robotcore_moveit2_fcl_check_self_collision_fini",
                "colors_fg": "red",
                "colors_fg_bokeh": "lightcoral",
                "layer": "userland",
                "label_layer": 4,
                "marker": "plus",
            }
        )
        ba.add_target(
            {
                "name": "robotcore_manipulation:robotcore_moveit2_fcl_check_self_collision_cb_fini",
                "name_disambiguous": "robotcore_manipulation:robotcore_moveit2_fcl_check_self_collision_cb_fini",
                "colors_fg": "yellow",
                "colors_fg_bokeh": "darkred",
                "layer": "userland",
                "label_layer": 4,
                "marker": "plus",
            }
        )
        
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
            ba.analyze_latency(trace_path, add_power, debug=False)
        elif metric == 'throughput':
            ba.analyze_throughput(trace_path, add_power, debug=False)
        elif metric == 'power': 
            if num_metrics == 0: # launch independently iff no other metric is requested
                total_consumption = ba.analyze_power(trace_path)
                print("The average consumption is {} W".format(total_consumption))
        else:
            print('The metric ' + metric + ' is not yet implemented\n')

def analyze_inverse_kinematics(argv):
    
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
    ba = BenchmarkAnalyzer('d1_xarm6_planning_and_traj_execution', hardware_device_type)

    # Manipulation traces cannot be identified, since no ID is being stored in the tracepoints
    ba.set_trace_sets_filter_type('name')

    if hardware_device_type == 'cpu':
        # add parameters for analyzing the traces
        ## using message header id
        target_chain = [
            "robotcore_manipulation:robotcore_moveit2_inverse_kinematics_kdl_cb_init:",      # 0
            "robotcore_manipulation:robotcore_moveit2_inverse_kinematics_kdl_init",          # 1
            "robotcore_manipulation:robotcore_moveit2_inverse_kinematics_kdl_fini",          # 2
            "robotcore_manipulation:robotcore_moveit2_inverse_kinematics_kdl_cb_fini",       # 3
        ]

        ba.add_target(
            {
                "name": "robotcore_manipulation:robotcore_moveit2_inverse_kinematics_kdl_cb_init",
                "name_disambiguous": "robotcore_manipulation:robotcore_moveit2_inverse_kinematics_kdl_cb_init",
                "colors_fg": "yellow",
                "colors_fg_bokeh": "salmon",
                "layer": "userland",
                "label_layer": 4,
                "marker": "plus",
            }
        )
        ba.add_target(
            {
                "name": "robotcore_manipulation:robotcore_moveit2_inverse_kinematics_kdl_init",
                "name_disambiguous": "robotcore_manipulation:robotcore_moveit2_inverse_kinematics_kdl_init",
                "colors_fg": "red",
                "colors_fg_bokeh": "darksalmon",
                "layer": "userland",
                "label_layer": 4,
                "marker": "plus",
            }
        )
        ba.add_target(
            {
                "name": "robotcore_manipulation:robotcore_moveit2_inverse_kinematics_kdl_fini",
                "name_disambiguous": "robotcore_manipulation:robotcore_moveit2_inverse_kinematics_kdl_fini",
                "colors_fg": "red",
                "colors_fg_bokeh": "lightcoral",
                "layer": "userland",
                "label_layer": 4,
                "marker": "plus",
            }
        )
        ba.add_target(
            {
                "name": "robotcore_manipulation:robotcore_moveit2_inverse_kinematics_kdl_cb_fini",
                "name_disambiguous": "robotcore_manipulation:robotcore_moveit2_inverse_kinematics_kdl_cb_fini",
                "colors_fg": "yellow",
                "colors_fg_bokeh": "darkred",
                "layer": "userland",
                "label_layer": 4,
                "marker": "plus",
            }
        )
        
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
            ba.analyze_latency(trace_path, add_power, debug=False)
        elif metric == 'throughput':
            ba.analyze_throughput(trace_path, add_power, debug=False)
        elif metric == 'power': 
            if num_metrics == 0: # launch independently iff no other metric is requested
                total_consumption = ba.analyze_power(trace_path)
                print("The average consumption is {} W".format(total_consumption))
        else:
            print('The metric ' + metric + ' is not yet implemented\n')

def analyze_direct_kinematics(argv):
    
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
    ba = BenchmarkAnalyzer('d1_xarm6_planning_and_traj_execution', hardware_device_type)

    # Manipulation traces cannot be identified, since no ID is being stored in the tracepoints
    ba.set_trace_sets_filter_type('name')

    if hardware_device_type == 'cpu':
        # add parameters for analyzing the traces
        ## using message header id
        target_chain = [
            "robotcore_manipulation:robotcore_moveit2_direct_kinematics_cb_init",       # 0
            "robotcore_manipulation:robotcore_moveit2_direct_kinematics_init",          # 1
            "robotcore_manipulation:robotcore_moveit2_direct_kinematics_fini",          # 2
            "robotcore_manipulation:robotcore_moveit2_direct_kinematics_cb_fini",       # 3
        ]

        ba.add_target(
            {
                "name": "robotcore_manipulation:robotcore_moveit2_direct_kinematics_cb_init",
                "name_disambiguous": "robotcore_manipulation:robotcore_moveit2_direct_kinematics_cb_init",
                "colors_fg": "yellow",
                "colors_fg_bokeh": "salmon",
                "layer": "userland",
                "label_layer": 4,
                "marker": "plus",
            }
        )
        ba.add_target(
            {
                "name": "robotcore_manipulation:robotcore_moveit2_direct_kinematics_init",
                "name_disambiguous": "robotcore_manipulation:robotcore_moveit2_direct_kinematics_init",
                "colors_fg": "red",
                "colors_fg_bokeh": "darksalmon",
                "layer": "userland",
                "label_layer": 4,
                "marker": "plus",
            }
        )
        ba.add_target(
            {
                "name": "robotcore_manipulation:robotcore_moveit2_direct_kinematics_fini",
                "name_disambiguous": "robotcore_manipulation:robotcore_moveit2_direct_kinematics_fini",
                "colors_fg": "red",
                "colors_fg_bokeh": "lightcoral",
                "layer": "userland",
                "label_layer": 4,
                "marker": "plus",
            }
        )
        ba.add_target(
            {
                "name": "robotcore_manipulation:robotcore_moveit2_direct_kinematics_cb_fini",
                "name_disambiguous": "robotcore_manipulation:robotcore_moveit2_direct_kinematics_cb_fini",
                "colors_fg": "yellow",
                "colors_fg_bokeh": "darkred",
                "layer": "userland",
                "label_layer": 4,
                "marker": "plus",
            }
        )
        
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
            ba.analyze_latency(trace_path, add_power, debug=False)
        elif metric == 'throughput':
            ba.analyze_throughput(trace_path, add_power, debug=False)
        elif metric == 'power': 
            if num_metrics == 0: # launch independently iff no other metric is requested
                total_consumption = ba.analyze_power(trace_path)
                print("The average consumption is {} W".format(total_consumption))
        else:
            print('The metric ' + metric + ' is not yet implemented\n')

def analyze_tf2_operations(argv):
    tf_tree = FrameHierarchy()
    tf_tree.add_frame("world", "link_base")

    # Parse the command-line arguments
    parser = argparse.ArgumentParser()
    parser.add_argument('--hardware_device_type', type=str, help='Hardware Device Type (e.g. cpu or fpga)', default ='cpu')
    parser.add_argument('--trace_path', type=str, help='Path to trace files (e.g. /tmp/analysis/trace)', default = '/tmp/analysis/trace')
    parser.add_argument('--metrics', type=str, help='List of metrics to be analyzed (e.g. latency and/or throughput)', default = ['latency'])
    parser.add_argument('--integrated', type=str, help='Integrated or separated version of the Resize and Rectify nodes (only for fpga now)', default='false') 
    parser.add_argument('--trace_set_filter_type', type=str, help='Weather to filter trace sets by name or by unique ID', default='name') 
    args = parser.parse_args(argv)

    # Get the values of the arguments
    hardware_device_type = args.hardware_device_type
    trace_path = args.trace_path
    metrics_string = args.metrics
    metrics_elements = [element.strip() for element in metrics_string.strip("[]").split(",")]
    metrics = json.loads(json.dumps(metrics_elements))
    integrated = args.integrated
    # trace_set_filter_type = args.trace_set_filter_type

    # Instantiate the class
    ba = BenchmarkAnalyzer('d1_xarm6_planning_and_traj_execution', hardware_device_type, tf_tree)
    ba.set_trace_sets_filter_type(filter_type="UID")

    if hardware_device_type == 'cpu':
        # add parameters for analyzing the traces
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


def print_total_benchmark_time(argv):
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
    ba = BenchmarkAnalyzer('d1_xarm6_planning_and_traj_execution', hardware_device_type)

    target_chain_name = [
        'robotcore_moveit2_planning',
        'robotcore_moveit2_traj_execution',
        'robotcore_moveit2_fcl_check', 
        'robotcore_moveit2_inverse_kinematics',
        'robotcore_moveit2_direct_kinematics',
        'robotcore_tf2'
    ]

    ba.get_time_spent_in_specified_target_chains(trace_path, target_chain_name)

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
            'python3', "src/benchmarks/benchmarks/manipulation/d1_xarm6_planning_and_traj_execution/launch/analyze_d1_xarm6_planning_and_traj_execution.launch.py",
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
    analyze_planning(sys.argv[1:])
    analyze_traj_execution(sys.argv[1:])
    analyze_collision_checking(sys.argv[1:])
    analyze_inverse_kinematics(sys.argv[1:])
    analyze_direct_kinematics(sys.argv[1:])
    
    analyze_tf2_operations(sys.argv[1:])

    print_total_benchmark_time(sys.argv[1:])
