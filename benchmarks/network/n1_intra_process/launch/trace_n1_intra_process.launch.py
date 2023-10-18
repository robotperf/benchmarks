#
#    @@@@@@@@@@@@@@@@@@@@
#    @@@@@@@@@&@@@&&@@@@@
#    @@@@@ @@  @@    @@@@
#    @@@@@ @@  @@    @@@@
#    @@@@@ @@  @@    @@@@ Copyright (c) 2023, Acceleration Robotics®
#    @@@@@ @@  @@    @@@@ Author: Víctor Mayoral Vilches <victor@accelerationrobotics.com>
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
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from tracetools_launch.action import Trace
from tracetools_trace.tools.names import DEFAULT_EVENTS_ROS
from tracetools_trace.tools.names import DEFAULT_EVENTS_KERNEL
from tracetools_trace.tools.names import DEFAULT_CONTEXT
 
def generate_launch_description():
     # Trace
    trace = Trace(
        session_name="n1_intra_process",
        events_ust=[
            "robotcore_rtps:*",
            "ros2:*"
            # "lttng_ust_cyg_profile*",
            # "lttng_ust_statedump*",
            # "liblttng-ust-libc-wrapper",
        ]
        + DEFAULT_EVENTS_ROS,
        context_fields={
                'kernel': [],
                'userspace': ['vpid', 'vtid', 'procname'],
        },
        # events_kernel=DEFAULT_EVENTS_KERNEL,
        # context_names=DEFAULT_CONTEXT,
    )

    network_container = ComposableNodeContainer(
        name="network_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt", # if not _mt (multi-thread), it will not work
        composable_node_descriptions=[
            ComposableNode(
                namespace="robotcore",
                package="cpp_loopback",
                plugin="robotcore::network::ClientComponent",
                name="client_component",
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package="cpp_loopback",
                namespace="robotcore",
                plugin="robotcore::network::ServerComponent",
                name="server_component",
                extra_arguments=[{'use_intra_process_comms': True}],
                # parameters=[
                #     {
                #         "iterations": 10000,
                #     }
                # ],
            ),
            
        ],
        output="screen",
    )

    return LaunchDescription([
        # LTTng tracing
        trace,
        network_container
    ])
