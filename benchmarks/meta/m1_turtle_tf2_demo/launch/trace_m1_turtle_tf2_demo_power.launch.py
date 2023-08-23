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
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from tracetools_launch.action import Trace
from tracetools_trace.tools.names import DEFAULT_EVENTS_ROS
from tracetools_trace.tools.names import DEFAULT_EVENTS_KERNEL
from tracetools_trace.tools.names import DEFAULT_CONTEXT

POWER_LIB = 'rapl' #os.environ.get('POWER_LIB')

def generate_launch_description():
    target_frame_arg = DeclareLaunchArgument(
        'target_frame', default_value='turtle1',
        description='Target frame name.'
    )

     # Trace
    trace = Trace(
        session_name="m1_turtle_tf2_demo",
        events_ust=[
            "robotperf_benchmarks:*",
            "robotcore_power:*",
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

    power_container = ComposableNodeContainer(
        name="power_container",
        namespace="robotcore/power",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="robotcore-power",
                namespace="robotcore/power",
                plugin="robotcore::power::PowerComponent",
                name="power_component",
                parameters=[
                    {"publish_rate": 20.0},
                    {"power_lib": POWER_LIB}
                ],
            ),
            
        ],
        output="screen",
    )
    
    sim_node = Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='sim'
    )
    
    broadcaster1_node = Node(
        package='learning_tf2_cpp',
        executable='turtle_tf2_broadcaster',
        name='broadcaster1',
        parameters=[
            {'turtlename': 'turtle1'}
        ]
    )

    broadcaster2_node = Node(
        package='learning_tf2_cpp',
        executable='turtle_tf2_broadcaster',
        name='broadcaster2',
        parameters=[
            {'turtlename': 'turtle2'}
        ]
    )

    listener_node = Node(
        package='learning_tf2_cpp',
        executable='turtle_tf2_listener',
        name='listener',
        parameters=[
            {'target_frame': LaunchConfiguration('target_frame')}
        ]
    )

    return LaunchDescription([
        target_frame_arg,
        sim_node,
        trace,
        broadcaster1_node,
        broadcaster2_node,
        listener_node,
        power_container
    ])
