#
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
        session_name="a2_rectify",
        events_ust=[
            "robotperf_benchmarks:*",
            "ros2_image_pipeline:*",
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

    perception_container = ComposableNodeContainer(
        name="perception_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                name='DataLoaderNode',
                namespace=TestRectifyNode.generate_namespace(),
                package='ros2_benchmark',
                plugin='ros2_benchmark::DataLoaderNode',
                remappings=[('camera/image_raw', 'data_loader/image'),
                            ('camera/camera_info', 'data_loader/camera_info')]
            )
            ComposableNode(
                name='PlaybackNode',
                namespace=TestRectifyNode.generate_namespace(),
                package='ros2_benchmark',
                plugin='ros2_benchmark::PlaybackNode',
                parameters=[{
                    'data_formats': [
                        'sensor_msgs/msg/Image',
                        'sensor_msgs/msg/CameraInfo'],
                }],
                remappings=[('buffer/input0', 'data_loader/image'),
                            ('input0', 'image_raw'),
                            ('buffer/input1', 'data_loader/camera_info'),
                            ('input1', 'camera_info')],
            )
            ComposableNode(
                package="a1_perception_2nodes",
                plugin="robotperf::perception::ImageInputComponent",
                name="image_input_component",
                remappings=[
                    ("image", "/camera/image_raw"),
                    ("camera_info", "/camera/camera_info"),
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                namespace="benchmark",
                package="image_proc",
                plugin="image_proc::RectifyNode",
                name="rectify_node",
                remappings=[
                    ("image", "/input"),
                    ("camera_info", "/camera/camera_info"),
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package="a1_perception_2nodes",
                plugin="robotperf::perception::ImageOutputComponent",
                name="image_output_component",
                remappings=[
                    ("image", "/benchmark/image_rect"),
                    ("camera_info", "/camera/camera_info"),
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
        output="screen",
    )

    return LaunchDescription([
        # LTTng tracing
        trace,
        # image pipeline
        perception_container        
    ])