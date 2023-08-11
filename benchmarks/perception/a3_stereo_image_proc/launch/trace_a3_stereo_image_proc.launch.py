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
        session_name="a3_stereo_image_proc",
        events_ust=[
            "robotperf_benchmarks:*",
            "ros2_image_pipeline:*",
            "ros2:*"
        ]
        + DEFAULT_EVENTS_ROS,
        context_fields={
                'kernel': [],
                'userspace': ['vpid', 'vtid', 'procname'],
        },
    )
 
    perception_container = ComposableNodeContainer(
        name="perception_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="a1_perception_2nodes",
                plugin="robotperf::perception::ImageInputComponent",
                name="image_input_component",
                namespace="robotperf",
                parameters=[
                    {"input_topic_name":"/robotperf/input/left_input/left_image_raw"}
                ],
                remappings=[
                    ("image", "/hawk_0_left_rgb_image"),
                    ("camera_info", "/hawk_0_left_rgb_camera_info")
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package="a1_perception_2nodes",
                plugin="robotperf::perception::ImageInputComponent",
                name="image_input_component",
                namespace="robotperf",
                parameters=[
                    {"input_topic_name":"/robotperf/input/right_input/right_image_raw"}
                ],
                remappings=[
                    ("image", "/hawk_0_right_rgb_image"),
                    ("camera_info", "/hawk_0_right_rgb_camera_info")
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                namespace="robotperf/benchmark",
                package="stereo_image_proc",
                plugin="stereo_image_proc::DisparityNode",
                name="stereo_image_proc_disparity_node",
                remappings=[
                    ('left/camera_info', '/robotperf/input/left_input/camera_info'),
                    ('left/image_rect', '/robotperf/input/left_input/left_image_raw'),
                    ('right/camera_info', '/robotperf/input/right_input/camera_info'),
                    ('right/image_rect', '/robotperf/input/right_input/right_image_raw'),                    
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                namespace="robotperf/benchmark",
                package="stereo_image_proc",
                plugin="stereo_image_proc::PointCloudNode",
                name="stereo_image_proc_pc_node",
                remappings=[
                    ('left/camera_info', '/robotperf/input/left_input/camera_info'),
                    ('left/image_rect_color', '/robotperf/input/left_input/left_image_raw'),
                    ('right/camera_info', '/robotperf/input/right_input/camera_info'),
                    ('disparity', '/robotperf/benchmark/disparity'),                         
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package="a8_stereo_image_proc_pc",
                namespace="robotperf",
                plugin="robotperf::perception::PcOutputComponent",
                name="pc_output_component",
                parameters=[
                    {"output_topic_name":"/robotperf/benchmark/points2"}
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
