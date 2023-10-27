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

POWER_LIB = os.environ.get('POWER_LIB')

def generate_launch_description():
     # Trace
    trace = Trace(
        session_name="a8_stereo_image_proc_pc",
        events_ust=[
            "robotperf_benchmarks:*",
            "ros2_image_pipeline:*",
            "robotcore_power:*",
            "ros2:*"
        ]
        + DEFAULT_EVENTS_ROS,
        context_fields={
                'kernel': [],
                'userspace': ['vpid', 'vtid', 'procname'],
        },
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
 
    perception_container = ComposableNodeContainer(
        name="perception_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                namespace="robotperf/preprocessing",
                package="stereo_image_proc",
                plugin="stereo_image_proc::DisparityNode",
                name="stereo_image_proc_disparity_node",
                remappings=[
                    ('left/camera_info', '/hawk_0_left_rgb_camera_info'),
                    ('left/image_rect', '/hawk_0_left_rgb_image'),
                    ('right/camera_info', '/hawk_0_right_rgb_camera_info'),
                    ('right/image_rect', '/hawk_0_right_rgb_image'),                    
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package="a1_perception_2nodes",
                plugin="robotperf::perception::ImageInputComponent",
                name="image_input_component_left",
                namespace="robotperf/input",
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
                name="image_input_component_right",
                namespace="robotperf/input",
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
                package="a8_stereo_image_proc_pc",
                namespace="robotperf/input",
                plugin="robotperf::perception::DisparityInputComponent",
                name="disparity_input_component",
                parameters=[
                    {"prev_topic_name":"/robotperf/preprocessing/disparity",
                    "post_topic_name":"/robotperf/input/disparity"}
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                namespace="robotperf/benchmark",
                package="stereo_image_proc",
                plugin="stereo_image_proc::PointCloudNode",
                name="stereo_image_proc_pc_node",
                remappings=[
                    ('left/image_rect_color', '/robotperf/input/left_input/left_image_raw'),
                    ('left/camera_info', '/robotperf/input/left_input/camera_info'),
                    ('right/camera_info', '/robotperf/input/right_input/camera_info'),
                    ('disparity', '/robotperf/input/disparity'),     
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
            )
        ],
        output="screen",
    )

    return LaunchDescription([
        # LTTng tracing
        trace,
        # image pipeline
        perception_container,
        power_container
    ])
