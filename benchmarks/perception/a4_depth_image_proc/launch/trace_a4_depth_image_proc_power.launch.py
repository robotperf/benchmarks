#
#    @@@@@@@@@@@@@@@@@@@@
#    @@@@@@@@@&@@@&&@@@@@
#    @@@@@ @@  @@    @@@@
#    @@@@@ @@  @@    @@@@
#    @@@@@ @@  @@    @@@@ Copyright (c) 2023, Acceleration Robotics®
#    @@@@@ @@  @@    @@@@ Author: Martiño Crespo <martinho@accelerationrobotics.com>
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
        session_name="a4_depth_image_proc",
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
                    {"power_lib": "rapl"}
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
            # Rectify Image
            ComposableNode(
                package='image_proc',
                plugin='image_proc::RectifyNode',
                namespace='robotperf/preprocessing',
                name='rectify_color_node',
                remappings=[
                    ('image', '/camera/image_raw'),
                    ('camera_info', "/camera/camera_info"),
                    ('image_rect', '/robotperf/preprocessing/rgb/image_rect_color')
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            # Rectify Depth Image
            ComposableNode(
                package='image_proc',
                plugin='image_proc::RectifyNode',
                namespace='robotperf/preprocessing',
                name='rectify_depth_node',
                remappings=[
                    ('image', '/camera/depth/image_raw'),
                    ('camera_info', "/camera/depth/camera_info"),
                    ('image_rect', '/robotperf/preprocessing/depth_registered/image_rect')
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
            ), 
            # Input Image Rectify Component
            ComposableNode(
                package="a1_perception_2nodes",
                plugin="robotperf::perception::ImageInputComponent",
                name="image_input_component",
                namespace='robotperf',
                parameters=[
                    {"input_topic_name":"/robotperf/input/rgb/image_rect_color"}
                ],
                remappings=[
                    ("image", "/robotperf/preprocessing/rgb/image_rect_color"),
                    ("camera_info", "/camera/camera_info"),
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            # Input Depth Image Rectify Component
            ComposableNode(
                package="a1_perception_2nodes",
                plugin="robotperf::perception::ImageInputComponent",
                name="image_input_component",
                namespace='robotperf',
                parameters=[
                    {"input_topic_name":"/robotperf/input/depth_registered/image_rect"}
                ],
                remappings=[
                    ("image", "/robotperf/preprocessing/depth_registered/image_rect"),
                    ("camera_info", "/camera/depth/camera_info"),
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),

            ComposableNode(
                namespace="robotperf/benchmark",
                package="depth_image_proc",
                plugin="depth_image_proc::PointCloudXyzrgbNode",
                name="depth_image_to_pointcloud_node",
                remappings=[
                    ('rgb/camera_info', '/robotperf/input/rgb/camera_info'),
                    ('rgb/image_rect_color', '/robotperf/input/rgb/image_rect_color'),
                    ('depth_registered/image_rect', '/robotperf/input/depth_registered/image_rect')
                ],                  
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package="a4_depth_image_proc",
                plugin="robotperf::perception::PointCloudOutputComponent",
                namespace='robotperf',
                name="point_cloud_output_component",
                parameters=[
                    {"output_topic_name":"/robotperf/benchmark/points"}
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
        perception_container,
        power_container
    ])


