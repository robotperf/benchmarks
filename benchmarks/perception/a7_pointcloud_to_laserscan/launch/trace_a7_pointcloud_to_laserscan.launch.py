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
        session_name="a7_pointcloud_to_laserscan",
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
            

            # Place Input Tracepoint
            ComposableNode(
                namespace="robotperf/input",
                package="a7_pointcloud_to_laserscan",
                plugin="robotperf::perception::PointCloudInputComponent",
                name="pointcloud_input_component",
                parameters=[
                    {"input_topic_name":"/robotperf/input/pandar_xt_32_0_lidar"}
                ],
                remappings=[
                    ("cloud", "/pandar_xt_32_0_lidar"),
                ], 
                extra_arguments=[{'use_intra_process_comms': True}],
            ),


            # Convert Depth Image to Laserscan (node of interest)
            ComposableNode(
                namespace="robotperf/benchmark",
                package='pointcloud_to_laserscan',
                plugin='pointcloud_to_laserscan::PointCloudToLaserScanNode',
                name='pointcloud_to_laserscan',
                remappings=[
                    ('cloud_in', '/robotperf/input/pandar_xt_32_0_lidar'),
                ], 
                parameters=[ {'scan_time': 0.000000001} 
                ]
            ), 

            # Record Final Tracepoint once scan is produced
            ComposableNode(
                namespace="robotperf",
                package='a6_depthimage_to_laserscan', 
                plugin='robotperf::perception::LaserscanOutputComponent', 
                name='laserscan_output_component',
                parameters=[
                    {'output_topic_name': '/robotperf/benchmark/scan'}
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
