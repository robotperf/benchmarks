import os
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import Node


def generate_launch_description():

    ld = LaunchDescription()
    container = ComposableNodeContainer(
        name='depthimage_to_laserscan_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[

            # Convert Depth Image to Laserscan
            ComposableNode(
                namespace="robotperf/benchmark",
                package='pointcloud_to_laserscan',
                plugin='pointcloud_to_laserscan::PointCloudToLaserScanNode',
                name='pointcloud_to_laserscan_node',
                remappings=[
                    ('cloud_in', '/pandar_xt_32_0_lidar'),
                ], 
                parameters=[ {'scan_time': 0.000000001} 
                ]
                
            ), 

        ],
        output='screen',
    )

    ld.add_action(container)

    return ld
