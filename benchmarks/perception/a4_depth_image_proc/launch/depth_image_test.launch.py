import os
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    container = ComposableNodeContainer(
        name='depth_image_proc_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='image_proc',
                plugin='image_proc::RectifyNode',
                name='rectify_color_node',
                remappings=[
                    ('image', '/camera/image_raw'),
                    ('camera_info', "/camera/camera_info"),
                    ('image_rect', '/rgb/image_rect_color')
                ],
            ),
            ComposableNode(
                package='image_proc',
                plugin='image_proc::RectifyNode',
                name='rectify_depth_node',
                remappings=[
                    ('image', '/camera/depth/image_raw'),
                    ('camera_info', "/camera/depth/camera_info"),
                    ('image_rect', '/depth_registered/image_rect')
                ],
            ),
            ComposableNode(
                package='depth_image_proc',
                plugin='depth_image_proc::PointCloudXyzrgbNode',
                name='depth_image_proc_disparity_node',
                remappings=[
                    ('rgb/camera_info', '/camera/camera_info'),
                    ('rgb/image_rect_color', '/rgb/image_rect_color'),
                    ('depth_registered/image_rect', '/depth_registered/image_rect')
                ],
            )
        ],
        output='screen',
    )

    ld.add_action(container)

    return ld