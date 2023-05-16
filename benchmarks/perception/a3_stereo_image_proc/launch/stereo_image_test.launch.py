import os
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    container = ComposableNodeContainer(
        name='stereo_image_proc_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='stereo_image_proc',
                plugin='stereo_image_proc::DisparityNode',
                name='stereo_image_proc_disparity_node',
                remappings=[
                    ('left/camera_info', '/left_camera/camera_info'),
                    ('left/image_rect', '/left_camera/image_raw'),
                    ('right/camera_info', '/right_camera/camera_info'),
                    ('right/image_rect', '/right_camera/image_raw'),                    
                ],
            )
        ],
        output='screen',
    )

    ld.add_action(container)

    return ld