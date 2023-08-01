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

            # Create disparity map using stereo images
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

            # Convert disparity map to depth image
            ComposableNode(
                namespace="robotperf/preprocessing",
                package="a6_depthimage_to_laserscan",
                plugin="robotperf::perception::DisparityToDepthImageComponent",
                name="disparity_to_depthimage_node",
                remappings=[
                    ('/depth', '/robotperf/preprocessing/depth') , 
                    ('/depth_camera_info', '/robotperf/preprocessing/depth_camera_info')                  
                ],
                parameters=[
                    {"disparity_topic_name":"/robotperf/preprocessing/disparity", 
                     "disparity_camera_info": "/hawk_0_left_rgb_camera_info"}
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),

            # Convert Depth Image to Laserscan
            ComposableNode(
                namespace="robotperf/benchmark",
                package='depthimage_to_laserscan',
                plugin='depthimage_to_laserscan::DepthImageToLaserScanROS',
                name='depthimage_to_laserscan',
                remappings=[
                    ('depth_camera_info', '/robotperf/preprocessing/depth_camera_info'),
                    ('depth', '/robotperf/preprocessing/depth'),                  
                ], 
                parameters=[ {'range_min': 0.1,
                              'range_max': 200.0,
                              'scan_time': 0.000000001, 
                              'output_frame_id': 'camera_depth_frame'} # Set frame in RVIZ to this to visualize scan
                            ]
                
            ), 

        ],
        output='screen',
    )

    ld.add_action(container)

    return ld
