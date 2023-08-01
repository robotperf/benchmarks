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
        session_name="a6_depthimage_to_laserscan",
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

            # Place Input Tracepoint
            ComposableNode(
                namespace="robotperf/input",
                package="a1_perception_2nodes",
                plugin="robotperf::perception::ImageInputComponent",
                name="image_input_component",
                parameters=[
                    {"input_topic_name":"/robotperf/input"}
                ],
                remappings=[
                    ("image", "/robotperf/preprocessing/depth"),
                    ("camera_info", "/robotperf/preprocessing/depth_camera_info"),
                ]
            ),


            # Convert Depth Image to Laserscan
            ComposableNode(
                namespace="robotperf/benchmark",
                package='depthimage_to_laserscan',
                plugin='depthimage_to_laserscan::DepthImageToLaserScanROS',
                name='depthimage_to_laserscan',
                remappings=[
                    ('depth_camera_info', '/robotperf/preprocessing/depth_camera_info'),
                    ('depth', '/robotperf/input'),                  
                ], 
                parameters=[ {'range_min': 0.1,
                              'range_max': 200.0,
                              'scan_time': 0.000000001, 
                              'output_frame_id': 'camera_depth_frame'} # Set frame in RVIZ to this to visualize scan
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
        perception_container,
        power_container
    ])
