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
            # Rectify Image
            ComposableNode(
                package='image_proc',
                plugin='image_proc::RectifyNode',
                name='rectify_color_node',
                remappings=[
                    ('image', '/camera/image_raw'),
                    ('camera_info', "/camera/camera_info"),
                    ('image_rect', '/rgb/image_rect_color')
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            # Rectify Depth Image
            ComposableNode(
                package='image_proc',
                plugin='image_proc::RectifyNode',
                name='rectify_depth_node',
                remappings=[
                    ('image', '/camera/depth/image_raw'),
                    ('camera_info', "/camera/depth/camera_info"),
                    ('image_rect', '/depth_registered/image_rect')
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
            ), 
            # Input Image Rectify Component
            ComposableNode(
                package="a1_perception_2nodes",
                plugin="robotperf::perception::ImageInputComponent",
                name="image_input_component",
                parameters=[
                    {"input_topic_name":"/input/rgb/image_rect_color"}
                ],
                remappings=[
                    ("image", "/rgb/image_rect_color"),
                    ("camera_info", "/camera/camera_info"),
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            # Input Depth Image Rectify Component
            ComposableNode(
                package="a1_perception_2nodes",
                plugin="robotperf::perception::ImageInputComponent",
                name="image_input_component",
                parameters=[
                    {"input_topic_name":"/input/depth_registered/image_rect"}
                ],
                remappings=[
                    ("image", "/depth_registered/image_rect"),
                    ("camera_info", "/camera/depth/camera_info"),
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),

            ComposableNode(
                namespace="benchmark",
                package="depth_image_proc",
                plugin="depth_image_proc::PointCloudXyzrgbNode",
                name="depth_image_proc_disparity_node",
                remappings=[
                    ('rgb/camera_info', '/input/rgb/camera_info'),
                    ('rgb/image_rect_color', '/input/rgb/image_rect_color'),
                    ('depth_registered/image_rect', '/input/depth_registered/image_rect')
                ],                  
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package="a4_depth_image_proc",
                plugin="robotperf::perception::PointCloudOutputComponent",
                name="point_cloud_output_component",
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


