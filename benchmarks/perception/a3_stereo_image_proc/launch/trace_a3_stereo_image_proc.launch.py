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
            # "lttng_ust_cyg_profile*",
            # "lttng_ust_statedump*",
            # "liblttng-ust-libc-wrapper",
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
                parameters=[
                    {"input_topic_name":"/input/left_input/left_image_raw"}
                ],
                remappings=[
                    ("image", "/left_camera/image_raw"),
                    ("camera_info", "/left_camera/camera_info"),
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package="a1_perception_2nodes",
                plugin="robotperf::perception::ImageInputComponent",
                name="image_input_component",
                parameters=[
                    {"input_topic_name":"/input/right_input/right_image_raw"}
                ],
                remappings=[
                    ("image", "/right_camera/image_raw"),
                    ("camera_info", "/right_camera/camera_info"),
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                namespace="benchmark",
                package="stereo_image_proc",
                plugin="stereo_image_proc::DisparityNode",
                name="stereo_image_proc_disparity_node",
                remappings=[
                    ('left/camera_info', '/input/left_input/camera_info'),
                    ('left/image_rect', '/input/left_input/left_image_raw'),
                    ('right/camera_info', '/input/right_input/camera_info'),
                    ('right/image_rect', '/input/right_input/right_image_raw'),                    
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package="a3_stereo_image_proc",
                plugin="robotperf::perception::DisparityOutputComponent",
                name="disparity_output_component",
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
