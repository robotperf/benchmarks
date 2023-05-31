#
#    @@@@@@@@@@@@@@@@@@@@
#    @@@@@@@@@&@@@&&@@@@@
#    @@@@@ @@  @@    @@@@
#    @@@@@ @@  @@    @@@@
#    @@@@@ @@  @@    @@@@ Copyright (c) 2023, Acceleration Robotics®
#    @@@@@ @@  @@    @@@@ Author: Alejandra Martínez Fariña <alex@accelerationrobotics.com>
#    @@@@@ @@  @@    @@@@ 
#    @@@@@ @@  @@    @@@@ 
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

import json
from launch_ros.actions import ComposableNodeContainer
from launch.actions import DeclareLaunchArgument
from launch_ros.descriptions import ComposableNode
from launch import LaunchDescription

from tracetools_launch.action import Trace
from tracetools_trace.tools.names import DEFAULT_EVENTS_ROS
from tracetools_trace.tools.names import DEFAULT_EVENTS_KERNEL
from tracetools_trace.tools.names import DEFAULT_CONTEXT

from ros2_benchmark import ImageResolution
from ros2_benchmark import ROS2BenchmarkConfig, ROS2BenchmarkTest

from launch.substitutions import LaunchConfiguration

IMAGE_RESOLUTION = ImageResolution.HD
# ROSBAG_PATH = '/tmp/benchmark_ws/src/rosbags/image'  # NOTE: hardcoded, modify accordingly
ROSBAG_PATH = '/home/martinho/acceleration/benchmark_ws/src/rosbags/perception/depth_image1'
SESSION_NAME = 'a4_depth_image_proc_auto_womon'
OPTION = 'without_monitor_node'

def launch_setup(container_prefix, container_sigterm_timeout):
    """Generate launch description for benchmarking image_proc RectifyNode."""

    data_loader_node = ComposableNode(
        name='DataLoaderNode',
        namespace=TestRectifyNode.generate_namespace(),
        package='ros2_benchmark',
        plugin='ros2_benchmark::DataLoaderNode',
        # remappings=[('hawk_0_left_rgb_image', 'data_loader/image'),
        #             ('hawk_0_left_rgb_camera_info', 'data_loader/camera_info')]
        remappings=[('camera/image_raw', 'data_loader/image'),
                    ('camera/camera_info', 'data_loader/camera_info'),
                    ('camera/depth/image_raw', 'data_loader/depth/image'),
                    ('camera/depth/camera_info', 'data_loader/depth/camera_info')]                   
    )

    playback_node = ComposableNode(
        name='PlaybackNode',
        namespace=TestRectifyNode.generate_namespace(),
        package='ros2_benchmark',
        plugin='ros2_benchmark::PlaybackNode',
        parameters=[{
            'data_formats': [
                'sensor_msgs/msg/Image',
                'sensor_msgs/msg/CameraInfo',
                'sensor_msgs/msg/Image',
                'sensor_msgs/msg/CameraInfo'],
        }],
        remappings=[('buffer/input0', 'data_loader/depth/image'),
                    ('input0', 'camera/depth/image_raw'),
                    ('buffer/input1', 'data_loader/depth/camera_info'),
                    ('input1', 'camera/depth/camera_info'),
                    ('buffer/input2', 'data_loader/image'),
                    ('input2', 'camera/image_raw'),
                    ('buffer/input3', 'data_loader/camera_info'),
                    ('input3', 'camera/camera_info')],                  
    )

    rectify_image_node = ComposableNode(
        package='image_proc',
        plugin='image_proc::RectifyNode',
        namespace='robotperf/preprocessing',
        name='rectify_color_node',
        remappings=[
            ('image', '/r2b/camera/image_raw'),
            ('camera_info', '/r2b/camera/camera_info'),
            ('image_rect', '/robotperf/preprocessing/rgb/image_rect_color')
        ],
        extra_arguments=[{'use_intra_process_comms': True}],
    )

    rectify_depth_node = ComposableNode(
        package='image_proc',
        plugin='image_proc::RectifyNode',
        namespace='robotperf/preprocessing',
        name='rectify_depth_node',
        remappings=[
            ('image', '/r2b/camera/depth/image_raw'),
            ('camera_info', '/r2b/camera/depth/camera_info'),
            ('image_rect', '/robotperf/preprocessing/depth_registered/image_rect')
        ],
        extra_arguments=[{'use_intra_process_comms': True}],
    )

    input_image_node = ComposableNode(
        package="a1_perception_2nodes",
        plugin="robotperf::perception::ImageInputComponent",
        namespace='robotperf',
        name="image_input_component",
        parameters=[
            {"input_topic_name":"/robotperf/input_rgb/image_rect_color"}
        ],
        remappings=[
            ("image", "/robotperf/preprocessing/rgb/image_rect_color"),
            ("camera_info", "/robotperf/preprocessing/camera/camera_info"),
        ],
        extra_arguments=[{'use_intra_process_comms': True}],
    )

    input_depth_node = ComposableNode(
        package="a1_perception_2nodes",
        plugin="robotperf::perception::ImageInputComponent",
        namespace='robotperf',
        name="image_input_component",
        parameters=[
            {"input_topic_name":"/robotperf/input_depth_registered/image_rect"}
        ],
        remappings=[
            ("image", "/robotperf/preprocessing/depth_registered/image_rect"),
            ("camera_info", "/robotperf/preprocessing/camera/depth/camera_info"),
        ],
        extra_arguments=[{'use_intra_process_comms': True}],
    )

    rgbd_to_pointcloud_node = ComposableNode(
        namespace="robotperf/benchmark",
        package="depth_image_proc",
        plugin="depth_image_proc::PointCloudXyzrgbNode",
        name="depth_image_to_pointcloud_node",
        remappings=[
            ('rgb/camera_info', '/robotperf/input_rgb/camera_info'),
            ('rgb/image_rect_color', '/robotperf/input_rgb/image_rect_color'),
            ('depth_registered/image_rect', '/robotperf/input_depth_registered/image_rect')
        ],                
        extra_arguments=[{'use_intra_process_comms': True}],
    )

    output_pointcloud_node = ComposableNode(
        package="a4_depth_image_proc",
        plugin="robotperf::perception::PointCloudOutputComponent",
        namespace='robotperf',
        name="point_cloud_output_component",
        parameters=[
            {"output_topic_name":"/robotperf/benchmark/points"}
        ],
        extra_arguments=[{'use_intra_process_comms': True}],
    )

    composable_node_container = ComposableNodeContainer(
        name='container',
        namespace=TestRectifyNode.generate_namespace(),
        package='rclcpp_components',
        executable='component_container_mt',
        prefix=container_prefix,
        sigterm_timeout=container_sigterm_timeout,
        composable_node_descriptions=[
            data_loader_node,
            playback_node,
            rectify_image_node,
            rectify_depth_node,
            input_image_node,
            input_depth_node,
            rgbd_to_pointcloud_node,
            output_pointcloud_node
        ],
        output='screen'
    )

    return [composable_node_container]


class TestRectifyNode(ROS2BenchmarkTest):
    """Performance test for image_proc RectifyNode and ResizeNode."""

    # Custom configurations
    config = ROS2BenchmarkConfig(
        benchmark_name='image_proc::RectifyNodeResizeNode Benchmark',
        input_data_path=ROSBAG_PATH,
        # Upper and lower bounds of peak throughput search window
        publisher_upper_frequency=30.0,
        publisher_lower_frequency=30.0,
        # The number of frames to be buffered
        playback_message_buffer_size=30,
        custom_report_info={'data_resolution': IMAGE_RESOLUTION},
        option = OPTION,
        session_name = SESSION_NAME
    )

    def test_benchmark(self):
        json_file_path = self.run_benchmark()
        
        if self.config.option == 'with_monitor_node':
            # Open the file and load the JSON content into a Python dictionary
            with open(json_file_path, 'r') as f:
                data = json.load(f)
            # Extract the desired fields
            mean_latency = data.get("BasicPerformanceMetrics.MEAN_LATENCY")
            max_latency = data.get("BasicPerformanceMetrics.MAX_LATENCY")
            min_latency = data.get("BasicPerformanceMetrics.MIN_LATENCY")
            rms_latency = data.get("BasicPerformanceMetrics.RMS_LATENCY")
            frames_sent = int(data.get("BasicPerformanceMetrics.NUM_FRAMES_SENT"))
            frames_missed = int(data.get("BasicPerformanceMetrics.NUM_MISSED_FRAMES"))               

            str_out =  "|     | Benchmark Mean | Benchmark RMS | Benchmark Max  | Benchmark Min | Lost Messages |\n"
            str_out += "| --- | -------------- | ------------- | -------------- | ------------- | --------------|\n"
            str_out += "| ros2_benchmark | **{:.2f}** ms | **{:.2f}** ms | **{:.2f}** ms | **{:.2f}** ms | {:.2f} % |\n".format(
                mean_latency, rms_latency, max_latency, min_latency, (frames_missed/frames_sent)*100)
            print(str_out)


def generate_test_description():
    qos_reliability_arg = DeclareLaunchArgument('qos_reliability', default_value='best_effort')
    ld = LaunchDescription()
    ld.add_action(qos_reliability_arg)
    return TestRectifyNode.generate_test_description_with_nsys(launch_setup)



