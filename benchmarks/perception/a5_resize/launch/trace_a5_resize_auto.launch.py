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
from launch_ros.descriptions import ComposableNode

from ros2_benchmark import ImageResolution
from ros2_benchmark import ROS2BenchmarkConfig, ROS2BenchmarkTest

IMAGE_RESOLUTION = ImageResolution.HD
ROSBAG_PATH = '/home/amf/benchmark_ws/src/rosbags/perception/image' #  NOTE: hardcoded, modify accordingly
SESSION_NAME = 'a5_resize_auto_wmon'
OPTION = 'without_monitor_node'

def launch_setup(container_prefix, container_sigterm_timeout):
    """Generate launch description for benchmarking image_proc RectifyNode."""

    data_loader_node = ComposableNode(
        name='DataLoaderNode',
        namespace=TestResizeNode.generate_namespace(),
        package='ros2_benchmark',
        plugin='ros2_benchmark::DataLoaderNode',
        remappings=[('camera/image_raw', 'data_loader/image'),
                    ('camera/camera_info', 'data_loader/camera_info')]                    
    )

    playback_node = ComposableNode(
        name='PlaybackNode',
        namespace=TestResizeNode.generate_namespace(),
        package='ros2_benchmark',
        plugin='ros2_benchmark::PlaybackNode',
        parameters=[{
            'data_formats': [
                'sensor_msgs/msg/Image',
                'sensor_msgs/msg/CameraInfo'],
        }],
        remappings=[('buffer/input0', 'data_loader/image'),
                    ('input0', 'image_raw'),
                    ('buffer/input1', 'data_loader/camera_info'),
                    ('input1', 'camera_info')],                  
    )

    input_node = ComposableNode(
        package="a1_perception_2nodes",
        namespace="robotperf",
        plugin="robotperf::perception::ImageInputComponent",
        name="image_input_component",
        remappings=[
            ("image", "/r2b/image_raw"),
            ("camera_info", "/r2b/camera_info"),
        ],
        extra_arguments=[{'use_intra_process_comms': True}],
    )


    resize_node = ComposableNode(
        namespace="robotperf/benchmark",
        package="image_proc",
        plugin="image_proc::ResizeNode",
        name="resize_node",
        remappings=[
            ("camera_info", "/r2b/camera_info"),
            ("image", "/robotperf/input"),
            ("resize", "/robotperf/benchmark/resize"),
        ],
        parameters=[
            {
                "scale_height": 2.0,
                "scale_width": 2.0,
            }
        ],
        extra_arguments=[{'use_intra_process_comms': True}],
    )

    output_node = ComposableNode(
        package="a1_perception_2nodes",
        namespace="robotperf",
        plugin="robotperf::perception::ImageOutputComponent",
        name="image_output_component",
        remappings=[
            ("image", "/robotperf/benchmark/resize"),
            ("camera_info", "/r2b/camera_info"),
        ],
        extra_arguments=[{'use_intra_process_comms': True}],
    )

    monitor_node = ComposableNode(
        name='MonitorNode',
        namespace=TestResizeNode.generate_namespace(),
        package='ros2_benchmark',
        plugin='ros2_benchmark::MonitorNode',
        parameters=[{
            'monitor_data_format': 'sensor_msgs/msg/Image',
        }],
        remappings=[
            ('output', '/robotperf/benchmark/resize')],
    )

    if OPTION == 'with_monitor_node':
        composable_node_descriptions_option=[
            data_loader_node,
            # prep_resize_node,
            playback_node,
            input_node,
            resize_node,
            output_node,
            monitor_node            
        ]
    else:
        composable_node_descriptions_option=[
            data_loader_node,
            # prep_resize_node,
            playback_node,
            input_node,
            resize_node,
            output_node,            
        ]

    composable_node_container = ComposableNodeContainer(
        name='container',
        namespace=TestResizeNode.generate_namespace(),
        package='rclcpp_components',
        executable='component_container_mt',
        prefix=container_prefix,
        sigterm_timeout=container_sigterm_timeout,
        composable_node_descriptions=composable_node_descriptions_option,
        output='screen'
    )

    return [composable_node_container]


class TestResizeNode(ROS2BenchmarkTest):
    """Performance test for image_proc RectifyNode."""

    # Custom configurations
    config = ROS2BenchmarkConfig(
        benchmark_name='image_proc::RectifyNode Benchmark',
        input_data_path=ROSBAG_PATH,
        # Upper and lower bounds of peak throughput search window
        publisher_upper_frequency=30.0,
        publisher_lower_frequency=30.0,
        # The number of frames to be buffered
        playback_message_buffer_size=68,
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
    return TestResizeNode.generate_test_description_with_nsys(launch_setup)

