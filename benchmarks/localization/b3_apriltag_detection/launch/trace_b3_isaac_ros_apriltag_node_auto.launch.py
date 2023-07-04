#
#    @@@@@@@@@@@@@@@@@@@@
#    @@@@@@@@@&@@@&&@@@@@
#    @@@@@ @@  @@    @@@@
#    @@@@@ @@  @@    @@@@
#    @@@@@ @@  @@    @@@@ Copyright (c) 2023, Acceleration Robotics®
#    @@@@@ @@  @@    @@@@ Author: Víctor Mayoral Vilches <victor@accelerationrobotics.com>
#    @@@@@ @@  @@    @@@@ Author: Alejandra Martínez Fariña <alex@accelerationrobotics.com>
#    @@@@@ @@  @@    @@@@ Author: Martiño Crespo Álvarez <martinho@accelerationrobotics.com>
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

# Based on isaac_ros_benchmark: Performance test for Isaac ROS IsaacROSAprilTagNode
# https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_benchmark/blob/main/scripts/isaac_ros_apriltag_node.py

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

from ros2_benchmark import ImageResolution
from ros2_benchmark import ROS2BenchmarkConfig, ROS2BenchmarkTest

import os
import json

# # These are provided as environment variables for CI, you can manually hardcord them for other uses
# rosbag = os.environ.get('ROSBAG')
# package = os.environ.get('PACKAGE')
# type = os.environ.get('TYPE')
# metric = os.environ.get('METRIC')

# POWER_LIB = os.environ.get('POWER_LIB')
# IMAGE_RESOLUTION = ImageResolution.HD
# ROSBAG_PATH = '/tmp/benchmark_ws/src/rosbags/' + rosbag # '/home/amf/benchmark_ws/src/rosbags/perception/image' # NOTE: hardcoded, modify accordingly
# SESSION_NAME = package
# if type == "grey":
#     OPTION = 'without_monitor_node'
# else:
#     OPTION = 'with_monitor_node'
# if metric == "power":
#     POWER = "on" # by default "off"
# else:
#     POWER = "off"

# NOTE: hardcoded, modify accordingly
IMAGE_RESOLUTION = ImageResolution.HD
ROSBAG_PATH = '/workspaces/isaac_ros-dev/src/ros2_benchmark/assets/datasets/r2b_dataset/r2b_storage' # NOTE: hardcoded, modify accordingly
SESSION_NAME = 'isaac_ros_apriltag'
OPTION = 'with_monitor_node'
POWER = "on" # by default "off"
POWER_LIB = "jetson"

def launch_setup(container_prefix, container_sigterm_timeout):
    """Generate launch description for benchmarking Isaac ROS AprilTagNode."""
    apriltag_node = ComposableNode(
        name='AprilTagNode',
        namespace='/robotperf/benchmark',
        package='isaac_ros_apriltag',
        plugin='nvidia::isaac_ros::apriltag::AprilTagNode',
        remappings=[('tag_detections', 'apriltag_detections'),
                    ('camera_info', '/r2b/camera_info'),
                    ('image', '/r2b/image'),
                    ('image/nitros', '/r2b/image/nitros'),
                    ('camera_info/nitros', '/r2b/camera_info/nitros')
        ]
    )

    data_loader_node = ComposableNode(
        name='DataLoaderNode',
        namespace=RobotPerfAprilTagNode.generate_namespace(),
        package='ros2_benchmark',
        plugin='ros2_benchmark::DataLoaderNode',
        remappings=[('hawk_0_left_rgb_image', 'data_loader/image'),
                    ('hawk_0_left_rgb_camera_info', 'data_loader/camera_info')]
    )

    prep_resize_node = ComposableNode(
        name='PrepResizeNode',
        namespace=RobotPerfAprilTagNode.generate_namespace(),
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::ResizeNode',
        parameters=[{
            'output_width': IMAGE_RESOLUTION['width'],
            'output_height': IMAGE_RESOLUTION['height'],
        }],
        remappings=[('image', 'data_loader/image'),
                    ('camera_info', 'data_loader/camera_info'),
                    ('resize/image', 'buffer/image'),
                    ('resize/camera_info', 'buffer/camera_info')],
    )

    playback_node = ComposableNode(
        name='PlaybackNode',
        namespace=RobotPerfAprilTagNode.generate_namespace(),
        package='isaac_ros_benchmark',
        plugin='isaac_ros_benchmark::NitrosPlaybackNode',
        parameters=[{
            'data_formats': [
                'nitros_image_bgr8',
                'nitros_camera_info'
            ],
        }],
        remappings=[('buffer/input0', 'buffer/image'),
                    ('input0', 'image'),
                    ('buffer/input1', 'buffer/camera_info'),
                    ('input1', 'camera_info')],
    )

    monitor_node = ComposableNode(
        name='MonitorNode',
        namespace=RobotPerfAprilTagNode.generate_namespace(),
        package='isaac_ros_benchmark',
        plugin='isaac_ros_benchmark::NitrosMonitorNode',
        parameters=[{
            'monitor_data_format': 'nitros_april_tag_detection_array',
            'monitor_power_data_format': 'power_msgs/msg/Power',
            'use_nitros_type_monitor_sub': True,
        }],
        remappings=[('output', '/robotperf/benchmark/apriltag_detections')]
    )

    composable_node_container = ComposableNodeContainer(
        name='container',
        namespace=RobotPerfAprilTagNode.generate_namespace(),
        package='rclcpp_components',
        executable='component_container_mt',
        prefix=container_prefix,
        sigterm_timeout=container_sigterm_timeout,
        composable_node_descriptions=[
            data_loader_node,
            playback_node,
            monitor_node,
            apriltag_node,
            prep_resize_node,
        ],
        output='screen'
    )

    if POWER == "on":
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
        return [composable_node_container, power_container]
    else:
        return [composable_node_container]

def generate_test_description():
    return RobotPerfAprilTagNode.generate_test_description_with_nsys(launch_setup)


class RobotPerfAprilTagNode(ROS2BenchmarkTest):
    """Performance test for Isaac ROS AprilTagNode."""

    # Custom configurations
    config = ROS2BenchmarkConfig(
        benchmark_name='Isaac ROS AprilTagNode Benchmark',
        input_data_path=ROSBAG_PATH,
        # The slice of the rosbag to use
        input_data_start_time=3.0,
        input_data_end_time=3.5,
        # Upper and lower bounds of peak throughput search window
        publisher_upper_frequency=10.0,
        publisher_lower_frequency=10.0,
        # The number of frames to be buffered
        playback_message_buffer_size=10,
        custom_report_info={'data_resolution': IMAGE_RESOLUTION},
        option = OPTION,
        session_name = SESSION_NAME,
        add_power = POWER
    )

    def test_benchmark(self):
        json_file_path = self.run_benchmark()

        # Copy the JSON file to the "/tmp/json" file
        # NOTE: this will be then used by the CI to post-process and analyze results
        os.system("cp " + json_file_path + " /tmp/json")

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
            if self.config.add_power == "on":
                average_power = data.get("BasicPerformanceMetrics.AVERAGE_POWER")   
                str_out =  "|     | Benchmark Mean | Benchmark RMS | Benchmark Max  | Benchmark Min | Lost Messages | Average Power |\n"
                str_out += "| --- | -------------- | ------------- | -------------- | ------------- | --------------| ------------------|\n"
                str_out += "| ros2_benchmark | **{:.2f}** ms | **{:.2f}** ms | **{:.2f}** ms | **{:.2f}** ms | {:.2f} % | **{:.2f}** W |\n".format(
                mean_latency, rms_latency, max_latency, min_latency, (frames_missed/frames_sent)*100, average_power)         
            else:
                str_out =  "|     | Benchmark Mean | Benchmark RMS | Benchmark Max  | Benchmark Min | Lost Messages |\n"
                str_out += "| --- | -------------- | ------------- | -------------- | ------------- | --------------|\n"
                str_out += "| ros2_benchmark | **{:.2f}** ms | **{:.2f}** ms | **{:.2f}** ms | **{:.2f}** ms | {:.2f} % |\n".format(
                    mean_latency, rms_latency, max_latency, min_latency, (frames_missed/frames_sent)*100)
            print(str_out)