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

# Based on isaac_ros_benchmark: Performance test for Isaac ROS IsaacROSVisualSlamNode
# https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_benchmark/blob/main/scripts/isaac_ros_visual_slam_node.py

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

from ros2_benchmark import BenchmarkMode, ROS2BenchmarkConfig, ROS2BenchmarkTest

import json

# NOTE: hardcoded, modify accordingly
ROSBAG_PATH = '/workspaces/isaac_ros-dev/src/rosbags/r2b_cafe'
SESSION_NAME = 'isaac_ros_visual_slam'
OPTION = 'with_monitor_node'
POWER = "on" # by default "off"

def launch_setup(container_prefix, container_sigterm_timeout):
    """Generate launch description for VSLAM node."""
    visual_slam_node = ComposableNode(
        name='VisualSlamNode',
        namespace=TestIsaacROSVisualSlamNode.generate_namespace(),
        package='isaac_ros_visual_slam',
        plugin='isaac_ros::visual_slam::VisualSlamNode',
        remappings=[('stereo_camera/left/image', 'image_left'),
                    ('stereo_camera/left/camera_info', 'camera_info_left'),
                    ('stereo_camera/right/image', 'image_right'),
                    ('stereo_camera/right/camera_info', 'camera_info_right')],
        parameters=[{
                    'enable_rectified_pose': True,
                    'denoise_input_images': False,
                    'rectified_images': True
                    }],
        extra_arguments=[
            {'use_intra_process_comms': False}])

    data_loader_node = ComposableNode(
        name='DataLoaderNode',
        namespace=TestIsaacROSVisualSlamNode.generate_namespace(),
        package='ros2_benchmark',
        plugin='ros2_benchmark::DataLoaderNode',
        remappings=[
            ('d455_1_left_ir_image', 'buffer/image_left'),
            ('d455_1_left_ir_camera_info', 'buffer/camera_info_left'),
            ('d455_1_right_ir_image', 'buffer/image_right'),
            ('d455_1_right_ir_camera_info', 'buffer/camera_info_right')]
    )

    playback_node = ComposableNode(
        name='PlaybackNode',
        namespace=TestIsaacROSVisualSlamNode.generate_namespace(),
        package='isaac_ros_benchmark',
        plugin='isaac_ros_benchmark::NitrosPlaybackNode',
        parameters=[{
            'data_formats': [
                'sensor_msgs/msg/Image',
                'sensor_msgs/msg/CameraInfo',
                'sensor_msgs/msg/Image',
                'sensor_msgs/msg/CameraInfo'],
        }],
        remappings=[('buffer/input0', 'buffer/image_left'),
                    ('input0', 'image_left'),
                    ('buffer/input1', 'buffer/camera_info_left'),
                    ('input1', 'camera_info_left'),
                    ('buffer/input2', 'buffer/image_right'),
                    ('input2', 'image_right'),
                    ('buffer/input3', 'buffer/camera_info_right'),
                    ('input3', 'camera_info_right')],
    )

    monitor_node = ComposableNode(
        name='MonitorNode',
        namespace=TestIsaacROSVisualSlamNode.generate_namespace(),
        package='isaac_ros_benchmark',
        plugin='isaac_ros_benchmark::NitrosMonitorNode',
        parameters=[{
            'monitor_data_format': 'nav_msgs/msg/Odometry',
            'monitor_power_data_format': 'power_msgs/msg/Power'
        }],
        remappings=[
            ('output', 'visual_slam/tracking/odometry')],
    )

    composable_node_container = ComposableNodeContainer(
        name='vslam_container',
        namespace=TestIsaacROSVisualSlamNode.generate_namespace(),
        package='rclcpp_components',
        executable='component_container_mt',
        prefix=container_prefix,
        sigterm_timeout=container_sigterm_timeout,
        composable_node_descriptions=[
            data_loader_node,
            playback_node,
            monitor_node,
            visual_slam_node
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
                        {"publish_rate": 40.0},
                        # {"hardware_device_type": "rapl"}
                        {"hardware_device_type": "jetson"}
                    ],
                ),
                
            ],
            output="screen",
        )
        return [composable_node_container, power_container]
    else:
        return [composable_node_container]


def generate_test_description():
    return TestIsaacROSVisualSlamNode.generate_test_description_with_nsys(launch_setup)


class TestIsaacROSVisualSlamNode(ROS2BenchmarkTest):
    """Performance test for the VisualSlam node."""

    # Custom configurations
    config = ROS2BenchmarkConfig(
        benchmark_name='Isaac ROS VisualSlamNode Benchmark',
        benchmark_mode=BenchmarkMode.SWEEPING,
        input_data_path=ROSBAG_PATH,
        # Upper and lower bounds of peak throughput search window
        publisher_upper_frequency=200.0,
        publisher_lower_frequency=200.0,
        # The number of frames to be buffered
        playback_message_buffer_size=100,
        # Fine tuned publisher frequency search parameters
        binary_search_acceptable_frame_rate_drop=15,
        linear_scan_acceptable_frame_rate_drop=10,
        start_recording_service_timeout_sec=60,
        start_recording_service_future_timeout_sec=65,
        start_monitoring_service_timeout_sec=60,
        default_service_future_timeout_sec=75,
        option = OPTION,
        session_name = SESSION_NAME,
        add_power = POWER
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


