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

import os

import json
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

from ros2_benchmark import ImageResolution
from ros2_benchmark import BenchmarkMode, ROS2BenchmarkConfig, ROS2BenchmarkTest

IMAGE_RESOLUTION = ImageResolution.HD
SESSION_NAME = 'robotcore_stella_vslam_auto_stereo_wmon'
OPTION = 'with_monitor_node'
POWER = "on" # by default "off"

# NOTE: hardcoded, modify accordingly
ROSBAG_PATH = '/data/stella_vslam/stereo/r2b_cafe'
STELLA_VSLAM_FBOW_PATH = '/data/stella_vslam/common/orb_vocab.fbow'
STELLA_VSLAM_CAMERA_CONFIG_PATH = 'data/stella_vslam/stereo/r2b_cafe.yaml'


def launch_setup(container_prefix, container_sigterm_timeout):
    """Generate launch description for benchmarking image_proc RectifyNode."""

    data_loader_node = ComposableNode(
        name='DataLoaderNode',
        namespace=StellaVSLAMNode.generate_namespace(),
        package='ros2_benchmark',
        plugin='ros2_benchmark::DataLoaderNode',
        # remappings=[('camera/image_raw', 'data_loader/image')]
        remappings=[
            ('d455_1_left_ir_image', 'data_loader/image_left'),
            ('d455_1_right_ir_image', 'data_loader/image_right')]                
    )

    playback_node = ComposableNode(
        name='PlaybackNode',
        namespace=StellaVSLAMNode.generate_namespace(),
        package='ros2_benchmark',
        plugin='ros2_benchmark::PlaybackNode',
        parameters=[{
            'data_formats': [
                'sensor_msgs/msg/Image',
                'sensor_msgs/msg/Image']
        }],
        # remappings=[('buffer/input0', 'data_loader/image'),
        #             ('input0', '/benchmark/camera/image_raw')]  
        remappings=[('buffer/input0', '/r2b/data_loader/image_left'),
                    ('input0', '/r2b/camera/left/image_raw'),
                    ('buffer/input1', '/r2b/data_loader/image_right'),
                    ('input1', '/r2b/camera/right/image_raw')]     
    )

    image_input_component_node = ComposableNode(
        package="a1_perception_2nodes",
        plugin="robotperf::perception::ImageInputComponent",
        name="image_input_component",
        remappings=[
            ("image", "/camera/image_raw"),
            ("camera_info", "/camera/camera_info"),
            ("input", "input_image")
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    initial_pose_input_component_node = ComposableNode(
        package="benchmark_components",
        plugin="robotperf::perception::OdometryInputComponent",
        name="odometry_input_component",
        remappings=[
            ("odometry", "/initialpose")
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    pose_output_component_node = ComposableNode(
        package="benchmark_components",
        plugin="robotperf::perception::OdometryOutputComponent",
        name="odometry_output_component",
        remappings=[
            ("odometry", "/benchmark/run_slam/camera_pose")
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    components_container = ComposableNodeContainer(
        name="perception_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            image_input_component_node,
            initial_pose_input_component_node,
            pose_output_component_node
        ],
        output="screen",
    )

    monitor_node_1 = ComposableNode(
        name='MonitorNode',
        namespace=StellaVSLAMNode.generate_namespace(),
        package='ros2_benchmark',
        plugin='ros2_benchmark::MonitorNode',
        parameters=[{
            'monitor_data_format': 'nav_msgs/msg/Odometry',
            'monitor_power_data_format': 'power_msgs/msg/Power',
        }],
        remappings=[
            ('output', '/robotperf/benchmark/run_slam/camera_pose')],
    )

    monitor_node_2 = ComposableNode(
        name='MonitorNode',
        namespace=StellaVSLAMNode.generate_namespace(),
        package='ros2_benchmark',
        plugin='ros2_benchmark::MonitorNode',
        parameters=[{
            'monitor_data_format': 'sensor_msgs/msg/Image',
            'monitor_power_data_format': 'power_msgs/msg/Power',
        }],
        remappings=[
            ('output', '/robotperf/benchmark/tf')],
    )

    if OPTION == 'with_monitor_node':
        composable_node_descriptions_option=[
            data_loader_node,
            playback_node,
            # Don't launch components
            # components_container
            monitor_node_1
            # monitor_node_2           
        ]
    else:
        composable_node_descriptions_option=[
            data_loader_node,
            playback_node,
            # Don't launch components
            # components_container         
        ]

    composable_node_container = ComposableNodeContainer(
        name='container',
        namespace=StellaVSLAMNode.generate_namespace(),
        package='rclcpp_components',
        executable='component_container_mt',
        prefix=container_prefix,
        sigterm_timeout=container_sigterm_timeout,
        composable_node_descriptions=composable_node_descriptions_option,
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
                        {"hardware_device_type": "rapl"}
                        # {"hardware_device_type": "nvml"}
                    ],
                ),
                
            ],
            output="screen",
        )
        return [composable_node_container, power_container]
    else:
        return [composable_node_container]

def define_simple_nodes():
    slam_tracking_node = Node(
            namespace='/robotperf/benchmark',
            package='stella_vslam_ros',
            executable='run_slam',
            name='run_slam',
            arguments=[
                    '-v', STELLA_VSLAM_FBOW_PATH,
                    '-c', STELLA_VSLAM_CAMERA_CONFIG_PATH,
                    '--headless',
                    '--map-db-out', '/tmp/vslam/map.msg',
                    '-ros-args',
                    # '-p publish_tf:=false',
                    '--remap',
                    '/robotperf/benchmark/camera/right/image_raw:=/r2b/camera/right/image_raw',
                    '/robotperf/benchmark/camera/left/image_raw:=/r2b/camera/left/image_raw',
                    # '/tf:=/robotperf/benchmark/tf',
                    # 'camera/image_raw:=/input_image',
                    # '/initialpose:=/input_odometry'
            ]
        )

    return [slam_tracking_node]

class StellaVSLAMNode(ROS2BenchmarkTest):
    """Performance test for image_proc RectifyNode."""

    # Custom configurations
    config = ROS2BenchmarkConfig(
        benchmark_name='stella_vslam_ros::system Benchmark',
        input_data_path=ROSBAG_PATH,
        # Upper and lower bounds of peak throughput search window
        publisher_upper_frequency=10.0,
        publisher_lower_frequency=10.0,
        # The number of frames to be buffered
        playback_message_buffer_size=100,
        binary_search_acceptable_frame_rate_drop=15,
        linear_scan_acceptable_frame_rate_drop=10,
        start_recording_service_timeout_sec=60,
        start_recording_service_future_timeout_sec=65,
        start_monitoring_service_timeout_sec=60,
        default_service_future_timeout_sec=75,
        play_messages_service_future_timeout_sec=100,
        custom_report_info={'data_resolution': IMAGE_RESOLUTION},
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


def generate_test_description():
    simple_nodes = define_simple_nodes()
    return StellaVSLAMNode.generate_test_description_with_nsys(launch_setup, simple_nodes)
