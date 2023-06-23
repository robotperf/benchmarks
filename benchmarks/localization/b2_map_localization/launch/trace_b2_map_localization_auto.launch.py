#
#    @@@@@@@@@@@@@@@@@@@@
#    @@@@@@@@@&@@@&&@@@@@
#    @@@@@ @@  @@    @@@@
#    @@@@@ @@  @@    @@@@
#    @@@@@ @@  @@    @@@@ Copyright (c) 2023, Acceleration Robotics®
#    @@@@@ @@  @@    @@@@ Author: Alejandra Martínez Fariña <alex@accelerationrobotics.com>
#    @@@@@ @@  @@    @@@@ Author: Martiño Crespo Álvarez <martinho@accelerationrobotics.com>
#    @@@@@ @@  @@    @@@@ Author: Víctor Mayoral Vilches <victor@accelerationrobotics.com>
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

# Based on isaac_ros_benchmark: Performance test for Isaac ROS OccupancyGridLocalizerNode.
# https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_benchmark/blob/main/scripts/isaac_ros_grid_localizer_node.py

import json
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

from ros2_benchmark import ImageResolution
from ros2_benchmark import ROS2BenchmarkConfig, ROS2BenchmarkTest

IMAGE_RESOLUTION = ImageResolution.HD
ROSBAG_PATH = '/workspaces/isaac_ros-dev/src/ros2_benchmark/assets/datasets/r2b_dataset/r2b_storage' # NOTE: hardcoded, modify accordingly
MAP_YAML_PATH = '/workspaces/isaac_ros-dev/src/isaac_ros_benchmark/scripts/occupancy_grid_localizer/maps/map.yaml'
SESSION_NAME = 'b2_map_localization_auto'
OPTION = 'with_monitor_node'
POWER = "on" # by default "off"

def launch_setup(container_prefix, container_sigterm_timeout):
    """Generate launch description for benchmarking Isaac ROS OccupancyGridLocalizerNode."""

    occupancy_grid_localizer_node = ComposableNode(
        name='OccupancyGridLocalizerNode',
        namespace='robotperf/benchmark',
        package='isaac_ros_occupancy_grid_localizer',
        plugin='nvidia::isaac_ros::occupancy_grid_localizer::OccupancyGridLocalizerNode',
        parameters=[
            MAP_YAML_PATH,
            {
                'loc_result_frame': 'map',
                'map_yaml_path': MAP_YAML_PATH,
            }
        ],
        remappings=[
            ('flatscan_localization', '/r2b/flatscan_localization')],
    )
    
    data_loader_node = ComposableNode(
        name='DataLoaderNode',
        namespace=TestOccupancyGridLocalizerNode.generate_namespace(),
        package='ros2_benchmark',
        plugin='ros2_benchmark::DataLoaderNode',
        remappings=[('pandar_xt_32_0_lidar', 'data_loader/pointcloud')]
    )


    pointcloud_to_flatscan_node = ComposableNode(
        name='PointCloudToFlatScanNode',
        namespace=TestOccupancyGridLocalizerNode.generate_namespace(),
        package='isaac_ros_pointcloud_utils',
        plugin='nvidia::isaac_ros::pointcloud_utils::PointCloudToFlatScanNode',
        remappings=[('pointcloud', 'data_loader/pointcloud'),
                    ('flatscan', 'buffer/flatscan_localization')]
    )

    playback_node = ComposableNode(
        name='PlaybackNode',
        namespace=TestOccupancyGridLocalizerNode.generate_namespace(),
        package='isaac_ros_benchmark',
        plugin='isaac_ros_benchmark::NitrosPlaybackNode',
        parameters=[{
            'data_formats': ['nitros_flat_scan']
        }],
        remappings=[('buffer/input0', 'buffer/flatscan_localization'),
                    ('input0', 'flatscan_localization')],
    )

    monitor_node = ComposableNode(
        name='MonitorNode',
        namespace=TestOccupancyGridLocalizerNode.generate_namespace(),
        package='isaac_ros_benchmark',
        plugin='isaac_ros_benchmark::NitrosMonitorNode',
        parameters=[{
            'monitor_data_format': 'nitros_pose_cov_stamped',
            'monitor_power_data_format': 'power_msgs/msg/Power',
            'use_nitros_type_monitor_sub': True,
        }],
        remappings=[
            ('output', '/robotperf/benchmark/localization_result')],
    )

    if OPTION == 'with_monitor_node':
        composable_node_descriptions_option=[
            data_loader_node,
            playback_node,
            pointcloud_to_flatscan_node,
            monitor_node,
            occupancy_grid_localizer_node         
        ]
    else:
        composable_node_descriptions_option=[
            data_loader_node,
            playback_node,
            pointcloud_to_flatscan_node,
            occupancy_grid_localizer_node              
        ]

    composable_node_container = ComposableNodeContainer(
        name='container',
        namespace=TestOccupancyGridLocalizerNode.generate_namespace(),
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
                    ],
                ),
                
            ],
            output="screen",
        )
        return [composable_node_container, power_container]
    else:
        return [composable_node_container]


class TestOccupancyGridLocalizerNode(ROS2BenchmarkTest):
    """Performance test for OccupancyGridLocalizerNode."""

    # Custom configurations
    config = ROS2BenchmarkConfig(
        benchmark_name='Occupancy Grid Localizer Benchmark',
        input_data_path=ROSBAG_PATH,
        # Upper and lower bounds of peak throughput search window
        publisher_upper_frequency=50.0,
        publisher_lower_frequency=1.0,
        # The number of frames to be buffered
        playback_message_buffer_size=5,
        # Frequency, in hz, to increase the target frequency by with each step
        linear_scan_step_size=1.0,
        # Frame rate drop between input and output that can be tolerated without failing the test
        binary_search_acceptable_frame_rate_drop=1,
        # Adding offset to prevent using first pointcloud msg, which is partially accumulated
        # Input Data Start Time (s)
        input_data_start_time=1,
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
    return TestOccupancyGridLocalizerNode.generate_test_description_with_nsys(launch_setup)

