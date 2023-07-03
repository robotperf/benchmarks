#
#    @@@@@@@@@@@@@@@@@@@@
#    @@@@@@@@@&@@@&&@@@@@
#    @@@@@ @@  @@    @@@@
#    @@@@@ @@  @@    @@@@
#    @@@@@ @@  @@    @@@@ Copyright (c) 2023, Acceleration Robotics®
#    @@@@@ @@  @@    @@@@ Author: Alejandra Martínez Fariña <alex@accelerationrobotics.com>
#    @@@@@ @@  @@    @@@@ Author: Víctor Mayoral Vilches <victor@accelerationrobotics.com>
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
import os
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

from ros2_benchmark import ImageResolution
from ros2_benchmark import ROS2BenchmarkConfig, ROS2BenchmarkTest

# These are provided as environment variables for CI, you can manually hardcord them for other uses
rosbag = os.environ.get('ROSBAG')
package = os.environ.get('PACKAGE')
type = os.environ.get('TYPE')
metric = os.environ.get('METRIC')

POWER_LIB = os.environ.get('POWER_LIB')
IMAGE_RESOLUTION = ImageResolution.HD
ROSBAG_PATH = '/tmp/benchmark_ws/src/rosbags/' + rosbag # '/home/amf/benchmark_ws/src/rosbags/perception/image3' # NOTE: hardcoded, modify accordingly
SESSION_NAME = package
if type == "grey":
    OPTION = 'without_monitor_node'
else:
    OPTION = 'with_monitor_node'
if metric == "power":
    POWER = "on" # by default "off"
else:
    POWER = "off"

def launch_setup(container_prefix, container_sigterm_timeout):
    """Generate launch description for benchmarking stereo_image_proc DisparityNode."""

    data_loader_node = ComposableNode(
        name='DataLoaderNode',
        namespace=TestDisparityNode.generate_namespace(),
        package='ros2_benchmark',
        plugin='ros2_benchmark::DataLoaderNode',
        remappings=[('left_camera/image_raw', 'data_loader/left_camera/image'),
                    ('left_camera/camera_info', 'data_loader/left_camera/camera_info'),
                    ('right_camera/image_raw', 'data_loader/right_camera/image'),
                    ('right_camera/camera_info', 'data_loader/right_camera/camera_info')]                    
    )

    playback_node = ComposableNode(
        name='PlaybackNode',
        namespace=TestDisparityNode.generate_namespace(),
        package='ros2_benchmark',
        plugin='ros2_benchmark::PlaybackNode',
        parameters=[{
            'data_formats': [
                'sensor_msgs/msg/Image',
                'sensor_msgs/msg/CameraInfo',
                'sensor_msgs/msg/Image',
                'sensor_msgs/msg/CameraInfo'],
        }],
        remappings=[('buffer/input0', 'data_loader/left_camera/image'),
                    ('input0', 'left_camera/image_raw'),
                    ('buffer/input1', 'data_loader/left_camera/camera_info'),
                    ('input1', 'left_camera/camera_info'),
                    ('buffer/input2', 'data_loader/right_camera/image'),
                    ('input2', 'right_camera/image_raw'),
                    ('buffer/input3', 'data_loader/right_camera/camera_info'),
                    ('input3', 'right_camera/camera_info')],                   
    )

    input_node_left = ComposableNode(
        package="a1_perception_2nodes",
        namespace="robotperf",
        plugin="robotperf::perception::ImageInputComponent",
        name="image_input_component",
        parameters=[
            {"input_topic_name":"/robotperf/input/left_input/left_image_raw"}
        ],
        remappings=[
            ("image", "/r2b/left_camera/image_raw"),
            ("camera_info", "/r2b/left_camera/camera_info"),
        ],
        extra_arguments=[{'use_intra_process_comms': True}],
    )

    input_node_right = ComposableNode(
        package="a1_perception_2nodes",
        namespace="robotperf",
        plugin="robotperf::perception::ImageInputComponent",
        name="image_input_component",
        parameters=[
            {"input_topic_name":"/robotperf/input/right_input/right_image_raw"}
        ],
        remappings=[
            ("image", "/r2b/right_camera/image_raw"),
            ("camera_info", "/r2b/right_camera/camera_info"),
        ],
        extra_arguments=[{'use_intra_process_comms': True}],
    )

    disparity_node = ComposableNode(
        namespace="robotperf/benchmark",
        package="stereo_image_proc",
        plugin="stereo_image_proc::DisparityNode",
        name="disparity_node",
        remappings=[
            ('left/camera_info', '/robotperf/input/left_input/camera_info'),
            ('left/image_rect', '/robotperf/input/left_input/left_image_raw'),
            ('right/camera_info', '/robotperf/input/right_input/camera_info'),
            ('right/image_rect', '/robotperf/input/right_input/right_image_raw'),                    
        ],
        extra_arguments=[{'use_intra_process_comms': True}],
    )

    output_node = ComposableNode(
        package="a3_stereo_image_proc",
        namespace="robotperf",
        plugin="robotperf::perception::DisparityOutputComponent",
        name="disparity_output_component",
        parameters=[
            {"output_topic_name":"/robotperf/benchmark/disparity"}
        ],
        extra_arguments=[{'use_intra_process_comms': True}],
    )

    monitor_node = ComposableNode(
        name='MonitorNode',
        namespace=TestDisparityNode.generate_namespace(),
        package='ros2_benchmark',
        plugin='ros2_benchmark::MonitorNode',
        parameters=[{
            'monitor_data_format': 'stereo_msgs/msg/DisparityImage',
            'monitor_power_data_format': 'power_msgs/msg/Power',
        }],
        remappings=[
            ('output', '/robotperf/benchmark/disparity')],
    )

    if OPTION == 'with_monitor_node':
        composable_node_descriptions_option=[
            data_loader_node,
            playback_node,
            input_node_left,
            input_node_right,
            disparity_node,
            output_node,
            monitor_node           
        ]
    else:
        composable_node_descriptions_option=[
            data_loader_node,
            playback_node,
            input_node_left,
            input_node_right,
            disparity_node,
            output_node,           
        ]

    composable_node_container = ComposableNodeContainer(
        name='container',
        namespace=TestDisparityNode.generate_namespace(),
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
                        {"power_lib": POWER_LIB}
                    ],
                ),
                
            ],
            output="screen",
        )
        return [composable_node_container, power_container]
    else:
        return [composable_node_container]


class TestDisparityNode(ROS2BenchmarkTest):
    """Performance test for stereo_image_proc DisparityNode."""

    # Custom configurations
    config = ROS2BenchmarkConfig(
        benchmark_name='stereo_image_proc::DisparityNode Benchmark',
        input_data_path=ROSBAG_PATH,
        # Upper and lower bounds of peak throughput search window
        publisher_upper_frequency=30.0,
        publisher_lower_frequency=30.0,
        # The number of frames to be buffered
        playback_message_buffer_size=50,
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
     
def generate_test_description():
    return TestDisparityNode.generate_test_description_with_nsys(launch_setup)
