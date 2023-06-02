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
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import LaunchConfigurationEquals
from ros2_benchmark import ImageResolution
from ros2_benchmark import ROS2BenchmarkConfig, ROS2BenchmarkTest
from tracetools_launch.action import Trace
from tracetools_trace.tools.names import DEFAULT_EVENTS_ROS
from tracetools_trace.tools.names import DEFAULT_EVENTS_KERNEL
from tracetools_trace.tools.names import DEFAULT_CONTEXT

import sys
import argparse

BENCHMARK_NAME = None
IMAGE_RESOLUTION = ImageResolution.HD
ROSBAG_PATH = None
SESSION_NAME = None
OPTION = None

def main(argv):
    print('It entered the main')
    # Parse the command-line arguments
    parser = argparse.ArgumentParser()
    parser.add_argument('--benchmark_name', type=str, help='Name for the benchmark', default ='benchmark')
    parser.add_argument('--hardware_device_type', type=str, help='Hardware Device Type (e.g. cpu or fpga)', default ='cpu')
    parser.add_argument('--trace_file_name', type=str, help='Name for the benchmark tracing files. It will be needed in the analyze file to provide the path.', default = 'session_name')
    parser.add_argument('--data_source', type=str, help='Type of data source (auto or manual). Use auto for automatically launching a ros2 bag with ros2_benchmark. Use manual for manually launching a ros2 bag or for real time robots.', default = ['auto'])
    parser.add_argument('--monitor_node', type=bool, help='Only for auto data source. Include the monitor_node or not.', default = True)
    parser.add_argument('--ros_bag_path', type=bool, help='Only for auto data source. Path to the ros bag file.', default = '/home/amf/benchmark_ws/src/rosbags/perception/image')
    args = parser.parse_args(argv)

    # Get the values of the arguments
    BENCHMARK_NAME = args.benchmark_name
    hardware_device_type = args.hardware_device_type
    SESSION_NAME = args.trace_file_name #'a5_resize_auto_wmon'
    data_source = args.data_source
    monitor_node = args.monitor_node
    ROSBAG_PATH = args.ros_bag_path #'/home/amf/benchmark_ws/src/rosbags/perception/image' #  NOTE: hardcoded, modify accordingly
    
    if monitor_node:
        OPTION = 'with_monitor_node'
    else:
        OPTION = 'without_monitor_node'

    if data_source == 'auto':
        TestResizeNode.generate_test_description_with_nsys(launch_auto_setup)
    
    
def launch_manual_setup():
    print('It entered the launch manual setup')
    # Trace
    trace = Trace(
        session_name="a5_resize",
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
        # events_kernel=DEFAULT_EVENTS_KERNEL,
        # context_names=DEFAULT_CONTEXT,
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
                remappings=[
                    ("image", "/camera/image_raw"),
                    ("camera_info", "/camera/camera_info"),
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                namespace="benchmark",
                package="image_proc",
                plugin="image_proc::ResizeNode",
                name="resize_node",
                remappings=[
                    ("camera_info", "/camera/camera_info"),
                    ("image", "/input"),
                    ("resize", "resize"),
                ],
                parameters=[
                    {
                        "scale_height": 2.0,
                        "scale_width": 2.0,
                    }
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package="a1_perception_2nodes",
                plugin="robotperf::perception::ImageOutputComponent",
                name="image_output_component",
                remappings=[
                    ("image", "/benchmark/resize"),
                    ("camera_info", "/camera/camera_info"),
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
        output="screen",
    )
    return [trace, perception_container]

def launch_auto_setup(container_prefix='', container_sigterm_timeout=5):
    print('It entered the launch auto setup')
    """Generate launch description for benchmarking image_proc RectifyNode."""

    # Declare the launch arguments
    hardware_device_type_arg = DeclareLaunchArgument(
        'hardware_device_type',
        default_value='cpu',
        description='Hardware Device Type (e.g. cpu or fpga)'
    )

    trace_file_name_arg = DeclareLaunchArgument(
        'trace_file_name',
        default_value='a5_resize',
        description='Name for the benchmark tracing files. It will be needed in the analyze file to provide the path.'
    )

    data_source_arg = DeclareLaunchArgument(
        'data_source',
        default_value='true',
        description='Type of data source (auto or manual). Use auto for automatically launching a ros2 bag with ros2_benchmark. Use manual for manually launching a ros2 bag or for real time robots.'
    )

    monitor_node_arg = DeclareLaunchArgument(
        'monitor_node',
        default_value= 'true',
        description='Only for auto data source. Include the monitor_node or not.'
    )

    ros_bag_path_arg = DeclareLaunchArgument(
        'ros_bag_path',
        default_value='/home/amf/benchmark_ws/src/rosbags/perception/image',
        description='Only for auto data source. Path to the ros bag file.'
    )

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

    composable_node_container = ComposableNodeContainer(
        name='container',
        namespace=TestResizeNode.generate_namespace(),
        package='rclcpp_components',
        executable='component_container_mt',
        prefix=container_prefix,
        sigterm_timeout=container_sigterm_timeout,
        composable_node_descriptions=[
            data_loader_node,
            # prep_resize_node,
            playback_node,
            input_node,
            resize_node,
            output_node,
            monitor_node            
        ],
        output='screen'
    )

    return [composable_node_container, hardware_device_type_arg, trace_file_name_arg, data_source_arg, monitor_node_arg, ros_bag_path_arg]


class TestResizeNode(ROS2BenchmarkTest):
    """Performance test for image_proc RectifyNode."""

    # Custom configurations
    config = ROS2BenchmarkConfig(
        benchmark_name= BENCHMARK_NAME,
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
    print('It entered the generate test description')  
    return TestResizeNode.generate_test_description_with_nsys(launch_auto_setup)

def generate_launch_description():
    print('It entered the generate launch description')
    # Declare the launch arguments
    hardware_device_type_arg = DeclareLaunchArgument(
        'hardware_device_type',
        default_value='cpu',
        description='Hardware Device Type (e.g. cpu or fpga)'
    )

    trace_file_name_arg = DeclareLaunchArgument(
        'trace_file_name',
        default_value='a5_resize',
        description='Name for the benchmark tracing files. It will be needed in the analyze file to provide the path.'
    )

    data_source_arg = DeclareLaunchArgument(
        'data_source',
        default_value='true',
        description='Type of data source (auto or manual). Use auto for automatically launching a ros2 bag with ros2_benchmark. Use manual for manually launching a ros2 bag or for real time robots.'
    )

    monitor_node_arg = DeclareLaunchArgument(
        'monitor_node',
        default_value= 'true',
        description='Only for auto data source. Include the monitor_node or not.'
    )

    ros_bag_path_arg = DeclareLaunchArgument(
        'ros_bag_path',
        default_value='/home/amf/benchmark_ws/src/rosbags/perception/image',
        description='Only for auto data source. Path to the ros bag file.'
    )

    # Create the launch description
    ld = LaunchDescription()

    # Add the declared launch arguments to the launch description
    ld.add_action(hardware_device_type_arg)
    ld.add_action(trace_file_name_arg)
    ld.add_action(data_source_arg)
    ld.add_action(monitor_node_arg)
    ld.add_action(ros_bag_path_arg)

    #print(PythonExpression(data_source_arg, '== true'))
    #print(LaunchConfiguration('data_source') == 'false')

    if LaunchConfiguration('data_source') == 'manual':
        # Define the nodes for the 'manual' data source ============================
        list_manual_nodes = launch_manual_setup()
        for node in list_manual_nodes:
            ld.add_action(node)
    #elif LaunchConfiguration('data_source') == 'auto':
    #    list_auto_nodes = generate_test_description()
    #    ld.add_action(list_auto_nodes)

    
    # end 'manual' ============================================================

    # Define the nodes for the 'auto' data source ========================
    # Define the ExecuteProcess action to run the Python script
    
    launch_test_action = ExecuteProcess(
        cmd=['launch_test', "src/benchmarks/benchmarks/perception/a5_resize/launch/trace_a5_resize_amf.launch.py",
            'hardware_device_type:=',LaunchConfiguration('hardware_device_type'),
            'trace_file_name:=',LaunchConfiguration('trace_file_name'),
            'data_source:=',LaunchConfiguration('data_source'),
            'monitor_node:=',LaunchConfiguration('monitor_node'),
            'ros_bag_path:=',LaunchConfiguration('ros_bag_path')],
        output='screen'
    )
    # end 'auto' ========================================
    
    # Add the ExecuteProcess action to the launch description
    ld.add_action(launch_test_action)
    
    return ld

if __name__ == '__main__':
    main(sys.argv[1:])
    #generate_test_description()