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

# Based on ros2_control_demos: RRBot, Example 1
# https://github.com/ros-controls/ros2_control_demos/blob/master/example_1/bringup/launch/rrbot.launch.py
# https://github.com/ros-controls/ros2_control_demos/blob/master/example_1/bringup/launch/test_joint_trajectory_controller.launch.py

from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer

from tracetools_launch.action import Trace
from tracetools_trace.tools.names import DEFAULT_EVENTS_ROS
from tracetools_trace.tools.names import DEFAULT_EVENTS_KERNEL
from tracetools_trace.tools.names import DEFAULT_CONTEXT

def generate_launch_description():

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("ros2_control_demo_example_1"),
                    "urdf",
                    "rrbot.urdf.xacro",
                ]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("ros2_control_demo_example_1"),
            "config",
            "rrbot_controllers.yaml",
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_position_controller", "--controller-manager", "/controller_manager"],
    )

    # Test JointTrajectoryController
    position_goals = PathJoinSubstitution(
        [
            FindPackageShare("ros2_control_demo_example_1"),
            "config",
            "rrbot_joint_trajectory_publisher.yaml",
        ]
    )

    joint_trajectory_commands = Node(
        package="ros2_controllers_test_nodes",
        executable="publisher_joint_trajectory_controller",
        name="publisher_joint_trajectory_controller",
        parameters=[position_goals],
        output="both",
    )

    delay_joint_commands_after_robot_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[joint_trajectory_commands],
        )
    )

    trace = Trace(
        session_name="c1_rrbot_joint_trajectory_controller",
        events_ust=[
            "ros2:*",
            "robotcore_control:*",
            "robotcore_power:*",
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

    delay_trace_after_joint_trajectory_commands = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=joint_trajectory_commands,
            on_start=[trace]
        )
    )

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

    nodes = [
        control_node,
        robot_controller_spawner,
        delay_joint_commands_after_robot_controller_spawner,
        delay_trace_after_joint_trajectory_commands,
        power_container
    ]

    return LaunchDescription(nodes)
