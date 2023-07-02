# c3_rrbot_forward_command_controller

Forward command controller

### ID
c3

### Description
A [ros2_control_demos](https://github.com/ros-controls/ros2_control_demos) used to demonstrate the [ros2_controllers](https://github.com/ros-controls/ros2_controllers) forward_command_controller


![](../../../imgs/c3_rrbot_forward_command_controller.svg)

## Reproduction Steps

```bash
# Create a ROS 2 overlay workspace
mkdir -p /tmp/benchmark_ws/src

# Clone the benchmark repository
cd /tmp/benchmark_ws/src && git clone https://github.com/robotperf/benchmarks

# Fetch dependencies
source /opt/ros/humble/setup.bash
cd /tmp/benchmark_ws && sudo rosdep update || true && sudo apt-get update &&
  sudo rosdep install --from-paths src --ignore-src --rosdistro humble -y

# Build the benchmark
colcon build --merge-install --packages-up-to c3_rrbot_forward_command_controller

# Source the workspace as an overlay
source install/setup.bash

# Select what type of controller (position, velocity or acceleration) to launch before launching the trace file
export C3_CONTROLLER_TYPE=position
ros2 launch c3_rrbot_forward_command_controller trace_c3_rrbot_forward_command_controller_power.launch.py

# Analyze trace files
cd path/to/benchmark_ws
ros2 launch c3_rrbot_forward_command_controller analyze_c3_rrbot_forward_command_controller.launch.py trace_path:=/root/.ros/tracing/c3_rrbot_forward_command_controller metrics:=[latency,power]

```

## Results

| Type | Hardware | Metric | Value | Category | Timestamp | Note | Data Source |
| --- | --- | --- | --- | --- | --- | --- | --- |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | ROBOTCORE | latency | 66.82 | edge | 14-10-2022 |  | [perception/image](https://github.com/robotperf/rosbags/tree/main/perception/image) |

