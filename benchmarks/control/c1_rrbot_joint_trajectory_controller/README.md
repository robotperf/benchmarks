# c1_rrbot_joint_trajectory_controller

Joint trajectory controller

### ID
c1

### Description
A [ros2_control_demos](https://github.com/ros-controls/ros2_control_demos) used to demonstrate the [ros2_controllers](https://github.com/ros-controls/ros2_controllers) joint_trajectory_controller


![](../../../imgs/c1_rrbot_joint_trajectory_controller.svg)

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
colcon build --merge-install --packages-up-to c1_rrbot_joint_trajectory_controller

# Source the workspace as an overlay, launch the benchmark
source install/setup.bash
ros2 launch c1_rrbot_joint_trajectory_controller trace_c1_rrbot_joint_trajectory_power.launch.py

# Analyze trace files
cd path/to/benchmark_ws
ros2 launch c1_rrbot_joint_trajectory_controller analyze_c1_rrbot_joint_trajectory_controller.launch.py trace_path:=/root/.ros/tracing/c1_rrbot_joint_trajectory_controller metrics:=[latency,power]

```

## Results

| Type | Hardware | Metric | Value | Category | Timestamp | Note | Data Source |
| --- | --- | --- | --- | --- | --- | --- | --- |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | ROBOTCORE | latency | 66.82 | edge | 14-10-2022 |  | [perception/image](https://github.com/robotperf/rosbags/tree/main/perception/image) |

