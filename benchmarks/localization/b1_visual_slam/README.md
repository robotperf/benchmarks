# b1_visual_slam

Visual SLAM ROS Component.

### ID
b1

### Description
A SLAM localization based on visual input, and optionally other sensors such as IMUs. Used to demonstrate the visual SLAM components [isaac_ros_visual_slam](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_benchmark/blob/main/scripts/isaac_ros_visual_slam_node.py) and [`stella_vslam_ros`](https://github.com/stella-cv/stella_vslam_ros).

`isaac_ros_visual_slam`-based graph:
![](../../../imgs/b1_visual_slam.svg)

`stella_vslam_ros`-based graph:

## Reproduction Steps

```bash
# Install isaac_ros_benchmark: https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_benchmark#quickstart
# Install stella_vslam_ros: https://stella-cv.readthedocs.io/en/latest/index.html

# Create a ROS 2 overlay workspace
mkdir -p /tmp/benchmark_ws/src

# Clone the benchmark repository
cd /tmp/benchmark_ws/src && git clone https://github.com/robotperf/benchmarks

# Fetch dependencies
source /opt/ros/humble/setup.bash
cd /tmp/benchmark_ws && sudo rosdep update || true && sudo apt-get update &&
  sudo rosdep install --from-paths src --ignore-src --rosdistro humble -y

# Build the benchmark
colcon build --merge-install --packages-up-to a5_resize

# Source the workspace as an overlay, launch the benchmark
source install/setup.bash
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ros2 launch a5_resize trace_a5_resize.launch.py

```

## Results

Data not valid at the moment, just a placeholder for now

| Type | Hardware | Metric | Value | Category | Timestamp | Note | Data Source |
| --- | --- | --- | --- | --- | --- | --- | --- |
| grey | Intel® Core™ i5-8250U CPU @ 1.60GHz × 8 | latency | 33.68 | workstation | 08-05-2023 |  | perception/image2 |

