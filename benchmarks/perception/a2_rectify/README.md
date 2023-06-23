# a2_rectify

Perception rectify ROS Component.

### ID
a2

### Description
A simple perception rectify ROS robotics operation. Used to demonstrate a simple perception component [image_pipeline](https://github.com/ros-perception/image_pipeline) package.


![](../../../imgs/a2_rectify.svg)

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
colcon build --merge-install --packages-up-to a2_rectify

# Source the workspace as an overlay, launch the benchmark
source install/setup.bash
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ros2 launch a2_rectify trace_a2_rectify.launch.py

```

## Results

| Type | Hardware | Metric | Value | Category | Timestamp | Note | Data Source |
| --- | --- | --- | --- | --- | --- | --- | --- |
| :white_circle: | ROBOTCORE | latency | 66.82 | edge | 14-10-2022 |  | perception/image |

