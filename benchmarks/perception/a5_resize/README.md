# a5_resize

Perception resize ROS Component.

### ID
a5

### Description
A simple perception resize ROS robotics operation. Used to demonstrate a simple perception component [image_pipeline](https://github.com/ros-perception/image_pipeline) package.


![](../../../imgs/a5_resize.svg)

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
colcon build --merge-install --packages-up-to a5_resize

# Source the workspace as an overlay, launch the benchmark
source install/setup.bash
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ros2 launch a5_resize trace_a5_resize.launch.py

```

## Results

| Type | Hardware | Metric | Value | Category | Timestamp | Note | Data Source |
| --- | --- | --- | --- | --- | --- | --- | --- |
| :white_circle: | Intel® Core™ i5-8250U CPU @ 1.60GHz × 8 | latency | 33.68 | workstation | 08-05-2023 |  | perception/image2 |

