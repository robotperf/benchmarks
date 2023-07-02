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
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | ROBOTCORE | latency | 66.82 | edge | 14-10-2022 |  | [perception/image](https://github.com/robotperf/rosbags/tree/main/perception/image) |
| [:black_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | NVIDIA AGX Orin Dev. Kit | throughput | 36.46251208914602 | edge | 2023-06-30 19:53:43 | mean_benchmark 30.204346636213202, rms_benchmark 30.273858539984825, max_benchmark 36.46251208914602, min_benchmark 26.931846242965786, lost messages 0.00 % | [perception/image](https://github.com/robotperf/rosbags/tree/main/perception/image) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-8700K | latency | 10.612124 | workstation | 2023-07-01 17:54:28 | mean_benchmark 9.59008332142857, rms_benchmark 9.818872594532918, max_benchmark 10.612124, min_benchmark 3.820048, lost messages 7.14 % | [perception/image](https://github.com/robotperf/rosbags/tree/main/perception/image) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-8700K | power | 6.592531204223633 | workstation | 2023-07-01 17:57:52 |  | [perception/image](https://github.com/robotperf/rosbags/tree/main/perception/image) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-8700K | throughput | 43.18326658783711 | workstation | 2023-07-01 18:01:23 | mean_benchmark 28.13578647678196, rms_benchmark 29.073051247606564, max_benchmark 43.18326658783711, min_benchmark 5.915050470316012, lost messages 7.14 % | [perception/image](https://github.com/robotperf/rosbags/tree/main/perception/image) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-8700K | latency | 19.59939 | workstation | 2023-07-01 18:04:50 | mean_benchmark 12.956591102941175, rms_benchmark 13.193771815681659, max_benchmark 19.59939, min_benchmark 5.019039, lost messages 0.00 % | [perception/image](https://github.com/robotperf/rosbags/tree/main/perception/image) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-8700K | power | 6.1853766441345215 | workstation | 2023-07-01 18:08:07 |  | [perception/image](https://github.com/robotperf/rosbags/tree/main/perception/image) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-8700K | throughput | 37.38958341909571 | workstation | 2023-07-01 18:11:18 | mean_benchmark 30.395764664758964, rms_benchmark 30.54505965326154, max_benchmark 37.38958341909571, min_benchmark 23.2270259454009, lost messages 0.00 % | [perception/image](https://github.com/robotperf/rosbags/tree/main/perception/image) |
| [:black_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-8700K | latency | 17.968975 | workstation | 2023-07-01 18:14:34 | mean_benchmark 12.712246926470588, rms_benchmark 12.86785854143323, max_benchmark 17.968975, min_benchmark 7.869791, lost messages 0.00 % | [perception/image](https://github.com/robotperf/rosbags/tree/main/perception/image) |
| [:black_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-8700K | power | 6.207215309143066 | workstation | 2023-07-01 18:17:46 |  | [perception/image](https://github.com/robotperf/rosbags/tree/main/perception/image) |
| [:black_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-8700K | throughput | 40.56219527143775 | workstation | 2023-07-01 18:20:15 | mean_benchmark 30.328278850221153, rms_benchmark 30.460221865736965, max_benchmark 40.56219527143775, min_benchmark 25.21200332384969, lost messages 0.00 % | [perception/image](https://github.com/robotperf/rosbags/tree/main/perception/image) |

