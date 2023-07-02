# a1_perception_2nodes

Perception computational graph composed by 2 dataflow-connected *Components*, `rectify` and `resize`.

### ID
a1

### Description
A simple perception computational graph composed by 2 Components, `rectify` and `resize` operations. Used to demonstrate a simple perception pipeline using the [image_pipeline](https://github.com/ros-perception/image_pipeline) package.


![](../../../imgs/a1_perception_2nodes.svg)

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
colcon build --merge-install --packages-up-to a1_perception_2nodes

# Source the workspace as an overlay, launch the benchmark
source install/setup.bash
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ros2 launch a1_perception_2nodes trace_a1_perception_2nodes.launch.py

```

## Results

| Type | Hardware | Metric | Value | Category | Timestamp | Note | Data Source |
| --- | --- | --- | --- | --- | --- | --- | --- |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | ROBOTCORE | latency | 66.82 | edge | 14-10-2022 |  | [perception/image](https://github.com/robotperf/rosbags/tree/main/perception/image) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Kria KR260 | latency | 66.82 | edge | 14-10-2022 |  | [perception/image](https://github.com/robotperf/rosbags/tree/main/perception/image) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Jetson Nano | latency | 238.13 | edge | 14-10-2022 |  | [perception/image](https://github.com/robotperf/rosbags/tree/main/perception/image) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Jetson AGX Xavier | latency | 106.34 | edge | 14-10-2022 |  | [perception/image](https://github.com/robotperf/rosbags/tree/main/perception/image) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-8700K | latency | 27.86298 | workstation | 2023-06-25 15:10:31 | mean_benchmark 13.880188545454546, rms_benchmark 14.387412826011047, max_benchmark 27.86298, min_benchmark 9.530861999999999, lost messages 4.55 % | [perception/image](https://github.com/robotperf/rosbags/tree/main/perception/image) |
| [:black_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-8700K | latency | 46.155170000000005 | workstation | 2023-06-30 21:13:04 | mean_benchmark 13.729018779411762, rms_benchmark 14.764683243129985, max_benchmark 46.155170000000005, min_benchmark 6.645244999999999, lost messages 0.00 % | [perception/image](https://github.com/robotperf/rosbags/tree/main/perception/image) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-8700K | latency | 16.729871 | workstation | 2023-07-01 13:33:05 | mean_benchmark 13.63853788, rms_benchmark 13.767741008697662, max_benchmark 16.729871, min_benchmark 10.103081999999999, lost messages 8.00 % | [perception/image](https://github.com/robotperf/rosbags/tree/main/perception/image) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-8700K | power | 7.1712188720703125 | workstation | 2023-07-01 13:36:18 |  | [perception/image](https://github.com/robotperf/rosbags/tree/main/perception/image) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-8700K | latency | 40.397632 | workstation | 2023-07-01 17:24:11 | mean_benchmark 14.55271958490566, rms_benchmark 15.300233260168039, max_benchmark 40.397632, min_benchmark 8.393206000000001, lost messages 0.00 % | [perception/image](https://github.com/robotperf/rosbags/tree/main/perception/image) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-8700K | power | 7.394293785095215 | workstation | 2023-07-01 17:27:21 |  | [perception/image](https://github.com/robotperf/rosbags/tree/main/perception/image) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-8700K | throughput | 45.27460770005173 | workstation | 2023-07-01 17:30:50 | mean_benchmark 29.25599254168803, rms_benchmark 29.710040331431454, max_benchmark 45.27460770005173, min_benchmark 9.43612660892683, lost messages 0.00 % | [perception/image](https://github.com/robotperf/rosbags/tree/main/perception/image) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-8700K | latency | 41.113 | workstation | 2023-07-01 17:34:06 | mean_benchmark 14.362254691176473, rms_benchmark 15.069385102593149, max_benchmark 41.113, min_benchmark 7.722244, lost messages 0.00 % | [perception/image](https://github.com/robotperf/rosbags/tree/main/perception/image) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-8700K | power | 8.909062385559082 | workstation | 2023-07-01 17:37:37 |  | [perception/image](https://github.com/robotperf/rosbags/tree/main/perception/image) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-8700K | throughput | 40.78378772658402 | workstation | 2023-07-01 17:40:49 | mean_benchmark 30.245347842949002, rms_benchmark 30.287857695872127, max_benchmark 40.78378772658402, min_benchmark 27.675432490902267, lost messages 0.00 % | [perception/image](https://github.com/robotperf/rosbags/tree/main/perception/image) |
| [:black_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-8700K | latency | 18.400618 | workstation | 2023-07-01 17:44:20 | mean_benchmark 13.160526823529413, rms_benchmark 13.34882615613099, max_benchmark 18.400618, min_benchmark 7.568259, lost messages 0.00 % | [perception/image](https://github.com/robotperf/rosbags/tree/main/perception/image) |
| [:black_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-8700K | power | 8.779195785522461 | workstation | 2023-07-01 17:47:49 |  | [perception/image](https://github.com/robotperf/rosbags/tree/main/perception/image) |
| [:black_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-8700K | throughput | 32.865597813885636 | workstation | 2023-07-01 17:51:14 | mean_benchmark 30.004940357991266, rms_benchmark 30.012328118390013, max_benchmark 32.865597813885636, min_benchmark 28.0221810134255, lost messages 0.00 % | [perception/image](https://github.com/robotperf/rosbags/tree/main/perception/image) |

