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
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel® Core™ i5-8250U CPU @ 1.60GHz × 8 | latency | 33.68 | workstation | 08-05-2023 |  | [perception/image2](https://github.com/robotperf/rosbags/tree/main/perception/image2) |
| [:black_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | NVIDIA AGX Orin Dev. Kit | power | 11.99144172668457 | edge | 2023-06-30 21:06:37 |  | [perception/image](https://github.com/robotperf/rosbags/tree/main/perception/image) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-8700K | latency | 42.158766 | workstation | 2023-07-01 19:22:31 | mean_benchmark 14.496061530612247, rms_benchmark 15.356667129367795, max_benchmark 42.158766, min_benchmark 8.635698999999999, lost messages 4.08 % | [perception/image](https://github.com/robotperf/rosbags/tree/main/perception/image) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-8700K | power | 7.256417274475098 | workstation | 2023-07-01 19:25:55 |  | [perception/image](https://github.com/robotperf/rosbags/tree/main/perception/image) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-8700K | throughput | 41.0098276361045 | workstation | 2023-07-01 19:29:08 | mean_benchmark 28.13782469333202, rms_benchmark 28.724165714347563, max_benchmark 41.0098276361045, min_benchmark 7.467416618321986, lost messages 4.08 % | [perception/image](https://github.com/robotperf/rosbags/tree/main/perception/image) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-8700K | latency | 20.891331 | workstation | 2023-07-01 19:32:27 | mean_benchmark 12.440568647058823, rms_benchmark 12.96673046128797, max_benchmark 20.891331, min_benchmark 6.284897, lost messages 0.00 % | [perception/image](https://github.com/robotperf/rosbags/tree/main/perception/image) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-8700K | power | 8.132538795471191 | workstation | 2023-07-01 19:35:40 |  | [perception/image](https://github.com/robotperf/rosbags/tree/main/perception/image) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-8700K | throughput | 34.55349916548109 | workstation | 2023-07-01 19:39:11 | mean_benchmark 30.022796000514997, rms_benchmark 30.056996886641937, max_benchmark 34.55349916548109, min_benchmark 27.56425662355994, lost messages 0.00 % | [perception/image](https://github.com/robotperf/rosbags/tree/main/perception/image) |
| [:black_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-8700K | latency | 35.53647 | workstation | 2023-07-01 19:42:28 | mean_benchmark 11.631717441176473, rms_benchmark 12.280089043557881, max_benchmark 35.53647, min_benchmark 5.307922, lost messages 0.00 % | [perception/image](https://github.com/robotperf/rosbags/tree/main/perception/image) |
| [:black_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-8700K | power | 8.141988754272461 | workstation | 2023-07-01 19:45:59 |  | [perception/image](https://github.com/robotperf/rosbags/tree/main/perception/image) |
| [:black_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-8700K | throughput | 34.86660264461089 | workstation | 2023-07-01 19:49:10 | mean_benchmark 30.10943593812154, rms_benchmark 30.127403064335265, max_benchmark 34.86660264461089, min_benchmark 27.652619716555716, lost messages 0.00 % | [perception/image](https://github.com/robotperf/rosbags/tree/main/perception/image) |

