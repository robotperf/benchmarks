# a4_depth_image_proc

Perception computational graph to compute a point cloud using a rectified depth image and a rectified color image.

### ID
a4

### Description
The [depth_image_proc](https://github.com/ros-perception/image_pipeline/tree/humble/depth_image_proc) package computes a point cloud using a rectified depth image and a rectified color image. 


![](/imgs/a3_stereo_image_proc_graph.png)

## Reproduction Steps

```bash
## ------- Installation -------
# Create a ROS 2 overlay workspace
mkdir -p /tmp/benchmark_ws/src

# Clone the benchmark repository
cd /tmp/benchmark_ws/src && git clone https://github.com/robotperf/benchmarks

# Fetch dependencies
source /opt/ros/humble/setup.bash
cd /tmp/benchmark_ws && sudo rosdep update || true && sudo apt-get update &&
  sudo rosdep install --from-paths src --ignore-src --rosdistro humble -y

# Build the benchmark
colcon build --merge-install --packages-up-to a4_depth_image_proc

# Terminal 1: Replay rosbag in a loop (within rosbags repo)
cd /path/to/rosbags/perception
ros2 bag play --loop depth_image1

# Verify you are working in the workspace
cd /tmp/benchmark_ws

# Source the workspace as an overlay
source install/setup.bash

# Terminal 2: Run Stereo Image Proc Benchmark
ros2 launch a4_depth_image_proc trace_a4_depth_image_proc.launch.py

# Verify data was collected
cd ~/.ros/tracing
babeltrace a4_depth_image_proc | less

# Analyze Data
ros2 launch a4_depth_image_proc analyze_a4_depth_image_proc.launch.py

# Optional: Visualize benchmark process while it's running using this script
chmod +x src/benchmarks/benchmarks/perception/a4_depth_image_proc/scripts/visualize_benchmark.sh
./src/benchmarks/benchmarks/perception/a4_depth_image_proc/scripts/visualize_benchmark.sh

## ------- Reproduce Simulation -------
cd /tmp/benchmark_ws

# Source
source install/setup.bash

# Terminal 1: Launch Simulation
cd /tmp/benchmark_ws 
ros2 launch a4_depth_image_proc launch_sim.launch.py world:=src/benchmarks/benchmarks/perception/a4_depth_image_proc/worlds/parking_garage.world

# Terminal 2: Visualize perception from robot's perspective using rviz
rviz2 -d src/benchmarks/benchmarks/perception/a4_depth_image_proc/config/sim_depth_camera_bot.rviz

# Terminal 3: Record a rosbag (ctrl+c to end recording)
ros2 bag record -a

# Terminal 4: Use keyboard to drive robot around world
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Verify: Play back rosbag after recording it in a loop
ros2 bag play --loop /path/to/bag/directory/

```

## Results

| Type | Hardware | Metric | Value | Category | Timestamp | Note | Data Source |
| --- | --- | --- | --- | --- | --- | --- | --- |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | 12th Gen Intel(R) Core(TM) i9-12900KF | latency | 939.43 | edge | 26-04-2023 | Mean: 37.09 ms,  RMS: 65.12 ms, Max: 939.43 ms, Min: 7.97 ms over 1147 samples. | [perception/depth_image1](https://github.com/robotperf/rosbags/tree/main/perception/depth_image1) |
| [:black_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | NVIDIA AGX Orin Dev. Kit | power | 11.513117790222168 | edge | 2023-06-30 20:37:42 |  | [perception/depth_image1](https://github.com/robotperf/rosbags/tree/main/perception/depth_image1) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-8700K | latency | 492.41293299999995 | workstation | 2023-07-01 18:52:57 | mean_benchmark 52.36187717005076, rms_benchmark 67.83441415522553, max_benchmark 492.41293299999995, min_benchmark 15.982555999999999, lost messages 1.14 % | [perception/image](https://github.com/robotperf/rosbags/tree/main/perception/image) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-8700K | power | 10.32787799835205 | workstation | 2023-07-01 18:56:07 |  | [perception/image](https://github.com/robotperf/rosbags/tree/main/perception/image) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-8700K | throughput | 14.56166787712958 | workstation | 2023-07-01 18:59:21 | mean_benchmark 10.01179515336643, rms_benchmark 10.053770754687228, max_benchmark 14.56166787712958, min_benchmark 1.401752375219967, lost messages 1.14 % | [perception/image](https://github.com/robotperf/rosbags/tree/main/perception/image) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-8700K | latency | 56.589352000000005 | workstation | 2023-07-01 19:02:52 | mean_benchmark 24.904020833333334, rms_benchmark 25.9619138102967, max_benchmark 56.589352000000005, min_benchmark 15.475099000000002, lost messages 0.00 % | [perception/image](https://github.com/robotperf/rosbags/tree/main/perception/image) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-8700K | power | 20.117462158203125 | workstation | 2023-07-01 19:06:20 |  | [perception/image](https://github.com/robotperf/rosbags/tree/main/perception/image) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-8700K | throughput | 34.984497844237744 | workstation | 2023-07-01 19:09:30 | mean_benchmark 30.164149180490657, rms_benchmark 30.17906066071334, max_benchmark 34.984497844237744, min_benchmark 29.23188800258132, lost messages 0.00 % | [perception/image](https://github.com/robotperf/rosbags/tree/main/perception/image) |
| [:black_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-8700K | latency | 32.137471999999995 | workstation | 2023-07-01 19:12:48 | mean_benchmark 27.460619466666667, rms_benchmark 27.531410528435206, max_benchmark 32.137471999999995, min_benchmark 23.952776, lost messages 0.00 % | [perception/image](https://github.com/robotperf/rosbags/tree/main/perception/image) |
| [:black_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-8700K | power | 22.733518600463867 | workstation | 2023-07-01 19:16:17 |  | [perception/image](https://github.com/robotperf/rosbags/tree/main/perception/image) |
| [:black_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-8700K | throughput | 32.56260364066182 | workstation | 2023-07-01 19:20:02 | mean_benchmark 30.002004606407837, rms_benchmark 30.016899528253816, max_benchmark 32.56260364066182, min_benchmark 28.05284763755054, lost messages 0.00 % | [perception/image](https://github.com/robotperf/rosbags/tree/main/perception/image) |

