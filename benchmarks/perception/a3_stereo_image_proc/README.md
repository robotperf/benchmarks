# a3_stereo_image_proc

Perception computational graph to compute a disparity map for stereo images.

### ID
a3

### Description
The [stereo_image_proc](https://github.com/ros-perception/image_pipeline/tree/humble/stereo_image_proc) package computes the disparity map using a left and right image. A disparity map represents the difference in pixel position between corresponding points in the stereo images. Disparity images are images that show the difference in displacement or distance between corresponding points in a pair of stereo images. Stereo images are two images of the same scene taken from slightly different viewpoints, such as from the left and right eyes of a human observer or from two cameras. Disparity images are often used in computer vision and image processing applications, such as 3D reconstruction, depth estimation, and object recognition. By analyzing the disparities between corresponding points in stereo images, it is possible to infer the depth of objects in the scene and create a 3D representation of the environment.
A simple perception computational graph composed by 2 Components, `rectify` and `resize` operations. Used to demonstrate a simple perception pipeline using the  package.


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
colcon build --merge-install --packages-up-to a3_stereo_image_proc

# Terminal 1: Replay rosbag in a loop (within rosbags repo)
cd /path/to/rosbags/perception
ros2 bag play --loop image3

# Verify you are working in the workspace
cd /tmp/benchmark_ws

# Source the workspace as an overlay
source install/setup.bash

# Terminal 2: Run Stereo Image Proc Benchmark
ros2 launch a3_stereo_image_proc trace_a3_stereo_image_proc.launch.py

# Verify data was collected
cd ~/.ros/tracing
babeltrace a3_stereo_image_proc | less

# Analyze Data
ros2 launch a3_stereo_image_proc analyze_a3_stereo_image_proc.launch.py

# Optional: Visualize benchmark process while it's running using this script
chmod +x src/benchmarks/benchmarks/perception/a3_stereo_image_proc/scripts/visualize_benchmark.sh
./src/benchmarks/benchmarks/perception/a3_stereo_image_proc/scripts/visualize_benchmark.sh

## ------- Reproduce Simulation -------
cd /tmp/benchmark_ws

# Source
source install/setup.bash

# Terminal 1: Launch Simulation
cd /tmp/benchmark_ws 
ros2 launch a3_stereo_image_proc launch_sim.launch.py world:=src/benchmarks/benchmarks/perception/a3_stereo_image_proc/worlds/neighborhood.world

# Terminal 2: Visualize perception from robot's perspective using rviz
rviz2 -d src/benchmarks/benchmarks/perception/a3_stereo_image_proc/config/camera_bot.rviz

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
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | 12th Gen Intel(R) Core(TM) i9-12900KF | latency | 132.12 | edge | 25-04-2023 | Mean: 26.25 ms,  RMS: 27.18 ms, Max: 132.12 ms, Min: 8.73 ms over 1124 samples. | [perception/image3](https://github.com/robotperf/rosbags/tree/main/perception/image3) |
| [:black_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | NVIDIA AGX Orin Dev. Kit | throughput | 33.277046318720096 | edge | 2023-06-30 20:21:42 | mean_benchmark 27.791574487768035, rms_benchmark 28.373919572519444, max_benchmark 33.277046318720096, min_benchmark 10.077661482684006, lost messages 16.28 % | [perception/image3](https://github.com/robotperf/rosbags/tree/main/perception/image3) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-8700K | latency | 61.133489 | workstation | 2023-07-01 18:23:26 | mean_benchmark 40.67432786374315, rms_benchmark 41.17130628578445, max_benchmark 61.133489, min_benchmark 20.81411, lost messages 0.63 % | [perception/image](https://github.com/robotperf/rosbags/tree/main/perception/image) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-8700K | power | 5.897109508514404 | workstation | 2023-07-01 18:26:40 |  | [perception/image](https://github.com/robotperf/rosbags/tree/main/perception/image) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-8700K | throughput | 12.975265148245784 | workstation | 2023-07-01 18:29:58 | mean_benchmark 9.985250716576333, rms_benchmark 10.005649079478957, max_benchmark 12.975265148245784, min_benchmark 1.6551238056605884, lost messages 0.63 % | [perception/image](https://github.com/robotperf/rosbags/tree/main/perception/image) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-8700K | latency | 59.300551 | workstation | 2023-07-01 18:33:18 | mean_benchmark 22.975132699999996, rms_benchmark 26.949610502463365, max_benchmark 59.300551, min_benchmark 7.832851, lost messages 0.00 % | [perception/image](https://github.com/robotperf/rosbags/tree/main/perception/image) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-8700K | power | 9.084548950195312 | workstation | 2023-07-01 18:36:30 |  | [perception/image](https://github.com/robotperf/rosbags/tree/main/perception/image) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-8700K | throughput | 31.884678474583428 | workstation | 2023-07-01 18:40:02 | mean_benchmark 30.021894043899596, rms_benchmark 30.03041143308092, max_benchmark 31.884678474583428, min_benchmark 28.689014184651903, lost messages 0.00 % | [perception/image](https://github.com/robotperf/rosbags/tree/main/perception/image) |
| [:black_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-8700K | latency | 64.159806 | workstation | 2023-07-01 18:43:16 | mean_benchmark 22.797836120000003, rms_benchmark 26.96963921531789, max_benchmark 64.159806, min_benchmark 8.604811999999999, lost messages 0.00 % | [perception/image](https://github.com/robotperf/rosbags/tree/main/perception/image) |
| [:black_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-8700K | power | 9.158670425415039 | workstation | 2023-07-01 18:46:31 |  | [perception/image](https://github.com/robotperf/rosbags/tree/main/perception/image) |
| [:black_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-8700K | throughput | 34.36352784361543 | workstation | 2023-07-01 18:49:44 | mean_benchmark 30.085827751710223, rms_benchmark 30.09714182807465, max_benchmark 34.36352784361543, min_benchmark 28.252331403576672, lost messages 0.00 % | [perception/image](https://github.com/robotperf/rosbags/tree/main/perception/image) |

