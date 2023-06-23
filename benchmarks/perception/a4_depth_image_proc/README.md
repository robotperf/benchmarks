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
| grey | 12th Gen Intel(R) Core(TM) i9-12900KF | latency | 939.43 | edge | 26-04-2023 | Mean: 37.09 ms,  RMS: 65.12 ms, Max: 939.43 ms, Min: 7.97 ms over 1147 samples. | perception/depth_image1 |

