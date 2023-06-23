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
| :white_circle: | 12th Gen Intel(R) Core(TM) i9-12900KF | latency | 132.12 | edge | 25-04-2023 | Mean: 26.25 ms,  RMS: 27.18 ms, Max: 132.12 ms, Min: 8.73 ms over 1124 samples. | perception/image3 |

