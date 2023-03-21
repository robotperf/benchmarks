# a3_stereo_image_proc

Stereo image processing to compute a disparity map given a left and right image

### ID

a3

### Description

The stereo_image_proc package computes the disparity map, which represents the difference in pixel position between corresponding points in the stereo images. Disparity images are images that show the difference in displacement or distance between corresponding points in a pair of stereo images. Stereo images are two images of the same scene taken from slightly different viewpoints, such as from the left and right eyes of a human observer or from two cameras.Disparity images are often used in computer vision and image processing applications, such as 3D reconstruction, depth estimation, and object recognition. By analyzing the disparities between corresponding points in stereo images, it is possible to infer the depth of objects in the scene and create a 3D representation of the environment.

![](imgs/a3_stereo_image_proc_graph.png)

**Metric**: latency (ms)

## Installation
```bash
# Create a ROS 2 overlay workspace
mkdir -p /tmp/benchmark_ws/src

# Clone the benchmark repository
cd /tmp/benchmark_ws/src && git clone https://github.com/robotperf/benchmarks

# Fetch dependencies
source /opt/ros/humble/setup.bash
cd /tmp/benchmark_ws && sudo rosdep update || true && sudo apt-get update &&
  sudo rosdep install --from-paths src --ignore-src --rosdistro humble -y

# Install the stereo_image_proc package
sudo apt-get install ros-humble-stereo-image-proc

# Build the benchmark
colcon build --merge-install --packages-up-to a3_stereo_image_proc
```

## Reproduction Steps

```bash
# Terminal 1: Replay rosbag in a loop (within rosbags repo)
cd /path/to/rosbags/perception
ros2 bag play --loop image3

# Verify you are working in the workspace
cd /tmp/benchmark_ws

# Source the workspace as an overlay
source install/setup.bash

# Terminal 2: Test Stereo Image
ros2 launch a3_stereo_image_proc trace_a3_stereo_image_proc.launch.py

# Terminal 3: Visualize Image Feed
rviz2 -d benchmarks/perception/a3_stereo_image_proc/config/camera_bot.rviz

# Terminal 4: View Stereo Image Disparty Map
ros2 run image_view disparity_view image:=/benchmark/disparity

# Terminal 5: View Graph
rqt_graph

# Verify data was collected
cd ~/.ros/tracing
babeltrace a3_stereo_image_proc | less
```

## Reproduce Simulation
```bash
# Verify you are working in the workspace
cd /tmp/benchmark_ws

# Source
source install/setup.bash

# Terminal 1: Launch Simulation
cd /tmp/benchmark_ws 
ros2 launch a3_stereo_image_proc launch_sim.launch.py world:=benchmarks/perception/a3_stereo_image_proc/worlds/neighborhood.world

# Terminal 2: Visualize perception from robot's perspective using rviz
rviz2 -d benchmarks/perception/a3_stereo_image_proc/config/camera_bot.rviz

# Terminal 3: Record a rosbag (ctrl+c to end recording)
ros2 bag record -a

# Terminal 4: Use keyboard to drive robot around world
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### Visualization of the Neighborhood World
![](imgs/neighborhood_world.png)

```bash
# Verify: Play back rosbag after recording it in a loop
ros2 bag play --loop /path/to/bag/directory/
```

### Recorded Data being Played in a Loop
![](imgs/recorded_rosbag_rviz.gif)

## Results

