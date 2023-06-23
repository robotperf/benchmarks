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
| grey | ROBOTCORE | latency | 66.82 | edge | 14-10-2022 |  | perception/image |
| grey | Kria KR260 | latency | 66.82 | edge | 14-10-2022 |  | perception/image |
| grey | Jetson Nano | latency | 238.13 | edge | 14-10-2022 |  | perception/image |
| grey | Jetson AGX Xavier | latency | 106.34 | edge | 14-10-2022 |  | perception/image |

