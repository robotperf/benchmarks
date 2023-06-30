# c3

Forward command controller.

### ID
c3

### Description
A [ros2_control_demos](https://github.com/ros-controls/ros2_control_demos) used to demonstrate the [ros2_controllers](https://github.com/ros-controls/ros2_controllers) forward_command_controller

#### Position-based control
![](../../../imgs/c3_rrbot_forward_command_controller_position.svg)

#### Velocity-based control
![](../../../imgs/c3_rrbot_forward_command_controller_velocity.svg)

#### Acceleration-based control
![](../../../imgs/c3_rrbot_forward_command_controller_acceleration.svg)

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
  colcon build --merge-install --packages-up-to c3_rrbot_forward_command_controller
  
  # Source the workspace as an overlay
  source install/setup.bash

  # Select what type of controller (position, velocity or acceleration) to launch before launching the trace file
  export EXAMPLE_3_CONTROLLER_TYPE=position
  ros2 launch c3_rrbot_forward_command_controller trace_c3_rrbot_forward_command_controller_power.launch.py

  # Analyze trace files
  cd path/to/benchmark_ws
  ros2 launch c3_rrbot_forward_command_controller analyze_c3_rrbot_forward_command_controller.launch.py trace_path:=/root/.ros/tracing/c3_rrbot_forward_command_controller metrics:=[latency,power]

```

## Results

Data not valid at the moment, just a placeholder for now

| Type | Hardware | Metric | Value | Category | Timestamp | Note | Data Source |
| --- | --- | --- | --- | --- | --- | --- | --- |
| grey | Intel® Core™ i5-8250U CPU @ 1.60GHz × 8 | latency | 33.68 | workstation | 08-05-2023 |  | perception/image2 |

