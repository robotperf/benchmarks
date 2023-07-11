# c3_rrbot_forward_command_controller_position

Forward command controller with position commands

### ID
c3

### Description
A [ros2_control_demos](https://github.com/ros-controls/ros2_control_demos) used to demonstrate the [ros2_controllers](https://github.com/ros-controls/ros2_controllers) forward_command_controller with position commands


![](../../../imgs/c3_rrbot_forward_command_controller_position.svg)

## Reproduction Steps

```bash
Refer to https://github.com/robotperf/benchmarks/tree/main/benchmarks/control/c3_rrbot_forward_command_controller and review the launch files to reproduce this package.
```

## Results

| Type | Hardware | Metric | Value | Category | Timestamp | Note | Data Source |
| --- | --- | --- | --- | --- | --- | --- | --- |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-12700H | latency | 0.05 | workstation | 10-07-2023 | mean 0.02 ms, rms 0.02 ms. max 0.05 ms, min 0.009 ms, lost 0.00%, update rate 100 Hz, position control | [N/A](https://github.com/robotperf/rosbags/tree/main/N/A) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-12700H | power | 8.9 | workstation | 10-07-2023 | mean 0.02 ms, rms 0.02 ms. max 0.05 ms, min 0.009 ms, lost 0.00%, update rate 100 Hz, position control | [N/A](https://github.com/robotperf/rosbags/tree/main/N/A) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i5-13600K | latency | 0.01 | workstation | 10-07-2023 | 0.009 ms 0.009 ms 0.01 ms 0.009 ms, lost 0.00%, update rate 100 Hz, position control | [N/A](https://github.com/robotperf/rosbags/tree/main/N/A) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i5-13600K | power | 29.24 | workstation | 10-07-2023 | mean 0.02, ms, rms 0.05 ms, max 0.44 ms, min 0.009 ms, lost 0.00%, update rate 100 Hz, position control | [N/A](https://github.com/robotperf/rosbags/tree/main/N/A) |

