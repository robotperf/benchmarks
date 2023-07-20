# c4_rrbot_forward_command_controller_velocity

Forward command controller with velocity commands

### ID
c4

### Description
A [ros2_control_demos](https://github.com/ros-controls/ros2_control_demos) used to demonstrate the [ros2_controllers](https://github.com/ros-controls/ros2_controllers) forward_command_controller with velocity commands


![](../../../imgs/c3_rrbot_forward_command_controller_position.svg)

## Reproduction Steps

```bash
Refer to https://github.com/robotperf/benchmarks/tree/main/benchmarks/control/c4_rrbot_forward_command_controller_velocity and review the launch files to reproduce this package.
```

## Results

| Type | Hardware | Metric | Value | Category | Timestamp | Note | Data Source |
| --- | --- | --- | --- | --- | --- | --- | --- |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-12700H | latency | 10.04 | workstation | 08-07-2023 | mean 0.21 ms, rms 1.17 ms, max 10.04 ms, min 0.06 ms, lost 0.00%, update rate 10 Hz | [N/A](https://github.com/robotperf/rosbags/tree/main/N/A) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-12700H | power | 8.0799007415 | workstation | 08-07-2023 | mean 0.21 ms, rms 1.17 ms, max 10.04 ms, min 0.06 ms, lost 0.00%, update rate 10 Hz | [N/A](https://github.com/robotperf/rosbags/tree/main/N/A) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i5-13600K | latency | 0.44 | workstation | 08-07-2023 | mean 0.02, ms, rms 0.05 ms, max 0.44 ms, min 0.009 ms, lost 0.00%, update rate 10 Hz | [N/A](https://github.com/robotperf/rosbags/tree/main/N/A) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i5-13600K | power | 35.25370407104492 | workstation | 08-07-2023 | mean 0.02, ms, rms 0.05 ms, max 0.44 ms, min 0.009 ms, lost 0.00%, update rate 10 Hz | [N/A](https://github.com/robotperf/rosbags/tree/main/N/A) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-8700K | latency | 0.09077700000000001 | workstation | 2023-07-20 14:43:27 | ✋mean_benchmark 0.02008280421377184, rms_benchmark 0.02164900963633569, max_benchmark 0.09077700000000001, min_benchmark 0.003523, lost messages 0.03 % | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-8700K | power | 5.9694318771362305 | workstation | 2023-07-20 14:46:57 | ✋ | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-8700K | throughput | 108.34 | workstation | 2023-07-20 14:50:57 | ✋mean_benchmark 100.0, rms_benchmark 100.0, max_benchmark 108.34, min_benchmark 94.94, lost messages 0.03 % | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |

