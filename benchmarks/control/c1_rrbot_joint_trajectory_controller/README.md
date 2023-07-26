# c1_rrbot_joint_trajectory_controller

Joint trajectory controller

### ID
c1

### Description
A [ros2_control_demos](https://github.com/ros-controls/ros2_control_demos) used to demonstrate the [ros2_controllers](https://github.com/ros-controls/ros2_controllers) joint_trajectory_controller


![](../../../imgs/c1_rrbot_joint_trajectory_controller.svg)

## Reproduction Steps

```bash
Refer to https://github.com/robotperf/benchmarks/tree/main/benchmarks/control/c1_rrbot_joint_trajectory_controller and review the launch files to reproduce this package.
```

## Results

| Type | Hardware | Metric | Value | Category | Timestamp | Note | Data Source |
| --- | --- | --- | --- | --- | --- | --- | --- |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-12700H | latency | 0.55 | workstation | 08-07-2023 | mean 0.08 ms, rms 0.10 ms, max 0.55 ms, min 0.02 ms, lost 0.00%, update rate 10 Hz | [N/A](https://github.com/robotperf/rosbags/tree/main/N/A) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-12700H | power | 8.297782897949219 | workstation | 08-07-2023 | mean 0.08 ms, rms 0.10 ms, max 0.55 ms, min 0.02 ms, lost 0.00%, update rate 10 Hz | [N/A](https://github.com/robotperf/rosbags/tree/main/N/A) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i5-13600K | latency | 0.13 | workstation | 08-07-2023 | mean 0.02 ms, rms 0.03 ms, max 0.13 ms, min 0.01 ms, lost 0.00%, update rate 10 Hz | [N/A](https://github.com/robotperf/rosbags/tree/main/N/A) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i5-13600K | power | 28.091297149658203 | workstation | 08-07-2023 | mean 0.02 ms, rms 0.03 ms, max 0.13 ms, min 0.01 ms, lost 0.00%, update rate 10 Hz | [N/A](https://github.com/robotperf/rosbags/tree/main/N/A) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-8700K | latency | 0.098052 | workstation | 2023-07-20 14:12:00 | ✋mean_benchmark 0.036972090909090906, rms_benchmark 0.046425814648944845, max_benchmark 0.098052, min_benchmark 0.011147999999999998, lost messages 0.00 % | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-8700K | power | 11.228675842285156 | workstation | 2023-07-20 14:15:19 | ✋ | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-8700K | throughput | 10.01 | workstation | 2023-07-20 14:18:49 | ✋mean_benchmark 10.0, rms_benchmark 10.0, max_benchmark 10.01, min_benchmark 9.99, lost messages 0.00 % | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | NVIDIA AGX Orin Dev. Kit | latency | 1.472075 | edge | 2023-07-21 16:59:11 | ✋mean_benchmark 0.20849618525896416, rms_benchmark 0.236222650035889, max_benchmark 1.472075, min_benchmark 0.025216, lost messages 0.20 % | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | NVIDIA AGX Orin Dev. Kit | power | 10.891936302185059 | edge | 2023-07-21 17:24:12 | ✋ | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | NVIDIA AGX Orin Dev. Kit | throughput | 10.03 | edge | 2023-07-21 17:48:23 | ✋mean_benchmark 10.0, rms_benchmark 10.0, max_benchmark 10.03, min_benchmark 9.98, lost messages 0.20 % | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |

