# c3_rrbot_forward_command_controller_position

Forward command controller Position

### ID
c3

### Description
A [ros2_control_demos](https://github.com/ros-controls/ros2_control_demos) used to demonstrate the [ros2_controllers](https://github.com/ros-controls/ros2_controllers) forward_command_controller


![](../../../imgs/c3_rrbot_forward_command_controller_position.svg)

## Reproduction Steps

```bash
Refer to https://github.com/robotperf/benchmarks/tree/main/benchmarks/control/c3_rrbot_forward_command_controller_position and review the launch files to reproduce this package.
```

## Results

| Type | Hardware | Metric | Value | Category | Timestamp | Note | Data Source |
| --- | --- | --- | --- | --- | --- | --- | --- |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-12700H | latency | 0.05 | workstation | 10-07-2023 | mean 0.02 ms, rms 0.02 ms. max 0.05 ms, min 0.009 ms, lost 0.00%, update rate 100 Hz, position control | [N/A](https://github.com/robotperf/rosbags/tree/main/N/A) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-12700H | power | 8.9 | workstation | 10-07-2023 | mean 0.02 ms, rms 0.02 ms. max 0.05 ms, min 0.009 ms, lost 0.00%, update rate 100 Hz, position control | [N/A](https://github.com/robotperf/rosbags/tree/main/N/A) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i5-13600K | latency | 0.01 | workstation | 10-07-2023 | 0.009 ms 0.009 ms 0.01 ms 0.009 ms, lost 0.00%, update rate 100 Hz, position control | [N/A](https://github.com/robotperf/rosbags/tree/main/N/A) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i5-13600K | power | 29.24 | workstation | 10-07-2023 | 0.009 ms 0.009 ms 0.01 ms 0.009 ms, lost 0.00%, update rate 100 Hz, position control | [N/A](https://github.com/robotperf/rosbags/tree/main/N/A) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-8700K | latency | 0.13906400000000002 | workstation | 2023-07-20 14:31:33 | ✋mean_benchmark 0.01652545670103093, rms_benchmark 0.016665976941803988, max_benchmark 0.13906400000000002, min_benchmark 0.002971, lost messages 0.07 % | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-8700K | power | 5.978651523590088 | workstation | 2023-07-20 14:35:33 | ✋ | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-8700K | throughput | 100.86 | workstation | 2023-07-20 14:39:15 | ✋mean_benchmark 100.0, rms_benchmark 100.0, max_benchmark 100.86, min_benchmark 99.02, lost messages 0.07 % | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | NVIDIA AGX Orin Dev. Kit | latency | 0.129058 | edge | 2023-07-21 17:04:20 | ✋mean_benchmark 0.012688248354693453, rms_benchmark 0.013911629579628325, max_benchmark 0.129058, min_benchmark 0.0017919999999999998, lost messages 0.07 % | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | NVIDIA AGX Orin Dev. Kit | power | 11.005424499511719 | edge | 2023-07-21 17:28:55 | ✋ | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | NVIDIA AGX Orin Dev. Kit | throughput | 39111.39 | edge | 2023-07-21 18:47:20 | ✋mean_benchmark 1625.22, rms_benchmark 7098.91, max_benchmark 39111.39, min_benchmark 0.35, lost messages 0.07 % | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |

