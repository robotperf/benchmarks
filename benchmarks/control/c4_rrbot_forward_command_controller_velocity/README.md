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
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-12700H | latency | 0.07 | workstation | 08-07-2023 | mean 0.02 ms, rms 0.02 ms, max 0.07 ms, min 0.00 ms, lost 0.00%, update rate 100 Hz | [N/A](https://github.com/robotperf/rosbags/tree/main/N/A) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-12700H | throughput | 100.0 | workstation | 08-07-2023 | lost 0.00%, update rate 100 Hz | [N/A](https://github.com/robotperf/rosbags/tree/main/N/A) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-12700H | power | 9.01890754699707 | workstation | 08-07-2023 | mean 0.02 ms, rms 0.02 ms, max 0.07 ms, min 0.00 ms, lost 0.00%, update rate 100 Hz | [N/A](https://github.com/robotperf/rosbags/tree/main/N/A) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i5-13600K | latency | 0.02 | workstation | 08-07-2023 | mean 0.01, ms, rms 0.01 ms, max 0.02 ms, min 0.00 ms, lost 0.00%, update rate 100 Hz | [N/A](https://github.com/robotperf/rosbags/tree/main/N/A) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i5-13600K | power | 34.11692428588867 | workstation | 08-07-2023 | mean 0.01, ms, rms 0.01 ms, max 0.02 ms, min 0.00 ms, lost 0.00%, update rate 100 Hz | [N/A](https://github.com/robotperf/rosbags/tree/main/N/A) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i5-13600K | throughput | 100.0 | workstation | 08-07-2023 | mean 0.01, ms, rms 0.01 ms, max 0.02 ms, min 0.00 ms, lost 0.00%, update rate 100 Hz | [N/A](https://github.com/robotperf/rosbags/tree/main/N/A) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-8700K | latency | 0.09077700000000001 | workstation | 2023-07-20 14:43:27 | ✋mean_benchmark 0.02008280421377184, rms_benchmark 0.02164900963633569, max_benchmark 0.09077700000000001, min_benchmark 0.003523, lost messages 0.03 % | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-8700K | power | 5.9694318771362305 | workstation | 2023-07-20 14:46:57 | ✋ | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-8700K | throughput | 100.0 | workstation | 2023-07-20 14:50:57 | ✋mean_benchmark 100.0, rms_benchmark 100.0, max_benchmark 108.34, min_benchmark 94.94, lost messages 0.03 % | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | NVIDIA AGX Orin Dev. Kit | latency | 0.125058 | edge | 2023-07-21 17:06:41 | ✋mean_benchmark 0.012953385667863861, rms_benchmark 0.014389249544548049, max_benchmark 0.125058, min_benchmark 0.001664, lost messages 0.07 % | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | NVIDIA AGX Orin Dev. Kit | power | 11.045801162719727 | edge | 2023-07-21 17:33:38 | ✋ | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | NVIDIA Jetson Nano | latency | 1.990149 | edge | 2023-07-24 11:13:08 | ✋mean_benchmark 0.0325762216120808, rms_benchmark 0.043266747737856784, max_benchmark 1.990149, min_benchmark 0.016562999999999998, lost messages 0.04 % | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | NVIDIA Jetson Nano | power | 0.0 | edge | 2023-07-24 11:15:38 | ✋ | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | NVIDIA Jetson Nano | throughput | 100.01 | edge | 2023-07-24 11:18:00 | ✋mean_benchmark 100.01, rms_benchmark 100.02, max_benchmark 111.5, min_benchmark 89.41, lost messages 0.04 % | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Kria KV260 | latency | 0.160621 | edge | 2023-07-24 23:37:01 | ✋mean_benchmark 0.036609515852442674, rms_benchmark 0.03677491040256997, max_benchmark 0.160621, min_benchmark 0.02577, lost messages 0.04 % | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Kria KV260 | power | 3.630000114440918 | edge | 2023-07-24 23:39:26 | ✋ | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Kria KV260 | throughput | 100.0 | edge | 2023-07-24 23:42:04 | ✋mean_benchmark 100.0, rms_benchmark 100.0, max_benchmark 103.41, min_benchmark 96.77, lost messages 0.04 % | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Kria KR260 | latency | 0.26609299999999997 | edge | 2023-07-25 01:30:55 | ✋mean_benchmark 0.03586736852833265, rms_benchmark 0.03663931127401722, max_benchmark 0.26609299999999997, min_benchmark 0.010110000000000001, lost messages 0.04 % | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Kria KR260 | power | 3.369999885559082 | edge | 2023-07-25 01:33:09 | ✋ | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | AMD Ryzen 5 PRO 4650G | latency | 0.0462 | workstation | 2023-09-13 17:29:00 | ✋mean_benchmark 0.0179 ms, rms_benchmark 0.0181 ms, max_benchmark 0.0462 ms, min_benchmark 0.0075 ms, lost messages 0.00 % | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | AMD Ryzen 5 PRO 4650G | power | 5.034379959106445 | workstation | 2023-09-13 17:29:00 | ✋mean_benchmark 0.0179 ms, rms_benchmark 0.0181 ms, max_benchmark 0.0462 ms, min_benchmark 0.0075 ms, lost messages 0.00 % | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | AMD Ryzen 5 PRO 4650G | throughput | 100.01 | workstation | 2023-09-13 17:29:00 | ✋mean_benchmark 100.01 fps, rms_benchmark 100.01 fps, max_benchmark 111.41 fps, min_benchmark 83.10 fps, lost messages 0.00 % | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | AMD Ryzen 5 PRO 4650G | latency | 0.0532 | workstation | 2023-09-13 17:31:00 | ✋mean_benchmark 0.0167 ms, rms_benchmark 0.0168 ms, max_benchmark 0.0532 ms, min_benchmark 0.0072 ms, lost messages 0.00 % | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | AMD Ryzen 5 PRO 4650G | power | 5.118247032165527 | workstation | 2023-09-13 17:31:00 | ✋mean_benchmark 0.0167 ms, rms_benchmark 0.0168 ms, max_benchmark 0.0532 ms, min_benchmark 0.0072 ms, lost messages 0.00 % | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | AMD Ryzen 5 PRO 4650G | throughput | 100.01 | workstation | 2023-09-13 17:31:00 | ✋mean_benchmark 100.01 fps, rms_benchmark 100.01 fps, max_benchmark 105.65 fps, min_benchmark 95.46 fps, lost messages 0.00 % | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |

