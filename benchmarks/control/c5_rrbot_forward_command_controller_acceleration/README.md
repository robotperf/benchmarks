# c5_rrbot_forward_command_controller_acceleration

Forward command controller with acceleration commands

### ID
c5

### Description
A [ros2_control_demos](https://github.com/ros-controls/ros2_control_demos) used to demonstrate the [ros2_controllers](https://github.com/ros-controls/ros2_controllers) forward_command_controller with acceleration commands


![](../../../imgs/c3_rrbot_forward_command_controller_position.svg)

## Reproduction Steps

```bash
Refer to https://github.com/robotperf/benchmarks/tree/main/benchmarks/control/c5_rrbot_forward_command_controller_acceleration and review the launch files to reproduce this package.
```

## Results

| Type | Hardware | Metric | Value | Category | Timestamp | Note | Data Source |
| --- | --- | --- | --- | --- | --- | --- | --- |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-12700H | latency | 0.07 | workstation | 08-07-2023 | mean 0.02 ms, rms 0.02 ms, max 0.07 ms, min 0.00 ms, lost 0.00%, update rate 100 Hz | [N/A](https://github.com/robotperf/rosbags/tree/main/N/A) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-12700H | power | 8.663870811462402 | workstation | 08-07-2023 | mean 0.02 ms, rms 0.02 ms, max 0.07 ms, min 0.00 ms, lost 0.00%, update rate 100 Hz | [N/A](https://github.com/robotperf/rosbags/tree/main/N/A) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-12700H | throughput | 100.0 | workstation | 08-07-2023 | mean 0.02 ms, rms 0.02 ms, max 0.07 ms, min 0.00 ms, lost 0.00%, update rate 100 Hz | [N/A](https://github.com/robotperf/rosbags/tree/main/N/A) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i5-13600K | latency | 0.01 | workstation | 08-07-2023 | mean 0.00, ms, rms 0.00 ms, max 0.01 ms, min 0.00 ms, lost 0.00%, update rate 100 Hz | [N/A](https://github.com/robotperf/rosbags/tree/main/N/A) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i5-13600K | power | 35.30375671386719 | workstation | 08-07-2023 | mean 0.00, ms, rms 0.00 ms, max 0.01 ms, min 0.00 ms, lost 0.00%, update rate 100 Hz | [N/A](https://github.com/robotperf/rosbags/tree/main/N/A) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i5-13600K | throughput | 100.0 | workstation | 08-07-2023 | mean 0.00, ms, rms 0.00 ms, max 0.01 ms, min 0.00 ms, lost 0.00%, update rate 100 Hz | [N/A](https://github.com/robotperf/rosbags/tree/main/N/A) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-8700K | latency | 0.055621000000000004 | workstation | 2023-07-20 14:54:29 | ✋mean_benchmark 0.016474258575197888, rms_benchmark 0.01653362410999869, max_benchmark 0.055621000000000004, min_benchmark 0.003118, lost messages 0.07 % | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-8700K | power | 5.920037269592285 | workstation | 2023-07-20 14:57:43 | ✋ | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-8700K | throughput | 100.0 | workstation | 2023-07-20 15:01:14 | ✋mean_benchmark 100.0, rms_benchmark 100.0, max_benchmark 107.15, min_benchmark 93.83, lost messages 0.07 % | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | NVIDIA AGX Orin Dev. Kit | latency | 0.5222470000000001 | edge | 2023-07-21 17:09:12 | ✋mean_benchmark 0.011955726917440157, rms_benchmark 0.015192538054428393, max_benchmark 0.5222470000000001, min_benchmark 0.001728, lost messages 0.07 % | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | NVIDIA AGX Orin Dev. Kit | power | 11.179306030273438 | edge | 2023-07-21 17:31:18 | ✋ | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | NVIDIA AGX Orin Dev. Kit | throughput | 6718.71 | edge | 2023-07-21 17:55:36 | ✋mean_benchmark 6718.71, rms_benchmark 13985.87, max_benchmark 38437.88, min_benchmark 0.08, lost messages 0.07 % | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | NVIDIA Jetson Nano | latency | 0.326364 | edge | 2023-07-24 12:47:51 | ✋mean_benchmark 0.03523208970099668, rms_benchmark 0.04177455189022708, max_benchmark 0.326364, min_benchmark 0.016668, lost messages 0.04 % | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | NVIDIA Jetson Nano | power | 0.0 | edge | 2023-07-24 12:50:11 | ✋ | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | NVIDIA Jetson Nano | throughput | 100.01 | edge | 2023-07-24 12:53:20 | ✋mean_benchmark 100.01, rms_benchmark 100.02, max_benchmark 131.27, min_benchmark 90.59, lost messages 0.04 % | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Kria KV260 | latency | 0.496105 | edge | 2023-07-25 00:36:52 | ✋mean_benchmark 0.03631871313294331, rms_benchmark 0.037524251132079925, max_benchmark 0.496105, min_benchmark 0.010331, lost messages 0.04 % | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Kria KV260 | power | 3.609999895095825 | edge | 2023-07-25 00:39:09 | ✋ | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Kria KV260 | throughput | 107.32 | edge | 2023-07-25 00:41:36 | ✋mean_benchmark 107.32, rms_benchmark 216.5, max_benchmark 5360.44, min_benchmark 11.62, lost messages 0.04 % | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Kria KR260 | latency | 0.15377200000000002 | edge | 2023-07-25 02:35:27 | ✋mean_benchmark 0.03534882551319648, rms_benchmark 0.03548432930808023, max_benchmark 0.15377200000000002, min_benchmark 0.011151, lost messages 0.02 % | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Kria KR260 | power | 3.7200000286102295 | edge | 2023-07-25 02:37:43 | ✋ | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Kria KR260 | throughput | 100.0 | edge | 2023-07-25 02:40:08 | ✋mean_benchmark 100.0, rms_benchmark 100.0, max_benchmark 131.81, min_benchmark 80.56, lost messages 0.02 % | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | AMD Ryzen 5 PRO 4650G | latency | 0.0438 | workstation | 2023-09-13 17:31:00 | ✋mean_benchmark 0.0175 ms, rms_benchmark 0.0178 ms, max_benchmark 0.0438 ms, min_benchmark 0.0054 ms, lost messages 0.00 % | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | AMD Ryzen 5 PRO 4650G | power | 5.105885982513428 | workstation | 2023-09-13 17:31:00 | ✋mean_benchmark 0.0175 ms, rms_benchmark 0.0178 ms, max_benchmark 0.0438 ms, min_benchmark 0.0054 ms, lost messages 0.00 % | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | AMD Ryzen 5 PRO 4650G | throughput | 100.0 | workstation | 2023-09-13 17:31:00 | ✋mean_benchmark 100.00 fps, rms_benchmark 100.01 fps, max_benchmark 127.94 fps, min_benchmark 82.08 fps, lost messages 0.00 % | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | AMD Ryzen 5 PRO 4650G | latency | 0.0729 | workstation | 2023-09-13 17:33:00 | ✋mean_benchmark 0.0185 ms, rms_benchmark 0.0186 ms, max_benchmark 0.0729 ms, min_benchmark 0.0076 ms, lost messages 0.00 % | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | AMD Ryzen 5 PRO 4650G | power | 5.199840068817139 | workstation | 2023-09-13 17:33:00 | ✋mean_benchmark 0.0185 ms, rms_benchmark 0.0186 ms, max_benchmark 0.0729 ms, min_benchmark 0.0076 ms, lost messages 0.00 % | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | AMD Ryzen 5 PRO 4650G | throughput | 100.0 | workstation | 2023-09-13 17:33:00 | ✋mean_benchmark 100.00 fps, rms_benchmark 100.01 fps, max_benchmark 109.15 fps, min_benchmark 92.24 fps, lost messages 0.00 % | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |

