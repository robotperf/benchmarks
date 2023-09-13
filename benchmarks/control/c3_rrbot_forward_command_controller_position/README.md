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
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-12700H | power | 8.908805847167969 | workstation | 10-07-2023 | mean 0.02 ms, rms 0.02 ms. max 0.05 ms, min 0.009 ms, lost 0.00%, update rate 100 Hz, position control | [N/A](https://github.com/robotperf/rosbags/tree/main/N/A) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i5-13600K | latency | 0.01 | workstation | 10-07-2023 | 0.009 ms 0.009 ms 0.01 ms 0.009 ms, lost 0.00%, update rate 100 Hz, position control | [N/A](https://github.com/robotperf/rosbags/tree/main/N/A) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i5-13600K | power | 29.240436553955078 | workstation | 10-07-2023 | 0.009 ms 0.009 ms 0.01 ms 0.009 ms, lost 0.00%, update rate 100 Hz, position control | [N/A](https://github.com/robotperf/rosbags/tree/main/N/A) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-8700K | latency | 0.13906400000000002 | workstation | 2023-07-20 14:31:33 | ✋mean_benchmark 0.01652545670103093, rms_benchmark 0.016665976941803988, max_benchmark 0.13906400000000002, min_benchmark 0.002971, lost messages 0.07 % | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-8700K | power | 5.978651523590088 | workstation | 2023-07-20 14:35:33 | ✋ | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-8700K | throughput | 100.0 | workstation | 2023-07-20 14:39:15 | ✋mean_benchmark 100.0, rms_benchmark 100.0, max_benchmark 100.86, min_benchmark 99.02, lost messages 0.07 % | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | NVIDIA AGX Orin Dev. Kit | latency | 0.129058 | edge | 2023-07-21 17:04:20 | ✋mean_benchmark 0.012688248354693453, rms_benchmark 0.013911629579628325, max_benchmark 0.129058, min_benchmark 0.0017919999999999998, lost messages 0.07 % | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | NVIDIA AGX Orin Dev. Kit | power | 11.005424499511719 | edge | 2023-07-21 17:28:55 | ✋ | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | NVIDIA AGX Orin Dev. Kit | throughput | 1625.22 | edge | 2023-07-21 18:47:20 | ✋mean_benchmark 1625.22, rms_benchmark 7098.91, max_benchmark 39111.39, min_benchmark 0.35, lost messages 0.07 % | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | NVIDIA Jetson Nano | latency | 0.166567 | edge | 2023-07-24 09:39:58 | ✋mean_benchmark 0.03043539281388777, rms_benchmark 0.031015526416291777, max_benchmark 0.166567, min_benchmark 0.016563, lost messages 0.00 % | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | NVIDIA Jetson Nano | power | 0.0 | edge | 2023-07-24 09:42:22 | ✋ | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | NVIDIA Jetson Nano | throughput | 100.01 | edge | 2023-07-24 09:44:43 | ✋mean_benchmark 100.01, rms_benchmark 100.02, max_benchmark 116.35, min_benchmark 87.72, lost messages 0.00 % | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Kria KV260 | latency | 0.363504 | edge | 2023-07-24 22:37:00 | ✋mean_benchmark 0.03608016535433071, rms_benchmark 0.03650452117458101, max_benchmark 0.363504, min_benchmark 0.01086, lost messages 0.02 % | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Kria KV260 | power | 3.359999895095825 | edge | 2023-07-24 22:46:43 | ✋ | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Kria KV260 | throughput | 100.0 | edge | 2023-07-24 22:49:16 | ✋mean_benchmark 100.0, rms_benchmark 100.0, max_benchmark 124.39, min_benchmark 83.64, lost messages 0.02 % | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Kria KR260 | latency | 0.263374 | edge | 2023-07-25 00:27:02 | ✋mean_benchmark 0.05087436282801995, rms_benchmark 0.055017706254678896, max_benchmark 0.263374, min_benchmark 0.011219999999999999, lost messages 0.00 % | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Kria KR260 | power | 3.549999952316284 | edge | 2023-07-25 00:29:17 | ✋ | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Kria KR260 | throughput | 100.0 | edge | 2023-07-25 00:31:46 | ✋mean_benchmark 100.0, rms_benchmark 100.0, max_benchmark 104.27, min_benchmark 96.03, lost messages 0.00 % | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | AMD Ryzen 5 PRO 4650G | latency | 0.0439 | workstation | 2023-09-13 17:27:52 | ✋mean_benchmark 0.0159 ms, rms_benchmark 0.0161 ms, max_benchmark 0.0439 ms, min_benchmark 0.0095 ms, lost messages 0.00 % | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | AMD Ryzen 5 PRO 4650G | power | 5.030899524688721 | workstation | 2023-09-13 17:27:52 | ✋mean_benchmark 0.0159 ms, rms_benchmark 0.0161 ms, max_benchmark 0.0439 ms, min_benchmark 0.0095 ms, lost messages 0.00 % | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | AMD Ryzen 5 PRO 4650G | throughput | 100.0 | workstation | 2023-09-13 17:27:52 | ✋mean_benchmark 100.00 fps, rms_benchmark 100.01 fps, max_benchmark 113.54 fps, min_benchmark 89.33 fps, lost messages 0.00 % | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | AMD Ryzen 5 PRO 4650G | latency | 0.055 | workstation | 2023-09-13 17:29:00 | ✋mean_benchmark 0.0143 ms, rms_benchmark 0.0145 ms, max_benchmark 0.0550 ms, min_benchmark 0.0073 ms, lost messages 0.00 % | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | AMD Ryzen 5 PRO 4650G | power | 5.1217756271362305 | workstation | 2023-09-13 17:29:00 | ✋mean_benchmark 0.0143 ms, rms_benchmark 0.0145 ms, max_benchmark 0.0550 ms, min_benchmark 0.0073 ms, lost messages 0.00 % | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | AMD Ryzen 5 PRO 4650G | throughput | 100.01 | workstation | 2023-09-13 17:29:00 | ✋mean_benchmark 100.01 fps, rms_benchmark 100.01 fps, max_benchmark 115.25 fps, min_benchmark 88.33 fps, lost messages 0.00 % | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |

