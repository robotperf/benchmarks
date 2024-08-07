# d6_direct_kinematics

Direct kinematics computation for xArm6.

### ID
d6

### Description
A computational graph that uses [moveit2](https://github.com/ros-planning/moveit2) to compute direct kinematics for a UFACTORY xArm6 manipulator.

![](../../../imgs/d6_direct_kinematics.svg)

## Reproduction Steps

```bash
Refer to https://github.com/robotperf/benchmarks/tree/main/benchmarks/manipulation/d6_direct_kinematics and review the launch files to reproduce this package.
```

## Results

| Type | Hardware | Metric | Value | Category | Timestamp | Note | Data Source |
| --- | --- | --- | --- | --- | --- | --- | --- |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i5-13600K | latency | 0.01 | workstation | 28-07-2023 | mean 0.01 ms ms, rms 0.01 ms ms, max 0.01 ms ms, min 0.00 ms, lost 0.00% | [N/A](https://github.com/robotperf/rosbags/tree/main/N/A) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i5-13600K | power | 31.654253005981445 | workstation | 28-07-2023 | mean 0.01 ms ms, rms 0.01 ms ms, max 0.01 ms ms, min 0.00 ms, lost 0.00% | [N/A](https://github.com/robotperf/rosbags/tree/main/N/A) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-12700H | latency | 0.1237 | workstation | 08-09-2023 | ✋mean_benchmark 0.0115, rms_benchmark 0.0130, max_benchmark 0.1237, min_benchmark 0.0019, lost messages 0.00 % | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-12700H | power | 10.126541137695312 | workstation | 08-09-2023 | ✋ | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-8700K | latency | 0.0457 | workstation | 08-09-2023 | ✋mean_benchmark 0.0151, rms_benchmark 0.0200, max_benchmark 0.0457, min_benchmark 0.0044, lost messages 0.00 % | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-8700K | power | 8.938963890075684 | workstation | 08-09-2023 | ✋ | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | AMD Ryzen 5 PRO 4650G | latency | 0.0171 | workstation | 2023-09-13 17:41:21 | ✋mean_benchmark 0.0129 ms, rms_benchmark 0.0133 ms, max_benchmark 0.0171 ms, min_benchmark 0.0092 ms, lost messages 0.00 % | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | AMD Ryzen 5 PRO 4650G | power | 5.911809921264648 | workstation | 2023-09-13 17:41:21 | ✋mean_benchmark 0.0129 ms, rms_benchmark 0.0133 ms, max_benchmark 0.0171 ms, min_benchmark 0.0092 ms, lost messages 0.00 % | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | AMD Ryzen 5 PRO 4650G | latency | 0.0223 | workstation | 2023-09-13 17:43:44 | ✋mean_benchmark 0.0129 ms, rms_benchmark 0.0137 ms, max_benchmark 0.0223 ms, min_benchmark 0.0089 ms, lost messages 0.00 % | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | AMD Ryzen 5 PRO 4650G | power | 6.037580490112305 | workstation | 2023-09-13 17:43:44 | ✋mean_benchmark 0.0129 ms, rms_benchmark 0.0137 ms, max_benchmark 0.0223 ms, min_benchmark 0.0089 ms, lost messages 0.00 % | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |

