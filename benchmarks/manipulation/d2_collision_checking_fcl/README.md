# d2_collision_checking_fcl

Collision checking between xArm6 and a box using FCL library.

### ID
d2

### Description
A computational graph that uses [moveit2](https://github.com/ros-planning/moveit2) to compute collision checking with the [FCL](https://github.com/flexible-collision-library/fcl) library between a UFACTORY xArm6 manipulator and a cube-shaped collision object

![](../../../imgs/d2_collision_checking_fcl.svg)

## Reproduction Steps

```bash
Refer to https://github.com/robotperf/benchmarks/tree/main/benchmarks/manipulation/d2_collision_checking_fcl and review the launch files to reproduce this package.
```

## Results

| Type | Hardware | Metric | Value | Category | Timestamp | Note | Data Source |
| --- | --- | --- | --- | --- | --- | --- | --- |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i5-13600K | latency | 3.0874 | workstation | 2023-09-11 09:20:32 | ✋mean_benchmark 1.8055, rms_benchmark 2.0244, max_benchmark 3.0874, min_benchmark 1.0066, lost messages 0.00 % | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i5-13600K | power | 81.74083709716797 | workstation | 2023-09-11 09:20:32 | ✋ | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i5-13600K | throughput | 0.0 | workstation | 2023-09-11 09:20:32 | ✋mean_benchmark 0.0, rms_benchmark 0.0, max_benchmark 0.0, min_benchmark 0.0, lost messages 0.00 % | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-12700H | latency | 10.9649 | workstation | 08-09-2023 | ✋mean_benchmark 9.1100, rms_benchmark 9.3270, max_benchmark 10.9649, min_benchmark 6.3328, lost messages 0.00 % | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-12700H | power | 9.9816255569458 | workstation | 08-09-2023 | ✋ | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-8700K | latency | 23.1109 | workstation | 08-09-2023 | ✋mean_benchmark 12.0303, rms_benchmark 14.5932, max_benchmark 23.1109, min_benchmark 3.2861, lost messages 0.00 % | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-8700K | power | 13.847026824951172 | workstation | 08-09-2023 | ✋ | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | AMD Ryzen 5 PRO 4650G | latency | 5.6565 | workstation | 2023-09-13 17:34:52 | ✋mean_benchmark 3.8750 ms, rms_benchmark 4.1162 ms, max_benchmark 5.6565 ms, min_benchmark 2.2694 ms, lost messages 0.00 % | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | AMD Ryzen 5 PRO 4650G | power | 11.578874588012695 | workstation | 2023-09-13 17:34:52 | ✋mean_benchmark 3.8750 ms, rms_benchmark 4.1162 ms, max_benchmark 5.6565 ms, min_benchmark 2.2694 ms, lost messages 0.00 % | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | AMD Ryzen 5 PRO 4650G | latency | 6.7518 | workstation | 2023-09-13 17:36:03 | ✋mean_benchmark 3.6152 ms, rms_benchmark 4.2670 ms, max_benchmark 6.7518 ms, min_benchmark 1.4750 ms, lost messages 0.00 % | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | AMD Ryzen 5 PRO 4650G | power | 10.524511337280273 | workstation | 2023-09-13 17:36:03 | ✋mean_benchmark 3.6152 ms, rms_benchmark 4.2670 ms, max_benchmark 6.7518 ms, min_benchmark 1.4750 ms, lost messages 0.00 % | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |

