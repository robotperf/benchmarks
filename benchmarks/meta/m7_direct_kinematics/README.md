# m7_direct_kinematics

Meta computational graph composed by direct kinematics computation for xArm6.

### ID
m7

### Description
A meta computational graph that uses [moveit2](https://github.com/ros-planning/moveit2) to compute direct kinematics for a UFACTORY xArm6 manipulator. Used to demonstrate a complex tranformation using the [geometry2](https://github.com/ros2/geometry2) package.

![](../../../imgs/d6_direct_kinematics.svg)

## Reproduction Steps

```bash
Refer to https://github.com/robotperf/benchmarks/tree/main/benchmarks/meta/m7_direct_kinematics and review the launch files to reproduce this package.
```

## Results

| Type | Hardware | Metric | Value | Category | Timestamp | Note | Data Source |
| --- | --- | --- | --- | --- | --- | --- | --- |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-12700H (ROBOTCORE Transforms) | latency | 0.0226 | workstation | 31-08-2023 | mean 0.0063 ms, RMS 0.0072 ms, max 0.0226 ms, min 0.0014 ms | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-12700H | latency | 0.3157 | workstation | 31-08-2023 | mean 0.0186 ms, RMS 0.0319 ms, max 0.3157 ms, min 0.0027 ms | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-8700K | latency | 0.1925 | workstation | 08-09-2023 | ✋mean_benchmark 0.0166, rms_benchmark 0.0306, max_benchmark 0.5770, min_benchmark 0.0015, lost messages 0.00 % | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-8700K | power | 8.938907623291016 | workstation | 08-09-2023 | ✋ | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |

