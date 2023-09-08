# m3_collision_checking_fcl

Meta computational graph composed by collision checking between xArm6 and a box using FCL library.

### ID
m3

### Description
A meta computational graph that uses [moveit2](https://github.com/ros-planning/moveit2) to compute collision checking with the [FCL](https://github.com/flexible-collision-library/fcl) library between a UFACTORY xArm6 manipulator and a cube-shaped collision object. Used to demonstrate a complex tranformation using the [geometry2](https://github.com/ros2/geometry2) package.

![](../../../imgs/d2_collision_checking_fcl.svg)

## Reproduction Steps

```bash
Refer to https://github.com/robotperf/benchmarks/tree/main/benchmarks/manipulation/d2_collision_checking_fcl and review the launch files to reproduce this package.
```

## Results

| Type | Hardware | Metric | Value | Category | Timestamp | Note | Data Source |
| --- | --- | --- | --- | --- | --- | --- | --- |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-12700H (ROBOTCORE Transforms) | latency | 0.0407 | workstation | 31-08-2023 | mean 0.0075 ms, RMS 0.0090 ms, max 0.0407 ms, min 0.0034 ms | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-12700H | latency | 0.5183 | workstation | 31-08-2023 | mean 0.0248 ms, RMS 0.0650 ms, max 0.5183 ms, min 0.0053 ms | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-8700K | latency | 0.1739 | workstation | 08-09-2023 | ✋mean_benchmark 0.0179, rms_benchmark 0.0285, max_benchmark 0.1739, min_benchmark 0.0023, lost messages 0.00 % | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-8700K | power | 13.846317291259766 | workstation | 08-09-2023 | ✋ | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |

