# m4_collision_checking_bullet

Meta computational graph composed by collision checking between xArm6 and a box using Bullet library.

### ID
m4

### Description
A meta computational graph that uses [moveit2](https://github.com/ros-planning/moveit2) to compute collision checking with the [Bullet](https://github.com/bulletphysics/bullet3) library between a UFACTORY xArm6 manipulator and a cube-shaped collision object. Used to demonstrate a complex tranformation using the [geometry2](https://github.com/ros2/geometry2) package.

![](../../../imgs/d3_collision_checking_bullet.svg)

## Reproduction Steps

```bash
Refer to https://github.com/robotperf/benchmarks/tree/main/benchmarks/manipulation/m4_collision_checking_bullet and review the launch files to reproduce this package.
```

## Results

| Type | Hardware | Metric | Value | Category | Timestamp | Note | Data Source |
| --- | --- | --- | --- | --- | --- | --- | --- |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-12700H (ROBOTCORE Transforms) | latency | 0.039 | workstation | 31-08-2023 | mean 0.0066 ms, RMS 0.0077 ms, max 0.0390 ms, min 0.0025 ms | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-12700H | latency | 0.3502 | workstation | 31-08-2023 | mean 0.0207 ms, RMS 0.0432 ms, max 0.3502 ms, min 0.0029 ms | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-8700K | latency | 0.3759 | workstation | 08-09-2023 | ✋mean_benchmark 0.0166, rms_benchmark 0.0332, max_benchmark 0.3759, min_benchmark 0.0023, lost messages 0.00 % | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-8700K | power | 14.872563362121582 | workstation | 08-09-2023 | ✋ | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |

