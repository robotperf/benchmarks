# d3_collision_checking_bullet

Collision checking between xArm6 and a box using Bullet library.

### ID
d3

### Description
A computational graph that uses [moveit2](https://github.com/ros-planning/moveit2) to compute collision checking with the [Bullet](https://github.com/bulletphysics/bullet3) library between a UFACTORY xArm6 manipulator and a cube-shaped collision object

![](../../../imgs/d3_collision_checking_bullet.svg)

## Reproduction Steps

```bash
Refer to https://github.com/robotperf/benchmarks/tree/main/benchmarks/manipulation/d3_collision_checking_bullet and review the launch files to reproduce this package.
```

## Results

| Type | Hardware | Metric | Value | Category | Timestamp | Note | Data Source |
| --- | --- | --- | --- | --- | --- | --- | --- |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-12700H | latency | 2.5722 | workstation | 08-09-2023 | ✋mean_benchmark 1.6704, rms_benchmark 1.7885, max_benchmark 2.5722, min_benchmark 1.1654, lost messages 0.00 % | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-12700H | power | 10.084168434143066 | workstation | 08-09-2023 | ✋ | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-8700K | latency | 2.3716 | workstation | 08-09-2023 | ✋mean_benchmark 1.8770, rms_benchmark 1.9093, max_benchmark 2.3716, min_benchmark 1.6177, lost messages 0.00 % | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-8700K | power | 14.359254837036133 | workstation | 08-09-2023 | ✋ | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |

