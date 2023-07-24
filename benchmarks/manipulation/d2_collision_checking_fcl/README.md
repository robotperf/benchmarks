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
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-12700H | latency | 0.55 | workstation | 08-07-2023 | mean 0.08 ms, rms 0.10 ms, max 0.55 ms, min 0.02 ms, lost 0.00%, update rate 10 Hz | [N/A](https://github.com/robotperf/rosbags/tree/main/N/A) |

