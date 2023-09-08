# m6_inverse_kinematics_lma

Meta computational graph composed by inverse kinematics computation for xArm6 using the LMA plugin.

### ID
m6

### Description
A meta computational graph that uses [moveit2](https://github.com/ros-planning/moveit2) to compute inverse kinematics with the [LMA](https://moveit.picknik.ai/main/doc/examples/kinematics_configuration/kinematics_configuration_tutorial.html#the-lma-kinematics-plugin) plugin for a UFACTORY xArm6 manipulator. Used to demonstrate a complex tranformation using the [geometry2](https://github.com/ros2/geometry2) package.

![](../../../imgs/d5_inverse_kinematics_lma.svg)

## Reproduction Steps

```bash
Refer to https://github.com/robotperf/benchmarks/tree/main/benchmarks/meta/m6_inverse_kinematics_lma and review the launch files to reproduce this package.
```

## Results

| Type | Hardware | Metric | Value | Category | Timestamp | Note | Data Source |
| --- | --- | --- | --- | --- | --- | --- | --- |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-12700H (ROBOTCORE Transforms) | latency | 0.0514 | workstation | 31-08-2023 | mean 0.0086 ms, RMS 0.0097 ms, max 0.0514 ms, min 0.0024 ms | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-12700H | latency | 0.3369 | workstation | 31-08-2023 | mean 0.0218 ms, RMS 0.0327 ms, max 0.3369 ms, min 0.0027 ms | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-8700K | latency | 0.1925 | workstation | 08-09-2023 | ✋mean_benchmark 0.0172, rms_benchmark 0.0223, max_benchmark 0.1925, min_benchmark 0.0022, lost messages 0.00 % | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-8700K | power | 7.0970540046691895 | workstation | 08-09-2023 | ✋ | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |

