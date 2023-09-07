# m5_inverse_kinematics_kdl

Meta computational graph composed by inverse kinematics computation for xArm6 using the KDL plugin.

### ID
m5

### Description
A meta computational graph that uses [moveit2](https://github.com/ros-planning/moveit2) to compute inverse kinematics with the [KDL](https://moveit.picknik.ai/main/doc/examples/kinematics_configuration/kinematics_configuration_tutorial.html#the-kdl-kinematics-plugin) plugin for a UFACTORY xArm6 manipulator. Used to demonstrate a complex tranformation using the [geometry2](https://github.com/ros2/geometry2) package.

![](../../../imgs/d4_inverse_kinematics_kdl.svg)

## Reproduction Steps

```bash
Refer to https://github.com/robotperf/benchmarks/tree/main/benchmarks/meta/m5_inverse_kinematics_kdl and review the launch files to reproduce this package.
```

## Results

| Type | Hardware | Metric | Value | Category | Timestamp | Note | Data Source |
| --- | --- | --- | --- | --- | --- | --- | --- |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-12700H (ROBOTCORE Transforms) | latency | 0.0388 | workstation | 31-08-2023 | mean 0.0082 ms, RMS 0.0092 ms, max 0.0388 ms, min 0.0023 ms | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-12700H | latency | 0.4492 | workstation | 31-08-2023 | mean 0.0224 ms, RMS 0.0330 ms, max 0.4492 ms, min 0.0029 ms | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |

