# d4_inverse_kinematics_kdl

Inverse kinematics computation for xArm6 using the KDL plugin.

### ID
d4

### Description
A computational graph that uses [moveit2](https://github.com/ros-planning/moveit2) to compute inverse kinematics with the [KDL](https://moveit.picknik.ai/main/doc/examples/kinematics_configuration/kinematics_configuration_tutorial.html#the-kdl-kinematics-plugin) plugin for a UFACTORY xArm6 manipulator.

![](../../../imgs/d4_inverse_kinematics_kdl.svg)

## Reproduction Steps

```bash
Refer to https://github.com/robotperf/benchmarks/tree/main/benchmarks/manipulation/d4_inverse_kinematics_kdl and review the launch files to reproduce this package.
```

## Results

| Type | Hardware | Metric | Value | Category | Timestamp | Note | Data Source |
| --- | --- | --- | --- | --- | --- | --- | --- |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i5-13600K | latency | 4.41 | workstation | 27-07-2023 | mean 1.68 ms, rms 2.56 ms, max 4.41 ms, min 0.15 ms, lost 0.00% | [N/A](https://github.com/robotperf/rosbags/tree/main/N/A) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i5-13600K | power | 48.451541900634766 | workstation | 27-07-2023 | mean 1.68 ms, rms 2.56 ms, max 4.41 ms, min 0.15 ms, lost 0.00% | [N/A](https://github.com/robotperf/rosbags/tree/main/N/A) |

