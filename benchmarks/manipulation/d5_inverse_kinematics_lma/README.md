# d5_inverse_kinematics_lma

Inverse kinematics computation for xArm6 using the LMA plugin.

### ID
d5

### Description
A computational graph that uses [moveit2](https://github.com/ros-planning/moveit2) to compute inverse kinematics with the [LMA](https://moveit.picknik.ai/main/doc/examples/kinematics_configuration/kinematics_configuration_tutorial.html#the-lma-kinematics-plugin) plugin for a UFACTORY xArm6 manipulator.

![](../../../imgs/d5_inverse_kinematics_lma.svg)

## Reproduction Steps

```bash
Refer to https://github.com/robotperf/benchmarks/tree/main/benchmarks/manipulation/d5_inverse_kinematics_lma and review the launch files to reproduce this package.
```

## Results

| Type | Hardware | Metric | Value | Category | Timestamp | Note | Data Source |
| --- | --- | --- | --- | --- | --- | --- | --- |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i5-13600K | latency | 1.36 | workstation | 27-07-2023 | mean 0.38 ms, rms 0.55 ms, max 1.36 ms, min 0.08 ms, lost 0.00% | [N/A](https://github.com/robotperf/rosbags/tree/main/N/A) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i5-13600K | power | 43.88430404663086 | workstation | 27-07-2023 | mean 0.38 ms, rms 0.55 ms, max 1.36 ms, min 0.08 ms, lost 0.00% | [N/A](https://github.com/robotperf/rosbags/tree/main/N/A) |

