# m2_xarm6_planning_and_traj_execution

Meta computational graph composed by xArm6 planning and trajectory execution

### ID
m2

### Description
A meta computational graph that uses [moveit2](https://github.com/ros-planning/moveit2) to compute motion planning and perform trajectory execution on a UFACTORY xArm6 manipulator. Used to demonstrate a complex tranformation using the [geometry2](https://github.com/ros2/geometry2) package.


![](../../../imgs/d1_xarm6_planning_and_traj_execution.svg)

## Reproduction Steps

```bash
Refer to https://github.com/robotperf/benchmarks/tree/main/benchmarks/meta/m2_xarm6_planning_and_traj_execution and review the launch files to reproduce this package.
```

## Results

| Type | Hardware | Metric | Value | Category | Timestamp | Note | Data Source |
| --- | --- | --- | --- | --- | --- | --- | --- |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-12700H (ROBOTCORE Transforms) | latency | 0.0416 | workstation | 31-08-2023 | mean 0.0092 ms, RMS 0.0104 ms, max 0.0416 ms, min 0.0024 ms | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |

