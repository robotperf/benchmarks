# d7_dual_arm_static_avoidance

Controlling commands computation to avoid obstacles for two UFACTORY manipulators.

### ID
d7

### Description
A computational graph that uses [moveit2](https://github.com/ros-planning/moveit2) to move two UFACTORY manipulators so they avoid the obstacles detected by two depth cameras.

![](../../../imgs/d7_dual_arm_static_avoidance.svg)

## Reproduction Steps

```bash
Refer to https://github.com/robotperf/benchmarks/tree/main/benchmarks/manipulation/d7_dual_arm_static_avoidance and review the launch files to reproduce this package.
```

## Results

| Type | Hardware | Metric | Value | Category | Timestamp | Note | Data Source |
| --- | --- | --- | --- | --- | --- | --- | --- |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i5-13600K | latency | 0.01 | workstation | 28-07-2023 | mean 0.01 ms ms, rms 0.01 ms ms, max 0.01 ms ms, min 0.00 ms, lost 0.00% | [N/A](https://github.com/robotperf/rosbags/tree/main/N/A) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i5-13600K | power | 31.654253005981445 | workstation | 28-07-2023 | mean 0.01 ms ms, rms 0.01 ms ms, max 0.01 ms ms, min 0.00 ms, lost 0.00% | [N/A](https://github.com/robotperf/rosbags/tree/main/N/A) |

