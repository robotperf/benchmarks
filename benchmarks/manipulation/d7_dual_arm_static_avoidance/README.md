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
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i5-13600K | latency | 0.01 | workstation | 08-09-2023 | mean 121.6300 ms, rms 121.6337 ms, max 122.5769 ms, min 120.6832 ms, lost 0.00% (planning) | [N/A](https://github.com/robotperf/rosbags/tree/main/N/A) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i5-13600K | latency | 0.01 | workstation | 08-09-2023 | mean 0.0499  ms, rms 0.1060 ms, max 2.3015 ms, min 0.0049 ms, lost 0.00% (collision checking with FCL) | [N/A](https://github.com/robotperf/rosbags/tree/main/N/A) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i5-13600K | latency | 0.01 | workstation | 08-09-2023 | mean 0.0141  ms, rms 0.0227 ms, max 0.2057 ms, min 0.0010  ms, lost 0.00% (direct kinematics) | [N/A](https://github.com/robotperf/rosbags/tree/main/N/A) |

