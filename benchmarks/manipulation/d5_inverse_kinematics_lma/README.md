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
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-12700H | latency | 0.6636 | workstation | 08-09-2023 | ✋mean_benchmark 0.5268, rms_benchmark 0.5440, max_benchmark 0.6636, min_benchmark 0.3671, lost messages 0.00 % | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-12700H | power | 10.509076118469238 | workstation | 08-09-2023 | ✋ | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-8700K | latency | 5.7135 | workstation | 08-09-2023 | ✋mean_benchmark 2.8718, rms_benchmark 3.5445, max_benchmark 5.7135, min_benchmark 0.3558, lost messages 0.00 % | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-8700K | power | 7.025414943695068 | workstation | 08-09-2023 | ✋ | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | AMD Ryzen 5 PRO 4650G | latency | 0.9601 | workstation | 2023-09-13 17:39:17 | ✋mean_benchmark 0.4217 ms, rms_benchmark 0.4930 ms, max_benchmark 0.9601 ms, min_benchmark 0.1594 ms, lost messages 0.00 % | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | AMD Ryzen 5 PRO 4650G | power | 4.864711284637451 | workstation | 2023-09-13 17:39:17 | ✋mean_benchmark 0.4217 ms, rms_benchmark 0.4930 ms, max_benchmark 0.9601 ms, min_benchmark 0.1594 ms, lost messages 0.00 % | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | AMD Ryzen 5 PRO 4650G | latency | 1.7834 | workstation | 2023-09-13 17:41:21 | ✋mean_benchmark 0.6306 ms, rms_benchmark 0.8425 ms, max_benchmark 1.7834 ms, min_benchmark 0.1267 ms, lost messages 0.00 % | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | AMD Ryzen 5 PRO 4650G | power | 4.946023464202881 | workstation | 2023-09-13 17:41:21 | ✋mean_benchmark 0.6306 ms, rms_benchmark 0.8425 ms, max_benchmark 1.7834 ms, min_benchmark 0.1267 ms, lost messages 0.00 % | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |

