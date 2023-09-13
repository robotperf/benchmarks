# d1_xarm6_planning_and_traj_execution

xArm6 planning and trajectory execution

### ID
d1

### Description
A computational graph that uses [moveit2](https://github.com/ros-planning/moveit2) to compute motion planning and perform trajectory execution on a UFACTORY xArm6 manipulator


![](../../../imgs/d1_xarm6_planning_and_traj_execution.svg)

## Reproduction Steps

```bash
Refer to https://github.com/robotperf/benchmarks/tree/main/benchmarks/manipulation/d1_xarm6_planning_and_traj_execution and review the launch files to reproduce this package.
```

## Results

| Type | Hardware | Metric | Value | Category | Timestamp | Note | Data Source |
| --- | --- | --- | --- | --- | --- | --- | --- |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-8700K | latency | 12340.02087 | workstation | 2023-07-20 15:04:32 | ✋mean_benchmark 8976.650255, rms_benchmark 9476.712474007052, max_benchmark 12340.02087, min_benchmark 4980.327211000001, lost messages 0.00 % | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-8700K | power | 6.039312362670898 | workstation | 2023-07-20 15:07:48 | ✋ | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-12700H | latency | 13333.1143 | workstation | 08-09-2023 | ✋mean_benchmark 13328.2834, rms_benchmark 13328.2843, max_benchmark 13333.1143, min_benchmark 13323.4525, lost messages 0.00 % | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i5-13600K | latency | 12279.9022 | workstation | 2023-09-11 09:20:32 | ✋mean_benchmark 8953.2444, rms_benchmark 9444.0619, max_benchmark 12279.9022, min_benchmark 4999.8081, lost messages 0.00 % | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i5-13600K | power | 76.66064453125 | workstation | 2023-09-11 09:20:32 | ✋ | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i5-13600K | throughput | 0.0 | workstation | 2023-09-11 09:20:32 | ✋mean_benchmark 0.0, rms_benchmark 0.0, max_benchmark 0.0, min_benchmark 0.0, lost messages 0.00 % | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-12700H | power | 10.07455062866211 | workstation | 08-09-2023 | ✋ | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-8700K | latency | 12347.8759 | workstation | 08-09-2023 | ✋mean_benchmark 9009.1484, rms_benchmark 9504.0478, max_benchmark 12347.8759, min_benchmark 5019.6707, lost messages 0.00 % | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-8700K | power | 6.190027713775635 | workstation | 08-09-2023 | ✋ | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | AMD Ryzen 5 PRO 4650G | latency | 12379.8178 | workstation | 2023-09-13 17:27:52 | ✋mean_benchmark 9029.3716 ms, rms_benchmark 9525.2828 ms, max_benchmark 12379.8178 ms, min_benchmark 5034.0037 ms, lost messages 0.00 % | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | AMD Ryzen 5 PRO 4650G | power | 4.555608749389648 | workstation | 2023-09-13 17:27:52 | ✋mean_benchmark 9029.3716 ms, rms_benchmark 9525.2828 ms, max_benchmark 12379.8178 ms, min_benchmark 5034.0037 ms, lost messages 0.00 % | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | AMD Ryzen 5 PRO 4650G | latency | 12379.7598 | workstation | 2023-09-13 17:34:52 | ✋mean_benchmark 9026.4061 ms, rms_benchmark 9521.3573 ms, max_benchmark 12379.7598 ms, min_benchmark 5039.6164 ms, lost messages 0.00 % | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | AMD Ryzen 5 PRO 4650G | power | 4.490983963012695 | workstation | 2023-09-13 17:34:52 | ✋mean_benchmark 9026.4061 ms, rms_benchmark 9521.3573 ms, max_benchmark 12379.7598 ms, min_benchmark 5039.6164 ms, lost messages 0.00 % | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |

