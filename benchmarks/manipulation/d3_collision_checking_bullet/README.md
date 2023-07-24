# d3_collision_checking_bullet

xArm6 planning and trajectory execution

### ID
d3

### Description
A computational graph that uses [moveit2](https://github.com/ros-planning/moveit2) to compute collision checking with the [Bullet](https://github.com/bulletphysics/bullet3) library between a UFACTORY xArm6 manipulator and a cube-shaped collision object

![](../../../imgs/d3_collision_checking_bullet.svg)

## Reproduction Steps

```bash
Refer to https://github.com/robotperf/benchmarks/tree/main/benchmarks/manipulation/d3_collision_checking_bullet and review the launch files to reproduce this package.
```

## Results

| Type | Hardware | Metric | Value | Category | Timestamp | Note | Data Source |
| --- | --- | --- | --- | --- | --- | --- | --- |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-12700H | latency | 0.55 | workstation | 08-07-2023 | mean 0.08 ms, rms 0.10 ms, max 0.55 ms, min 0.02 ms, lost 0.00%, update rate 10 Hz | [N/A](https://github.com/robotperf/rosbags/tree/main/N/A) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-8700K | latency | 12340.02087 | workstation | 2023-07-20 15:04:32 | ✋mean_benchmark 8976.650255, rms_benchmark 9476.712474007052, max_benchmark 12340.02087, min_benchmark 4980.327211000001, lost messages 0.00 % | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-8700K | power | 6.039312362670898 | workstation | 2023-07-20 15:07:48 | ✋ | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-8700K | throughput | 0.0 | workstation | 2023-07-20 15:11:05 | ✋mean_benchmark 0.0, rms_benchmark 0.0, max_benchmark 0.0, min_benchmark 0.0, lost messages 0.00 % | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |

