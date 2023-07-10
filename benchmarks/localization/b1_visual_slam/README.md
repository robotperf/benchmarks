# b1_visual_slam

Visual SLAM ROS Component.

### ID
b1

### Description
A SLAM localization based on visual input, and optionally other sensors such as IMUs. Used to demonstrate the visual SLAM components.


![](../../../imgs/b1_visual_slam_stella.svg)

## Reproduction Steps

```bash
Refer to https://github.com/robotperf/benchmarks/tree/main/benchmarks/localization/b1_visual_slam and review the launch files to reproduce this package.
```

## Results

| Type | Hardware | Metric | Value | Category | Timestamp | Note | Data Source |
| --- | --- | --- | --- | --- | --- | --- | --- |
| [:black_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-12700H + NVIDIA GeForce RTX 3060 | latency | 9.71 | workstation | 08-07-2023 | mean 4.85 ms, rms 5.06 ms, max 9.71 ms, min 1.57 ms, lost 0.00 %, throughput targeting 10 FPS | [r2b_dataset/r2b_storage](https://github.com/robotperf/rosbags/tree/main/r2b_dataset/r2b_storage) |
| [:black_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-12700H + NVIDIA GeForce RTX 3060 | throughput | 10.0 | workstation | 08-07-2023 | mean 4.85 ms, rms 5.06 ms, max 9.71 ms, min 1.57 ms, lost 0.00 %, throughput targeting 10 FPS | [r2b_dataset/r2b_storage](https://github.com/robotperf/rosbags/tree/main/r2b_dataset/r2b_storage) |
| [:black_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | NVIDIA AGX Orin Dev. Kit (ROBOTCORE® Perception) | latency | 19.78 | edge | 08-07-2023 | mean 9.97 ms, rms 10.38 ms, max 19.78 ms, min 4.24 ms, lost 0.00 %, throughput targeting 10 FPS | [r2b_dataset/r2b_storage](https://github.com/robotperf/rosbags/tree/main/r2b_dataset/r2b_storage) |
| [:black_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | NVIDIA AGX Orin Dev. Kit (ROBOTCORE® Perception) | throughput | 10.203 | edge | 08-07-2023 | mean 9.97 ms, rms 10.38 ms, max 19.78 ms, min 4.24 ms, lost 0.00 %, throughput targeting 10 FPS | [r2b_dataset/r2b_storage](https://github.com/robotperf/rosbags/tree/main/r2b_dataset/r2b_storage) |
| [:black_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-12700H + NVIDIA GeForce RTX 3060 (ROBOTCORE® Perception) | latency | 33.34 | workstation | 08-07-2023 | mean 14.80 ms, rms 18.10 ms, max 33.34 ms, min 1.30 ms, lost 0.41 %, throughput targeting 50 FPS | [r2b_dataset/r2b_storage](https://github.com/robotperf/rosbags/tree/main/r2b_dataset/r2b_storage) |
| [:black_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-12700H + NVIDIA GeForce RTX 3060 (ROBOTCORE® Perception) | throughput | 48.972 | workstation | 08-07-2023 | mean 14.80 ms, rms 18.10 ms, max 33.34 ms, min 1.30 ms, lost 0.41 %, throughput targeting 50 FPS | [r2b_dataset/r2b_storage](https://github.com/robotperf/rosbags/tree/main/r2b_dataset/r2b_storage) |
| [:black_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-12700H + NVIDIA GeForce RTX 3060 (ROBOTCORE® Perception) | power | 31.56 | workstation | 08-07-2023 | mean 14.80 ms, rms 18.10 ms, max 33.34 ms, min 1.30 ms, lost 0.41 %, throughput targeting 50 FPS | [r2b_dataset/r2b_storage](https://github.com/robotperf/rosbags/tree/main/r2b_dataset/r2b_storage) |
| [:black_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i5-13600K + NVIDIA GeForce RTX 3060 (ROBOTCORE® Perception) | latency | 28.97 | workstation | 08-07-2023 | mean 12.20 ms, rms 15.95 ms, max 28.97 ms, min 0.82 ms lost 0.41 %, , throughput targeting 50 FPS | [r2b_dataset/r2b_storage](https://github.com/robotperf/rosbags/tree/main/r2b_dataset/r2b_storage) |
| [:black_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i5-13600K + NVIDIA GeForce RTX 3060 (ROBOTCORE® Perception) | throughput | 48.986 | workstation | 08-07-2023 | mean 12.20 ms, rms 15.95 ms, max 28.97 ms, min 0.82 ms lost 0.41 %, throughput targeting 50 FPS | [r2b_dataset/r2b_storage](https://github.com/robotperf/rosbags/tree/main/r2b_dataset/r2b_storage) |

