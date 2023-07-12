# b2_map_localization

Map localization ROS Component.

### ID
b2

### Description
Benchmarks for map localization ROS robotics operation. Used to demonstrate occupancy grid localization packages..


![](../../../imgs/b2_map_localization.png)

## Reproduction Steps

```bash
Refer to https://github.com/robotperf/benchmarks/tree/main/benchmarks/localization/b2_map_localization and review the launch files to reproduce this package.
```

## Results

| Type | Hardware | Metric | Value | Category | Timestamp | Note | Data Source |
| --- | --- | --- | --- | --- | --- | --- | --- |
| [:black_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-12700H + NVIDIA GeForce RTX 3060 (ROBOTCORE® Perception) | latency | 57.48 | workstation | 08-07-2023 | mean 40.40 ms, rms 40.55 ms, max 57.48 ms, min 23.60 ms 0.00 %, throughput targeting 30 FPS | [r2b_dataset/r2b_storage](https://github.com/robotperf/rosbags/tree/main/r2b_dataset/r2b_storage) |
| [:black_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-12700H + NVIDIA GeForce RTX 3060 (ROBOTCORE® Perception) | throughput | 30.0 | workstation | 08-07-2023 | mean 40.40 ms, rms 40.55 ms, max 57.48 ms, min 23.60 ms 0.00 % | [r2b_dataset/r2b_storage](https://github.com/robotperf/rosbags/tree/main/r2b_dataset/r2b_storage) |
| [:black_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | NVIDIA AGX Orin Dev. Kit  (ROBOTCORE® Perception) | latency | 143.01 | edge | 08-07-2023 | mean 78.35 ms, rms 79.35 ms, max 143.01 ms, min 67.59 ms 0.00 % | [r2b_dataset/r2b_storage](https://github.com/robotperf/rosbags/tree/main/r2b_dataset/r2b_storage) |
| [:black_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | NVIDIA AGX Orin Dev. Kit (ROBOTCORE® Perception) | throughput | 13.54 | edge | 08-07-2023 | mean 78.35 ms, rms 79.35 ms, max 143.01 ms, min 67.59 ms 0.00 % | [r2b_dataset/r2b_storage](https://github.com/robotperf/rosbags/tree/main/r2b_dataset/r2b_storage) |
| [:black_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | NVIDIA AGX Orin Dev. Kit (ROBOTCORE® Perception) | power | 19.68 | edge | 08-07-2023 | mean 78.35 ms, rms 79.35 ms, max 143.01 ms, min 67.59 ms 0.00 % | [r2b_dataset/r2b_storage](https://github.com/robotperf/rosbags/tree/main/r2b_dataset/r2b_storage) |
| [:black_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-12700H + NVIDIA GeForce RTX 3060 (ROBOTCORE® Perception) | latency | 98.39 | workstation | 08-07-2023 | mean 65.77 ms, rms 66.39 ms, max 98.39 ms, min 28.81 ms, lost 0.48 % | [r2b_dataset/r2b_storage](https://github.com/robotperf/rosbags/tree/main/r2b_dataset/r2b_storage) |
| [:black_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-12700H + NVIDIA GeForce RTX 3060 (ROBOTCORE® Perception) | throughput | 41.58 | workstation | 08-07-2023 | mean 65.77 ms, rms 66.39 ms, max 98.39 ms, min 28.81 ms, lost 0.48 % | [r2b_dataset/r2b_storage](https://github.com/robotperf/rosbags/tree/main/r2b_dataset/r2b_storage) |
| [:black_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-12700H + NVIDIA GeForce RTX 3060 (ROBOTCORE® Perception) | power | 106.78 | workstation | 08-07-2023 | mean 65.77 ms, rms 66.39 ms, max 98.39 ms, min 28.81 ms, lost 0.48 % | [r2b_dataset/r2b_storage](https://github.com/robotperf/rosbags/tree/main/r2b_dataset/r2b_storage) |

