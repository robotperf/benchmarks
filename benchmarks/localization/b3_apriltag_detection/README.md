# b3_apriltag_detection

Apriltag detection ROS Component.

### ID
b3

### Description
An april tag pose detection component.


![](../../../imgs/b3_apriltag_detection_node.svg)

## Reproduction Steps

```bash
Refer to https://github.com/robotperf/benchmarks/tree/main/benchmarks/localization/b3_apriltag_detection and review the launch files to reproduce this package.
```

## Results

| Type | Hardware | Metric | Value | Category | Timestamp | Note | Data Source |
| --- | --- | --- | --- | --- | --- | --- | --- |
| [:black_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-12700H + NVIDIA GeForce RTX 3060 (ROBOTCORE® Perception) | latency | 20.58 | workstation | 08-07-2023 | mean 16.32 ms, rms 16.37 ms, max 20.58 ms, min 12.52 ms, lost 0.00 %, throughput targeting 50 FPS | [r2b_dataset/r2b_storage](https://github.com/robotperf/rosbags/tree/main/r2b_dataset/r2b_storage) |
| [:black_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-12700H + NVIDIA GeForce RTX 3060 (ROBOTCORE® Perception) | throughput | 49.06 | workstation | 08-07-2023 | mean 16.32 ms, rms 16.37 ms, max 20.58 ms, min 12.52 ms, lost 0.00 %, throughput targeting 50 FPS | [r2b_dataset/r2b_storage](https://github.com/robotperf/rosbags/tree/main/r2b_dataset/r2b_storage) |
| [:black_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | NVIDIA AGX Orin Dev. Kit  (ROBOTCORE® Perception) | latency | 55.88 | edge | 08-07-2023 | mean 36.91 ms, rms 37.05 ms, max 55.88 ms, min 30.38 ms, lost 0.00 %, throughput targeting 10 FPS | [r2b_dataset/r2b_storage](https://github.com/robotperf/rosbags/tree/main/r2b_dataset/r2b_storage) |
| [:black_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | NVIDIA AGX Orin Dev. Kit (ROBOTCORE® Perception) | throughput | 10.203 | edge | 08-07-2023 | mean 36.91 ms, rms 37.05 ms, max 55.88 ms, min 30.38 ms, lost 0.00 %, throughput targeting 10 FPS | [r2b_dataset/r2b_storage](https://github.com/robotperf/rosbags/tree/main/r2b_dataset/r2b_storage) |
| [:black_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | NVIDIA AGX Orin Dev. Kit (ROBOTCORE® Perception) | power | 12.51 | edge | 08-07-2023 | mean 36.91 ms, rms 37.05 ms, max 55.88 ms, min 30.38 ms, lost 0.00 %, throughput targeting 10 FPS | [r2b_dataset/r2b_storage](https://github.com/robotperf/rosbags/tree/main/r2b_dataset/r2b_storage) |
| [:black_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i5-13600K + NVIDIA GeForce RTX 3060 (ROBOTCORE® Perception) | latency | 17.94 | workstation | 08-07-2023 | mean 9.16 ms, rms 9.21 ms, max 17.94 ms, min 6.58 ms, lost 0.00 %, throughput targeting 50 FPS | [r2b_dataset/r2b_storage](https://github.com/robotperf/rosbags/tree/main/r2b_dataset/r2b_storage) |
| [:black_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i5-13600K + NVIDIA GeForce RTX 3060 (ROBOTCORE® Perception) | throughput | 49.09 | workstation | 08-07-2023 | mean 9.16 ms, rms 9.21 ms, max 17.94 ms, min 6.58 ms, lost 0.00 %, throughput targeting 50 FPS | [r2b_dataset/r2b_storage](https://github.com/robotperf/rosbags/tree/main/r2b_dataset/r2b_storage) |

