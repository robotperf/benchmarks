# m1_turtle_tf2_demo

Meta computational graph composed by two tf2 broadcaster and a tf2 listener.

### ID
m1

### Description
A simple meta computational graph composed by two transform broadcasters and a transform listener. Used to demonstrate a simple tranformation using the [geometry2](https://github.com/ros2/geometry2) package.

![](../../../imgs/m1_turtle_tf2_demo.png)

## Reproduction Steps

```bash
Refer to https://github.com/robotperf/benchmarks/tree/main/benchmarks/meta/m1_turtle_tf2_demo and review the launch files to reproduce this package.
```

## Results

| Type | Hardware | Metric | Value | Category | Timestamp | Note | Data Source |
| --- | --- | --- | --- | --- | --- | --- | --- |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-12700H (ROBOTCORE Transforms) | latency | 0.937 | workstation | 31-08-2023 | mean 0.0455 ms, RMS 0.0580 ms, max 0.9370 ms, min 0.0047 ms | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |

