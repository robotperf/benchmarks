# n2_inter_process

Local computational graph composed by two nodes.

### ID
n2

### Description
A simple local computational graph composed by two nodes. Used to demonstrate a simple ping-pong for inter-process communication.

![](../../../imgs/n2_inter_process.png)

## Reproduction Steps

```bash
Refer to https://github.com/robotperf/benchmarks/tree/main/benchmarks/network/n2_inter_process and review the launch files to reproduce this package.
```

## Results

| Type | Hardware | Metric | Value | Category | Timestamp | Note | Data Source |
| --- | --- | --- | --- | --- | --- | --- | --- |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i5-13600K | latency | 0.0283 | workstation - workstation | 31-10-2023 | median 0.0283 ms, mean 0.0289 ms, 1000000 samples, 0.00 % lost messages, Fast DDS | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel Agilex® 7 FPGA F-Series | latency | 0.3801 | embedded - embedded | 31-10-2023 | median 0.3801 ms, mean 0.4285 ms, 998222 samples, 0.00 % lost messages, Fast DDS | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-12700H | latency | 0.0854 | workstation - workstation | 07-11-2023 | median 0.0854 ms, mean 0.0919 ms, 1000000 samples, 0.00 % lost messages, Fast DDS | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-12700H | latency | 0.0714 | workstation - workstation | 07-11-2023 | median 0.0714 ms, mean 0.0769 ms, 1000000 samples, 0.00 % lost messages, Cyclone DDS | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-12700H | latency | 0.0566 | workstation - workstation | 07-11-2023 | median 0.0566 ms, mean 0.0598 ms, 9145 samples, 0.00 % lost messages, Ecal Dynamic DDS | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-12700H | latency | 0.1216 | workstation - workstation | 07-11-2023 | median 0.1216 ms, mean 0.1298 ms, 1000000 samples, 0.00 % lost messages, Gurum DDS | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-12700H | latency | 0.134 | workstation - workstation | 07-11-2023 | median 0.1340 ms, mean 0.1654 ms, 1000000 samples, 0.00 % lost messages, Connext DDS | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | AMD Ryzen 5 PRO 4650G | latency | 0.2929 | workstation - workstation | 07-11-2023 | median 0.2929 ms, mean 0.3160 ms, 1000000 samples, 0.00 % lost messages, Connext DDS | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-8700K | latency | 0.1098 | workstation - workstation | 07-11-2023 | median 0.1098 ms, mean 0.1234 ms, 998676 samples, 0.00 % lost messages, Connext DDS | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |

