# n1_intra_process

Local computational graph composed by two components.

### ID
n1

### Description
A simple network computational graph composed by two components. Used to demonstrate a simple ping-pong for intra-process communication.

![](../../../imgs/n1_intra_process.png)

## Reproduction Steps

```bash
Refer to https://github.com/robotperf/benchmarks/tree/main/benchmarks/network/n1_intra_process and review the launch files to reproduce this package.
```

## Results

| Type | Hardware | Metric | Value | Category | Timestamp | Note | Data Source |
| --- | --- | --- | --- | --- | --- | --- | --- |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i5-13600K | latency | 0.0172 | workstation - workstation | 31-10-2023 | median 0.0172 ms, mean 0.0218 ms, 1000000 samples, 0.00 % lost messages, Fast DDS | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel Agilex® 7 FPGA F-Series | latency | 0.201 | embedded - embedded | 31-10-2023 | median 0.2010 ms, mean 0.2059 ms, 998582 samples, 0.00 % lost messages, Fast DDS | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-12700H | latency | 0.0687 | workstation - workstation | 07-11-2023 | median 0.0687 ms, mean 0.1000 ms, 1000000 samples, 0.00 % lost messages, Fast DDS | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-12700H | latency | 0.0902 | workstation - workstation | 07-11-2023 | median 0.0902 ms, mean 0.0981 ms, 1000000 samples, 0.00 % lost messages, Cyclone DDS | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-12700H | latency | 0.085 | workstation - workstation | 07-11-2023 | median 0.0850 ms, mean 0.1044 ms, 1000000 samples, 0.00 % lost messages, Gurum DDS | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-12700H | latency | 0.0633 | workstation - workstation | 07-11-2023 | median 0.0633 ms, mean 0.0944 ms, 1000000 samples, 0.00 % lost messages, Connext DDS | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | AMD Ryzen 5 PRO 4650G | latency | 0.146 | workstation - workstation | 07-11-2023 | median 0.1460 ms, mean 0.1409 ms, 1000000 samples, 0.00 % lost messages, Connext DDS | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |

