# n4_intra_network_security

Network computational graph composed by two nodes.

### ID
n4

### Description
A simple network computational graph composed by two nodes. Used to demonstrate a simple ping-pong for intra-network communication.

![](../../../imgs/n4_intra_network_security.png)

## Reproduction Steps

```bash
Refer to https://github.com/robotperf/benchmarks/tree/main/benchmarks/network/n4_intra_network_security and review the launch files to reproduce this package.
```

## Results

@todo

| Type | Hardware | Metric | Value | Category | Timestamp | Note | Data Source |
| --- | --- | --- | --- | --- | --- | --- | --- |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i5-8250U <-> Intel i5-13600K | latency | 0.4243 | workstation - workstation | 31-10-2023 | median 0.4243 ms, mean 0.4307 ms, 1000000 samples, 0.00 % lost messages, Fast DDS | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i5-13600K <-> Intel Agilex® 7 | latency | 0.4457 | workstation - embedded | 31-10-2023 | median 0.4457 ms, mean 0.4534 ms, 1000000 samples, 0.00 % lost messages, Fast DDS | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i5-13600K <-> Intel Agilex® 7 | latency | 0.4039 | workstation - embedded | 31-10-2023 | median 0.4039 ms, mean 0.4095 ms, 1000000 samples, 0.00 % lost messages, Cyclone DDS | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i5-13600K <-> Intel Agilex® 7 (ROBOTCORE® ROS 2) | latency | 0.0285 | workstation - embedded | 31-10-2023 | median 0.0285 ms, mean 0.1028 ms, 1000000 samples, 0.00 % lost messages, Cyclone DDS | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i5-13600K <-> Intel Agilex® 7 (ROBOTCORE® ROS 2) <-> HPS | latency | 0.0411 | workstation - embedded | 31-10-2023 | median 0.0411 ms, mean 0.1035 ms, 1000000 samples, 0.00 % lost messages, Cyclone DDS | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel Agilex® 7 (ROBOTCORE® ROS 2) <-> Intel Agilex® 7 (ROBOTCORE® ROS 2) | latency | 0.0049 | embedded - embedded | 31-10-2023 | median 0.0049 ms, mean 0.0056 ms, 1000000 samples, 0.00 % lost messages, Cyclone DDS | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i5-13600K <->  Intel i5-8250U | latency | 0.3797 | workstation - workstation | 07-11-2023 | median 0.3797 ms, mean 0.4721 ms, 1000000 samples, 0.00 % lost messages, Ecal Dynamic DDS | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i5-8250U <-> Intel i5-13600K | latency | 0.3661 | workstation - workstation | 07-11-2023 | median 0.3661 ms, mean 0.3767 ms, 1000000 samples, 0.00 % lost messages, Ecal Dynamic DDS | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-8700K <-> AMD Ryzen 5 PRO 4650G | latency | 0.5469 | workstation - workstation | 07-11-2023 | median 0.5469 ms, mean 0.5465 ms, 780061 samples, 0.00 % lost messages, Connext DDS | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | AMD Ryzen 5 PRO 4650G <-> Intel i7-8700K | latency | 1.1165 | workstation - workstation | 07-11-2023 | median 1.1165 ms, mean 1.0618 ms, 1000000 samples, 0.00 % lost messages, Connext DDS | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-8700K <-> AMD Ryzen 5 PRO 4650G | latency | 0.3312 | workstation - workstation | 07-11-2023 | median 0.3312 ms, mean 0.3390 ms, 780061 samples, 0.00 % lost messages, Fast DDS | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | AMD Ryzen 5 PRO 4650G <-> Intel i7-8700K | latency | 0.2925 | workstation - workstation | 07-11-2023 | median 0.2925 ms, mean 0.2984 ms, 1000000 samples, 0.00 % lost messages, Fast DDS | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | Intel i7-8700K <-> AMD Ryzen 5 PRO 4650G | latency | 0.3219 | workstation - workstation | 07-11-2023 | median 0.3219 ms, mean 0.3283 ms, 780061 samples, 0.00 % lost messages, Cyclone DDS | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |
| [:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type) | AMD Ryzen 5 PRO 4650G <-> Intel i7-8700K | latency | 0.3009 | workstation - workstation | 07-11-2023 | median 0.3009 ms, mean 0.3121 ms, 1000000 samples, 0.00 % lost messages, Fast DDS | [simulation](https://github.com/robotperf/rosbags/tree/main/simulation) |

