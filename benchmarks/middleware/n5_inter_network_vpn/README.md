# n5_inter_network_vpn

Network computational graph composed by two nodes.

### ID
n5

### Description
A simple network computational graph composed by two nodes. Used to demonstrate a simple ping-pong for inter-network communication via Wireguard.

![](../../../imgs/n5_inter_network_vpn.png)

## Reproduction Steps

```bash
Refer to https://github.com/robotperf/benchmarks/tree/main/benchmarks/network/n5_inter_network_vpn and review the launch files to reproduce this package.

# 1) Prior to launching the node, the VPN must be configured and enabled from both sides (client and server).

# Make sure the firewall is not blocking the communication:

# 1.a) Option 1: disable the firewall
sudo ufw disable

# 1.b) Option 2: specific configuration of the firewall. For example, to allow traffic in computer 1 (10.0.0.1) towards computer 2 (10.0.0.2):
sudo ufw allow from 10.0.0.2
sudo ufw allow to 10.0.0.2


# 2) Besides, CycloneDDS must be properly configured:

# 2.1) Create cycloneDDS.xml file and dump the following content and replacing the address by those VPN addresses of the computers involved:

<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:schemaLocation="https://cdds.io/config https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd">
<Domain id="any">
    <Compatibility><ManySocketsMode>many</ManySocketsMode></Compatibility>
    <!-- For CycloneDDS in ROS Galactic -->
    <!-- <General><NetworkInterfaceAddress>enp1s0</NetworkInterfaceAddress></General> -->
    <!-- For CycloneDDS in ROS Humble or Rolling -->
    <General><Interfaces><NetworkInterface name="wg0"/></Interfaces></General>

    <Discovery>
        <Peers><Peer address="10.0.0.1"/><Peer address="10.0.0.2"/></Peers>
        <ParticipantIndex>auto</ParticipantIndex>
    </Discovery>
</Domain>
</CycloneDDS>

# 2.2) Configure ROS to use CycloneDDS as middleware 

export CYCLONEDDS_URI=file://$PWD/cyclonedds.xml
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# 3) Finally, compile and launch
colcon build --merge-install
source install/local_setup.bash
ros2 launch n5_inter_network_vpn trace_n5_inter_network_vpn_server.launch.py # Launch server
ros2 launch n5_inter_network_vpn trace_n5_inter_network_vpn_client.launch.py # Launch client

```

## Results

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

