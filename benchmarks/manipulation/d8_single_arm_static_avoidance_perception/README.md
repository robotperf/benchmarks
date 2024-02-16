# d8_single_arm_static_avoidance_perception

Benchmark involving a single robotic arm with holding a static position and implementing a perception-based collision avoidance approach.

### ID
d8

### Description
A computational graph whose perception workload allows one robotic arm avoid collisions while maintaining a static position.

![](../../../imgs/d8_single_arm_static_avoidance_perception.svg)

## Reproduction Steps

```bash
## collect data for benchmarking
ros2 launch d8_single_arm_static_avoidance_perception trace_d8_single_arm_static_avoidance_perception.launch.py

# NOTE: Refer to https://github.com/robotperf/benchmarks/tree/main/benchmarks/manipulation/d8_single_arm_static_avoidance_perception and review the launch files for more details.
#
# NOTE 2: An alternative way to reproduce the same results without relying on the launch files of this package
# is described below:
#
## no tracing
ros2 launch cobra_setup cobra_control.launch.py setup:=xarm5 simulation:=true | highlight
## tracing
ros2 launch cobra_setup trace_cobra_control.launch.py setup:=xarm5 simulation:=true | highlight
```

*NOTE*: 

## Results

| Type | Hardware | Metric | Value | Category | Timestamp | Note | Data Source |
| --- | --- | --- | --- | --- | --- | --- | --- |
| | | |  | |  | |  |
