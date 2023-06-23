# Benchmark Specification

- [Benchmark Specification](#benchmark-specification)
  - [Motivation](#motivation)
    - [Why ROS 2 for performance benchmarking in robotics?](#why-ros-2-for-performance-benchmarking-in-robotics)
    - [Standard-driven benchmarking](#standard-driven-benchmarking)
  - [General concepts](#general-concepts)
    - [Nomenclature](#nomenclature)
    - [Category](#category)
    - [IDs](#ids)
    - [ROS 2-native](#ros-2-native)
    - [Computing Targets](#computing-targets)
    - [Metric](#metric)
    - [Machine-readable definition of benchmarks](#machine-readable-definition-of-benchmarks)
    - [Readibility of benchmarks](#readibility-of-benchmarks)
  - [Creating a new benchmark](#creating-a-new-benchmark)


<a href="https://accelerationrobotics.com/robotperf.php"><img src="../imgs/ROBOTPerf.svg" align="left" hspace="8" vspace="2" width="200"></a>

This document describes **how each RobotPerf benchmark should be designed and implemented**. The benchmarks are designed to be <ins>representative of the performance of a robotic system and should be easily reproducible</ins> across compute targets. The benchmarks are designed to be <ins>technology agnostic</ins> and <ins>vendor-neutral</ins> so that they can be used to evaluate robotics computing performance across compute substratrated including CPUs, GPUs, FPGAs and other compute accelerators. The benchmarks are designed to be <ins>open and fair</ins> so that robotic architects can make informed decisions about the hardware and software components of their robotic systems. For all these reasons, we build on top of ROS 2, the de facto standard for robot application development.

*The current document is purposely written informal to allow rapid iterations and feedback. In time, the format of this document may change to a more formal specification*.

## Motivation

### Why ROS 2 for performance benchmarking in robotics?

Robot behaviors take the form of computational graphs, with data flowing between computation Nodes, across physical networks (communication buses) and while mapping to underlying sensors and actuators. The popular choice to build these computational graphs for robots these days is the Robot Operating System (ROS)[^1], a framework for robot application development. ROS enables you to build computational graphs and create robot behaviors by providing libraries, a communication infrastructure, drivers and tools to put it all together. Most companies building real robots today use ROS or similar event-driven software frameworks. ROS is thereby the common language in robotics, with several [hundreds of companies](https://github.com/vmayoral/ros-robotics-companies) and thousands of developers using it everyday. ROS 2 [^2] was redesigned from the ground up to address some of the challenges in ROS and solves many of the problems in building reliable robotics systems.


### Standard-driven benchmarking

RobotPerf benchmarks aligns (*and will contribute*) to robotics standards so that you don’t spend time reinventing the wheel and re-develop what already works for most. Particularly benchmarks are conducted using the Robot Operating System 2 ([ROS 2](https://accelerationrobotics.com/ros.php)) as its common baseline. RobotPerf also aligns to standardization initiatives within the ROS ecosystem related to computing performance and benchmarking such as [REP 2008](https://github.com/ros-infrastructure/rep/pull/324) (ROS 2 Hardware Acceleration Architecture and Conventions) and the [REP 2014](https://github.com/ros-infrastructure/rep/pull/364) (Benchmarking performance in ROS 2).


## General concepts

### Nomenclature
This specification follows [REP 2014](https://github.com/ros-infrastructure/rep/pull/364) nomenclature. Refer to this document for more information. Below is the current list of all the terminology deemed important to distinguish. Recommendations for additional terminology are welcomed and can be done through pull requests or discussion during [working group meetings](https://github.com/ros-acceleration/community#meetings).

- **Application.** An application refers to a particular use case that aims to achieve a specific objective in the real world. Each application will vary and may span one or more tasks or sub-tasks. An example of an application would be an autonomous vehicle, which would be associated with a specific set of tasks, sub-tasks, and workloads.
- **Benchmark.** A benchmark is a standardized test or set of tests used to evaluate the performance of computer hardware, software, or systems. A benchmark typically consists of a workload designed to simulate real-world tasks and measure the time it takes to complete them.
- **Benchmarking Suite.** A benchmarking suite refers to a collection of benchmarks, each typically covering one or multiple components of the robotics pipeline.
- **Dataset.** A dataset is a tool that enables standardization of the information used for evaluating benchmarks. A rosbag is one type of dataset that is used for recording and playing back ROS message data. It is a file containing a recorded stream of ROS messages in a serialized format, along with the metadata necessary to reproduce the original message stream. The rosbag can be replayed later to simulate a specific scenario or to analyze data.
- **Meta-package.** A meta-package is a type of package that groups together multiple related packages, providing a convenient way to manage and distribute them as a single unit.
- **Package.** In the context of ROS2, a package is a basic unit of code organization that contains related ROS nodes, libraries, launch files, configuration files, and other resources necessary to implement a particular functionality or behavior.
- **Reference Implementation.** A reference implementation is a standardized implementation of a benchmark that is used as a baseline for comparison with other implementations. The reference implementation serves as a common point of reference for evaluating the performance of different hardware or software configurations.

### Category
Each benchmark should be categorized as one of the following:

| `a` Perception | `b` Localization | `c` Control | `d` Navigation | `e` Manipulation |
|:---:|:---:|:---:|:---:|:---:|
| ![perception benchmarks](../imgs/icon-perception.png) | ![localization benchmarks](../imgs/icon-localization.png)| ![control benchmarks](../imgs/icon-control.png) | ![navigation benchmarks](../imgs/icon-navigation.png) | ![manipulation benchmarks](../imgs/icon-manipulation.png) |

To reflect the category of the benchmark, the benchmark's ROS 2 package name should be prefixed with the corresponding **category identifier letter**  (*e.g. `a` if related to Perception*).
### IDs
Each benchmark should have an associated ID that is unique across all benchmarks. The ID should be a string and correspond with the preffix of the associated ROS 2 package name to that benchmark and that includes the Category (*e.g. ID: `a1`, package name: `a1_perception_2node`*). The ID should be used to identify the benchmark in the RobotPerf suite.

### ROS 2-native 
Each benchmark should be a ROS 2 package and should build and run using the common ROS 2 development flows (build tools, meta-build tools, etc.).

### Computing Targets

Each computing target will be marked with a symbol denoting its group as in:
  - edge/embedded -> 🤖
  - workstation -> 🖥️
  - data center -> 🗄
  - cloud targets -> ⛅

This way, targets can be grouped and compared easily.
### Metric
Benchmarks should be designed to measure one or more metrics and according to [REP 2014](https://github.com/ros-infrastructure/rep/pull/364). Each metric should be associated with a unit of measurement.

### Type
In `RobotPerf` non-functional benchmarks can be of two types: :white_circle: `grey` or :black_circle: `black`. This should be specified on each measurement:

```
             Probe      Probe
             +            +
             |            |
    +--------|------------|-------+     +-----------------------------+
    |        |            |       |     |                             |
    |     +--|------------|-+     |     |                             |
    |     |  v            v |     |     |        - latency   <--------------+ Probe
    |     |                 |     |     |        - throughput<--------------+ Probe
    |     |     Function    |     |     |        - memory    <--------------+ Probe
    |     |                 |     |     |        - power     <--------------+ Probe
    |     +-----------------+     |     |                             |
    |      System under test      |     |       System under test     |
    +-----------------------------+     +-----------------------------+


              Functional                            Non-functional


    +-------------+                     +----------------------------+
    | Test App.   |                     |  +-----------------------+ |
    |  + +  +  +  |                     |  |    Application        | |
    +--|-|--|--|--+---------------+     |  |                   <------------+ Probe
    |  | |  |  |                  |     |  +-----------------------+ |
    |  v v  v  v                  |     |                            |
    |     Probes                  |     |                      <------------+ Probe
    |                             |     |                            |
    |       System under test     |     |   System under test        |
    |                             |     |                      <------------+ Probe
    |                             |     |                            |
    |                             |     |                            |
    +-----------------------------+     +----------------------------+


             Black-Box                            Grey-box

```

- `Black-box performance testing`: Measures performance by eliminating the layers above the *layer-of-interest* and replacing those with a specific test application that stimulates the layer-of-interest in the way you need it. This allows to gather the measurement data right inside your specific test application. The acquisition of the data (the probe) resides inside the test application. A major design constraint in such a test is that you have to eliminate the “application” in order to test the system. Otherwise, the real application and the test application would likely interfere.

- `Grey-box performance testing`: More application-specific measure which is capable of watching internal states of the system and can measure (probe) certain points in the system, thus generate the measurement data with minimal interference. Requires to instrument the complete application.


### Machine-readable definition of benchmarks
So that benchmark information can be easily consumed by other tools, each benchmark should be defined in a machine-readable format. The format will use YAML data serialization language. A YAML file named `benchmark.yaml` should be placed in the root of the ROS 2 package describing each benchmark at any of its results.

For a practical example, refer to the [TEMPLATE](./TEMPLATE.yaml).

### Readibility of benchmarks
For readibility purposes, each benchmark will include a README.md file that describes the benchmark and its results. The README.md file should be placed in the root of the ROS 2 package and will be auto-generated from the machine-readable definition of the benchmark (`benchmark.yaml` file)


## Creating a new benchmark
Create a new benchmark by following these steps:

1. Select the [category](#category) of the benchmark and its corresponding category identifier letter.
2. Compose the benchmark [ID](#ids) by concatenating the category identifier letter with a number that is unique across all benchmarks of that category (generally, iterate to the next number available).
3. Create the corresponding ROS 2 package with the name `<ID>_<benchmark_name>` (*e.g. `a1_perception_2node`*) in the right category folder (*e.g. `perception`*). The benchmark should follow [REP 2014](https://github.com/ros-infrastructure/rep/pull/364) guidelines.
4. Create a `benchmark.yaml` file within the ROS 2 package that describes the benchmark. At least the following fields should be defined:
   - `id`: id of the benchmark
   - `name`: name of the benchmark
   - `description`: short description of the benchmark
   - `short`: short description of the benchmark
   - `graph`: depiction of the computational graph of the benchmark
   - `reproduction`: instructions on how to reproduce the benchmark. Additional reproduction fields can be added as needed (e.g. `reproduction-robotcore` if special instructions are needed to reproduce the benchmark on [ROBOTCORE](https://accelerationrobotics.com/robotcore.php) hardware)
   - `results`: a list of results (`result`), each containing:
     - `type`: `grey` for grey-box and `black` for black-box
     - `metric`: metric measured, e.g. latency, power, throughput, etc.
    - `metric_unit`: unit of the metric measured, e.g. ms, W, etc.
     - `hardware`: name of the hardware used to run the benchmark
     - `timestampt`: timestamp of the result
     - `value`: value of the metric
     - `note`: a note or comment about the result
     - `category`: category of the hardware used to run the benchmark, typically one of the following:
       - `edge/embedded`
       - `workstation`
       - `data center`
       - `cloud`
     - datasource: path of the datasource to obtain the result (relative to [rosbags](https://github.com/robotperf/rosbags) repository of RobotPerf)

For a practical example, refer to the [TEMPLATE](./TEMPLATE.yaml). 