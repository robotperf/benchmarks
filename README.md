# RobotPerf Benchmarks

[**Benchmarks** 🤖](#benchmarks) | [*Contributing* 🌍](#contributing) | [`Contact and support` 📨](#contact-and-support)

<a href="https://accelerationrobotics.com/robotperf.php"><img src="imgs/robotperf-temporary.png" align="left" hspace="8" vspace="2" width="200"></a>

RobotPerf is an **open reference benchmarking suite that is used to evaluate robotics computing performance** fairly with [ROS 2](https://accelerationrobotics.com/ros.php) as its common baseline, *so that robotic architects can make informed decisions about the hardware and software components of their robotic systems*. 

The project's <ins>mission is to build open, fair and useful robotics benchmarks that are technology agnostic, vendor-neutral and provide unbiased evaluations of robotics computing performance for hardware, software, and services</ins>.  As a reference performance benchmarking suite in robotics, RobotPerf *can be used to evaluate robotics computing performance across compute substratrated including CPUs, GPUs, FPGAs and other compute accelerators*. The benchmarks are designed to be representative of the performance of a robotic system and to be reproducible across different robotic systems. For that, RobotPerf builds on top of ROS 2, the de facto standard for robot application development.

<details><summary><b>More on "why ROS 2"?</b></summary>

<sup><sub>Robot behaviors take the form of computational graphs, with data flowing between computation Nodes, across physical networks (communication buses) and while mapping to underlying sensors and actuators. The popular choice to build these computational graphs for robots these days is the Robot Operating System (ROS)[^1], a framework for robot application development. ROS enables you to build computational graphs and create robot behaviors by providing libraries, a communication infrastructure, drivers and tools to put it all together. Most companies building real robots today use ROS or similar event-driven software frameworks. ROS is thereby the common language in robotics, with several [hundreds of companies](https://github.com/vmayoral/ros-robotics-companies) and thousands of developers using it everyday. ROS 2 [^2] was redesigned from the ground up to address some of the challenges in ROS and solves many of the problems in building reliable robotics systems.</sup></sub>

</details>


<details><summary><b>Standards</b></summary>

<sup><sub>RobotPerf aligns to robotics standards so that you don’t spend time reinventing the wheel and re-develop what already works for most. Particularly benchmarks are conducted using the Robot Operating System 2 ([ROS 2](https://accelerationrobotics.com/ros.php)) as its common baseline. RobotPerf also aligns to standardization initiatives within the ROS ecosystem related to computing performance and benchmarking such as [REP 2008](https://github.com/ros-infrastructure/rep/pull/324) (ROS 2 Hardware Acceleration Architecture and Conventions) and the [REP 2014](https://github.com/ros-infrastructure/rep/pull/364) (Benchmarking performance in ROS 2).</sup></sub>

</details>

## Benchmarks

| Perception | Localization | Control | Navigation | Manipulation |
|:---:|:---:|:---:|:---:|:---:|
| [![perception benchmarks](imgs/icon-perception.png)](#perception) | [![localization benchmarks](imgs/icon-localization.png)](#localization) | [![control benchmarks](imgs/icon-control.png)](#control) | [![navigation benchmarks](imgs/icon-navigation.png)](#navigation) | [![manipulation benchmarks](imgs/icon-manipulation.png)](#manipulation) | 

RobotPerf benchmarks aim to cover the **complete robotics pipeline** including perception, localization, motion control, control, manipulation and navigation. In time, new benchmarks will be added.

<sup><sub> New categories may appear over time. </sup></sub>


### Perception
TODO


### Localization
TODO
### Control
TODO
### Navigation
TODO
### Manipulation
TODO

## Contributing

## Contact and support



[^1]: Quigley, M., Conley, K., Gerkey, B., Faust, J., Foote, T., Leibs, J., ... & Ng, A. Y. (2009, May). ROS: an open-source Robot Operating System. In ICRA workshop on open source software (Vol. 3, No. 3.2, p. 5).
[^2]: Macenski, S., Foote, T., Gerkey, B., Lalancette, C., & Woodall, W. (2022). Robot Operating System 2: Design, architecture, and uses in the wild. Science Robotics, 7(66), eabm6074.
