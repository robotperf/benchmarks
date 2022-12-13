# a1_perception_2nodes

### ID
`a1`
### Description
A simple perception computational graph composed by 2 Components, `rectify` and `resize` operations. Used to demonstrate a simple perception pipeline using the [image_pipeline](https://github.com/ros-perception/image_pipeline) package.


### Metric
`latency`, ms

### Reproduction
```bash
# Install the image_pipeline package
apt-get install ros-humble-image-pipeline

# Create a ROS 2 overlay workspace
mkdir -p ~/ros2_overlay_ws/src  

# Clone the benchmark repository
cd ~/ros2_overlay_ws/src
git clone https://github.com/robotperf/benchmarks

# Build the benchmark
cd ~/ros2_overlay_ws; colcon build --packages-select a1_perception_2nodes

# Launch the benchmark
ros2 launch a1_perception_2nodes a1_perception_2nodes.launch.py

# Generate the report for the benchmark
ros2 launch a1_perception_2nodes a1_perception_2nodes.launch.py
```

### Results

| ID | Benchmark summary | Metric | 🖥️ AMD Ryzen 5[^3] | 🤖[`ROBOTCORE`](https://accelerationrobotics.com/robotcore.php)[^4] | 🤖 `Kria KR260` | 🤖 `Jetson Nano` | 🤖 `Jetson AGX Xavier` |
|:---:|---|---|:---:|:---:|:---:|:---:|:---:|
| (*example, <ins>to be updated</ins>*) [`a1`](benchmarks/perception/a1_perception_2nodes) | Perception computational graph composed by 2 dataflow-connected *Components*, `rectify` and `resize`. Package relies on [image_pipeline](https://github.com/ros-perception/image_pipeline) package. | latency (ms) |  | **66.82** ms <sub><sup><i>(14-10-2022)</i></sub></sup> |  66.82 ms (⚪`1.0`x)<sub><sup><i>(14-10-2022)</i></sub></sup> |  238.13 ms (🔻`0.38`x)<sub><sup><i>(14-10-2022)</i></sub></sup> | 106.34 ms (🔻`0.86`x)<sub><sup><i>(14-10-2022)</i></sub></sup> |