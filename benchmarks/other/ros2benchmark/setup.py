from setuptools import find_packages
from setuptools import setup

package_name = "ros2benchmark"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/" + package_name, ["package.xml"]),
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
    ],
    install_requires=["ros2cli", "pyyaml"],
    zip_safe=True,
    author="Víctor Mayoral Vilches",
    author_email="victor@accelerationrobotics.com",
    maintainer="Víctor Mayoral Vilches",
    maintainer_email="victor@accelerationrobotics.com",
    url="https://github.com/robotperf/benchmarks/tree/main/benchmarks/other/ros2benchmark",
    download_url="https://github.com/robotperf/benchmarks",
    keywords=[],
    classifiers=[
        "Environment :: Console",
        "Intended Audience :: Developers",
        "License :: OSI Approved :: Apache Software License",
        "Programming Language :: Python",
    ],
    description="The benchmark command for ROS 2 command line tools.",
    long_description="""\
The package provides the benchmark command for the ROS 2 command line tools.""",
    license="Apache License, Version 2.0",
    tests_require=["pytest"],
    entry_points={
        "ros2cli.command": [
            "benchmark = ros2benchmark.command.benchmark:BenchmarkCommand",
        ],
        "ros2cli.extension_point": [
            "ros2benchmark.verb = ros2benchmark.verb:VerbExtension",
        ],
        "ros2benchmark.verb": [
            "list = ros2benchmark.verb.list:ListVerb",
            "update = ros2benchmark.verb.update:UpdateVerb",
        ],
    },
)
