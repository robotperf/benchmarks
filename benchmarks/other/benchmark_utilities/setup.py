from setuptools import find_packages
from setuptools import setup

package_name = 'benchmark_utilities'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/" + package_name, ["package.xml"]),
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Martiño Crespo',
    author_email='martinho@accelerationrobotics.com',
    maintainer='martinho',
    maintainer_email='martinho@accelerationrobotics.com',
    url="https://github.com/robotperf/benchmarks/tree/main/benchmarks/other/benchmark_utilities",
    download_url="https://github.com/robotperf/benchmarks",
    keywords=[],
    classifiers=[
        "Environment :: Console",
        "Intended Audience :: Developers",
        "License :: OSI Approved :: Apache Software License",
        "Programming Language :: Python",
    ],
    description="Tools to be used when writing analysis scripts for the benchmarks",
    long_description="""\
The package provides the tools to be used when writing analysis scripts for the benchmarks.""",
    license="Apache License, Version 2.0",
    tests_require=["pytest"],
)
