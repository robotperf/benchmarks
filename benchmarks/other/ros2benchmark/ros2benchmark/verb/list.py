# Copyright (C) Acceleration Robotics S.L.U. - All Rights Reserved
#
# Written by Víctor Mayoral-Vilches <victor@accelerationrobotics.com>
# Licensed under the Apache License, Version 2.0

from ros2cli.node.strategy import add_arguments as add_strategy_node_arguments
from ros2cli.node.strategy import NodeStrategy
from ros2benchmark.verb import VerbExtension, Benchmark, run
import os
import yaml


def search_benchmarks(filename="benchmark.yaml", searchpath="src"):
    """
    Returns a list with the paths of each benchmarks' benchmark.yaml
    """
    benchmark_paths = []
    for root, dirs, files in os.walk("./" + searchpath):
        for file in files:
            if file == filename:
                benchmark_paths.append(os.path.join(root, file))    
    return benchmark_paths


class ListVerb(VerbExtension):
    """
    List available benchmarks in the workspace.

    NOTE: each benchmark should follow the specification
    detailed at https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md
    """
    def main(self, *, args):

        # get paths of "benchmark.yaml" files for each benchmark
        benchmark_meta_paths = search_benchmarks()
        
        for meta in benchmark_meta_paths:
            benchmark = Benchmark(meta)
            print(benchmark.name)