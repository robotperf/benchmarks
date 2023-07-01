# Copyright (C) Acceleration Robotics S.L.U. - All Rights Reserved
#
# Written by Víctor Mayoral-Vilches <victor@accelerationrobotics.com>
# Licensed under the Apache License, Version 2.0

from ros2cli.node.strategy import add_arguments as add_strategy_node_arguments
from ros2cli.node.strategy import NodeStrategy
from ros2benchmark.verb import VerbExtension, Benchmark, run, search_benchmarks
import os
import yaml
import pprint
import matplotlib.pyplot as plt
import arrow


class SummaryVerb(VerbExtension):
    """
    Preprocess and summarize available benchmarks in the workspace.

    NOTE: each benchmark should follow the specification
    detailed at https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md
    """

    def proprocess(self, benchmark_meta_paths):
        """ Preprocess a benchmark and return list with all results

        End up dumpting a list with dicts as follows
        {
            "metric": metric,
            "metric_unit": metric_unit,
            "type": result_type, # "type" is a reserved keyword in Python, so we use "result_type
            "hardware": hardware,
            "category": category,
            "timestampt": timestampt,
            "value": value,
            "note": note,
            "datasource": datasource
            "name": name
            "id": id
        }    
        """
        list_proceprocess = []
        
        for meta in benchmark_meta_paths:
            benchmark = Benchmark(meta)
            for result in benchmark.results:
                result["name"] = benchmark.name
                result["id"] = benchmark.id
                list_proceprocess.append(result)
        
        return list_proceprocess


    def plot_data(self, data, condition_func):
        # Filter data using the provided function
        filtered_data = [d for d in data if condition_func(d)]

        # Use a dictionary to hold the most recent entry for each 'name'
        filtered_dict = {}
        for entry in filtered_data:
            name = entry['name']
            if name not in filtered_dict or arrow.get(entry['timestampt'], ['D-M-YYYY', 'YYYY-MM-DD HH:mm:ss']).datetime > arrow.get(filtered_dict[name]['timestampt'], ['D-M-YYYY', 'YYYY-MM-DD HH:mm:ss']).datetime:
                filtered_dict[name] = entry

        # Extract 'name' and 'value' for each filtered record
        names = [d['name'] for d in filtered_dict.values()]
        values = [d['value'] for d in filtered_dict.values()]

        # Create a bar plot
        plt.figure(figsize=(10, 5))
        plt.bar(names, values, color='blue')
        plt.title('Latency Values')
        plt.xlabel('Name')
        plt.ylabel('Latency (ms)')
        plt.xticks(rotation=90)  # Rotate x-axis labels for better visibility

        # Save the figure
        plt.savefig('/tmp/resultimg.png', bbox_inches='tight')

    def filter_robotcore(self, d):
        return d['hardware'] == 'ROBOTCORE'

    def main(self, *, args):
        # get paths of "benchmark.yaml" files for each benchmark
        benchmark_meta_paths = search_benchmarks()
        list_results = self.proprocess(benchmark_meta_paths)
        # pprint.pprint(list_results)

        # filtered_data = [item for item in list_results if item['hardware'] == 'ROBOTCORE']
        # pprint.pprint(filtered_data)

        self.plot_data(list_results, self.filter_robotcore)