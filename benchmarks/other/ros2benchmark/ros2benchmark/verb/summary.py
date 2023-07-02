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

    @staticmethod
    def preprocess(benchmark_meta_paths):
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
        list_preprocess = []
        
        for meta in benchmark_meta_paths:
            benchmark = Benchmark(meta)
            for result in benchmark.results:
                result["name"] = benchmark.name
                result["id"] = benchmark.id
                list_preprocess.append(result)
        
        return list_preprocess


    def most_recent(self, data):
        """ Return the most recent entry for each 'name' in the provided data"""

        # Use a dictionary to hold the most recent entry for each 'name'
        filtered_dict = {}
        for entry in data:
            name = entry['name'] + entry['hardware'] + entry['type'] + entry['datasource']
            if name not in filtered_dict or arrow.get(entry['timestampt'], ['D-M-YYYY', 'YYYY-MM-DD HH:mm:ss']).datetime > arrow.get(filtered_dict[name]['timestampt'], ['D-M-YYYY', 'YYYY-MM-DD HH:mm:ss']).datetime:
                filtered_dict[name] = entry
        return filtered_dict.values()        

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

    @staticmethod
    def filter_robotcore(d):
        return d['hardware'] == 'ROBOTCORE'

    @staticmethod
    def extract_unique_x(data, x="hardware"):
        hardware_set = {d[x] for d in data}
        return list(hardware_set)        

    @staticmethod
    def to_markdown_table(data, title=None, unique=False):
        """Print a list of results as a markdown table, sorted by timestamp and name"""
        
        # Sort the data by 'timestampt' and 'name' and 'metric'
        sorted_data = sorted(
            data,
            key=lambda d: (arrow.get(d['timestampt'], ['D-M-YYYY', 'YYYY-MM-DD HH:mm:ss']).datetime, d['name']),
            reverse=True
        )

        if unique:
            sorted_data = self.most_recent(sorted_data)

        return_str = ""
        
        # Print the table header
        if title:
            # print(f"\n**{title}**")
            return_str += f"\n**{title}**\n"

        # print("| Type | Benchmark | Metric | Value | Category | Timestamp | Note | Data Source |")
        # print("| --- | --- | --- | --- | --- | --- | --- | --- |")
        return_str += "| Type | Benchmark | Metric | Value | Category | Hardware | Timestamp | Note | Data Source |\n"
        return_str += "| --- | --- | --- | --- | --- | --- | --- | --- | --- |\n"

        # Print each row of the table
        for d in sorted_data:
            # customize the results
            if d['type'].lower() == "grey":
                aux_type = "[:white_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type)"
            elif d['type'].lower() == "black":
                aux_type = "[:black_circle:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#type)"
            else:
                aux_type = d['type']

            if d['category'] == "workstation":
                aux_category = "[:desktop_computer:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#computing-targets)"
            elif d['category'] == "edge":
                aux_category = "[:pager:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#computing-targets)"
            elif d['category'] == "cloud":
                aux_category = "[:partly_sunny:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#computing-targets)"
            elif d['category'] == "datacenter":
                aux_category = "[:file_cabinet:](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md#computing-targets)"
            else:
                aux_category = d['category']

            datasource = "[{}](https://github.com/robotperf/rosbags/tree/main/{})".format(d['datasource'], d['datasource'])
            subgroup = ""
            if d['id'].startswith("a"):
                subgroup = "perception"
            elif d['id'].startswith("b"):
                subgroup = "localization"
            elif d['id'].startswith("c"):
                subgroup = "control"
            benchmark_url = "[{}](https://github.com/robotperf/benchmarks/tree/main/benchmarks/{}/{})".format(d['name'], subgroup, d['name'])

            metric_icon = d['metric']
            if d['metric'].lower() == "latency":
                metric_icon += " :stopwatch:"
            elif d['metric'].lower() == "throughput":
                metric_icon += " :signal_strength:"
            elif d['metric'].lower() == "power":
                metric_icon += " :zap:"

            # print(f"| {aux_type} | {benchmark_url} | {metric_icon} | {d['value']} | {d['category']} | {d['timestampt']} | {d['note']} | {datasource} |")
            return_str += f"| {aux_type} | {benchmark_url} | {metric_icon} | {d['value']} | {aux_category} |  {d['hardware']} | {d['timestampt']} | {d['note']} | {datasource} |\n"
        return return_str

    def main(self, *, args):
        # get paths of "benchmark.yaml" files for each benchmark
        benchmark_meta_paths = search_benchmarks()
        list_results = SummaryVerb.preprocess(benchmark_meta_paths)
        # pprint.pprint(list_results)

        # # 0. Print all results for a given hardware solution
        # ######################################################
        # # NOTE: ordered by timestamp and name
        # # NOTE 2: unique applied, so only the most recent entry for each 'name','hardware','type','datasource' 
        # # combination is printed
        # hw = 'Intel i7-8700K'
        # filtered_data = [item for item in list_results if item['hardware'] == hw]
        # print(SummaryVerb.to_markdown_table(filtered_data, hw, unique=True))
        # ######################################################
        
        # # 1. Print all results for a given benchmark
        # ######################################################
        # # NOTE: ordered by timestamp and name
        # benchmark_id = 'a1'
        # filtered_data = [item for item in list_results if item['id'] == benchmark_id]
        # print(SummaryVerb.to_markdown_table(filtered_data, benchmark_id))
        # ######################################################

        # 2. Print all results grouped by hardware solution, for all of the solutions
        ######################################################
        # NOTE: ordered by timestamp and name
        list_hardware = self.extract_unique_x(list_results, "hardware")
        for hw in list_hardware:
            extracted_data = [d for d in list_results if d['hardware'] == hw]
            print(SummaryVerb.to_markdown_table(extracted_data, hw))
        ######################################################                

        # self.plot_data(list_results, SummaryVerb.filter_robotcore)
