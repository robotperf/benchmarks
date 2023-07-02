# Copyright (C) Acceleration Robotics S.L.U. - All Rights Reserved
#
# Written by Víctor Mayoral-Vilches <victor@accelerationrobotics.com>
# Licensed under the Apache License, Version 2.0

from ros2cli.node.strategy import add_arguments as add_strategy_node_arguments
from ros2cli.node.strategy import NodeStrategy
from ros2benchmark.verb import VerbExtension, Benchmark, run, search_benchmarks
from ros2benchmark.verb.summary import SummaryVerb
import os
import yaml
import pprint
import matplotlib.pyplot as plt
import arrow
import random


class ReportVerb(VerbExtension):
    """
    Generate a markdown report with the results of the benchmarks in the workspace.

    NOTE: each benchmark should follow the specification
    detailed at https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md
    """ 

    def write_to_file(self, path, content):
        # Create directory structure if it doesn't exist
        directory = os.path.dirname(path)
        if not os.path.exists(directory):
            os.makedirs(directory)
        
        # Open the file and write the content
        with open(path, 'w') as f:
            f.write(content)

    @staticmethod
    def plot_function_values(data_sample):
        """
        Extract the "value" from a data_sample

        :param data_sample: dict with the data
        """
        return data_sample['value']

    @staticmethod
    def plot_function_names(data_sample):
        """
        Extract the "name" from a data_sample

        :param data_sample: dict with the data
        """
        return data_sample['name']

    @staticmethod
    def plot_function_names_forid(data_sample):
        """
        Extract the "name" from a data_sample

        :param data_sample: dict with the data
        """

        return_str = ""         
        return_str += data_sample['hardware'] + " (" + data_sample['timestampt'] + ")"
        if data_sample['type'].lower() == "grey":
            return_str += "⚪"
        elif data_sample['type'].lower() == "black":
            return_str += "⚫"
        else:
            return_str += "?"

        return return_str

    @staticmethod
    def plot_data(data, 
                  title,
                  name_function=plot_function_names,
                  value_function=plot_function_values, 
                  filter=None, 
                  unique=False):
        """
        Plot the data in a bar plot and save it to a file.

        :param data: list of dicts with the data to plot
        :param title: title of the plot
        :param name_function: function to extract the name from the data
        :param value_function: function to extract the value from the data
        :param filter: function to filter the data
        :param unique: if True, only the most recent entry for each 'name', 'hardware', 'type', 'datasource' combination is plotted
        """

        plotpath = '/tmp/report-' + title + '-latency.png'
        if filter:
            # Filter data using the provided function
            filtered_data = [d for d in data if filter(d)]
        else:
            filtered_data = data

        if unique:
            # Use a dictionary to hold the most recent entry for each 'name', 
            # 'hardware', 'type', 'datasource' combination
            filtered_dict = {}
            for entry in filtered_data:
                name = entry['name'] + entry['hardware'] + entry['type'] + entry['datasource']
                if name not in filtered_dict or arrow.get(entry['timestampt'], ['D-M-YYYY', 'YYYY-MM-DD HH:mm:ss']).datetime > arrow.get(filtered_dict[name]['timestampt'], ['D-M-YYYY', 'YYYY-MM-DD HH:mm:ss']).datetime:
                    filtered_dict[name] = entry
            filtered_data = filtered_dict.values()            

        names = [name_function(d) for d in filtered_data]
        values = [value_function(d) for d in filtered_data]

        # # debug
        # print(names)
        # print(values)
        # # print(filtered_data)
        
        # colors = [plt.cm.viridis(random.random()) for _ in range(len(names))]
        colors = [plt.cm.viridis(random.random(), alpha=0.2 if (data['type'].lower() == "grey") else 1.0) for data in filtered_data]

        # Create a bar plot
        plt.figure(figsize=(10, 5))
        # plt.bar(names, values, color='blue')
        plt.bar(names, values, color=colors)
        plt.title('Latency Values: ' + title)
        plt.xlabel('Hardware')
        plt.ylabel('Latency (ms)')
        plt.xticks(rotation=80)  # Rotate x-axis labels for better visibility

        # Save the figure
        plt.savefig(plotpath, bbox_inches='tight')
        
        return plotpath


    def main(self, *, args):
        # get paths of "benchmark.yaml" files for each benchmark
        benchmark_meta_paths = search_benchmarks()
        list_results = SummaryVerb.preprocess(benchmark_meta_paths)
        # pprint.pprint(list_results)

        benchmark_id_report = ""
        # Add a title to the report indicating a timestamp of when it was generated
        benchmark_id_report += f"# [RobotPerf](https://robotperf.org/) report (generated on `{arrow.now().format('YYYY-MM-DD HH:mm:ss')}`)\n\n"

        # Generate a Table of Contents
        benchmark_id_report += "## Table of Contents\n"
        benchmark_id_report += "- [Intro](#intro)\n"
        benchmark_id_report += "- [Legend](#legend)\n"
        benchmark_id_report += "- [Benchmark results by `id`](#benchmark-results-by-id)\n"
        benchmark_id_report += "- [Benchmarking results by `hardware` solution](#benchmarking-results-by-hardware-solution)\n"

        ######################################################
        # 0. Intro and Legend
        ######################################################
        benchmark_id_report += "## Intro\n"

        benchmark_id_report += '[**Benchmark results** 🤖](https://github.com/robotperf/benchmarks#benchmarks) | [Benchmark spec 📖](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md) | [*Contributing* 🌍](https://github.com/robotperf/benchmarks#contributing) | [`Contact and support` 📨](https://github.com/robotperf/benchmarks#contact-and-support)\n'


        benchmark_id_report += '<a href="https://accelerationrobotics.com/robotperf.php"><img src="https://raw.githubusercontent.com/robotperf/benchmarks/main/imgs/ROBOTPerf.svg" align="left" hspace="8" vspace="2" width="200"></a>'
        benchmark_id_report += 'RobotPerf is an **open reference benchmarking suite that is used to evaluate robotics computing performance** fairly with [ROS 2](https://accelerationrobotics.com/ros.php) as its common baseline, *so that robotic architects can make informed decisions about the hardware and software components of their robotic systems*.\n\n'
        benchmark_id_report += "The project's <ins>mission is to build open, fair and useful robotics benchmarks that are technology agnostic, vendor-neutral and provide unbiased evaluations of robotics computing performance for hardware, software, and services</ins>.  As a reference performance benchmarking suite in robotics, RobotPerf *can be used to evaluate robotics computing performance across compute substratrated including CPUs, GPUs, FPGAs and other compute accelerators*. The benchmarks are designed to be representative of the performance of a robotic system and to be reproducible across different robotic systems. For that, RobotPerf builds on top of ROS 2, the de facto standard for robot application development.\n\n"
        benchmark_id_report += "RobotPerf is a project led by [Acceleration Robotics](https://accelerationrobotics.com/), a company that provides robotics acceleration solutions for the next generation of robots. Refer to the [Benchmark Specification](https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md) for more details.\n\n"

        # Add a legend
        benchmark_id_report += "### Legend\n"
        benchmark_id_report += "\n"
        benchmark_id_report += "**Metrics**\n"
        benchmark_id_report += "- ⏱: latency\n"
        benchmark_id_report += "- 📶: throughput (fps)\n"
        benchmark_id_report += "- ⚡: power\n"
        benchmark_id_report += "\n"
        benchmark_id_report += "**Type**\n"
        benchmark_id_report += "- ⚪: grey\n"
        benchmark_id_report += "- ⚫: black\n"
        benchmark_id_report += "- ✋: manual (rosbag directly)\n"
        benchmark_id_report += "\n"
        benchmark_id_report += "**Category**\n"
        benchmark_id_report += "- 📟: edge/embedded\n"
        benchmark_id_report += "- 🖥️: workstation\n"
        benchmark_id_report += "- 🗄: data center\n"
        benchmark_id_report += "- ⛅: cloud targets\n\n"
        
        ######################################################        
        # 1. Print all results for each benchmark
        ######################################################
        benchmark_id_report += "## Benchmark results by `id`\n"
        list_ids = SummaryVerb.extract_unique_x(list_results, "id")
        alphabetical_list_ids = sorted(list_ids)
        for benchmark_id in alphabetical_list_ids:
            # get body
            filtered_data = [item for item in list_results if item['id'] == benchmark_id]
            benchmark_id_report += SummaryVerb.to_markdown_table(filtered_data, benchmark_id)
            # plot
            plotpath = (ReportVerb.plot_data(filtered_data, 
                                            name_function=ReportVerb.plot_function_names_forid,
                                            value_function=ReportVerb.plot_function_values,
                                            title=benchmark_id, 
                                            unique=False))
            # plotpath = (ReportVerb.plot_data(list_results, title=benchmark_id, filter=SummaryVerb.filter_robotcore, unique=True))        
            benchmark_id_report += f"\n![{plotpath}]({plotpath})"
        benchmark_id_report += f"\n\n"

        ######################################################
        # 2. Print all results grouped by hardware solution, for all of the solutions
        # ######################################################
        benchmark_id_report += "## Benchmarking results by `hardware` solution\n"
        # NOTE: ordered by timestamp and name
        list_hardware = SummaryVerb.extract_unique_x(list_results, "hardware")
        for hw in list_hardware:
            extracted_data = [d for d in list_results if d['hardware'] == hw]
            benchmark_id_report += SummaryVerb.to_markdown_table(extracted_data, hw)
        

        # produce report
        self.write_to_file('/tmp/report.md', benchmark_id_report)
        print("Writing report to /tmp/report.md")
