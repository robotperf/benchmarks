# Copyright (C) Acceleration Robotics S.L.U. - All Rights Reserved
#
# Written by Víctor Mayoral-Vilches <victor@accelerationrobotics.com>
# Licensed under the Apache License, Version 2.0

# For each benchmarks in the workspace, update README.md file. Then, 
# update general README.md file.

from ros2cli.node.strategy import add_arguments as add_strategy_node_arguments
from ros2cli.node.strategy import NodeStrategy
from ros2benchmark.verb import VerbExtension, Benchmark, run, search_benchmarks, search_benchmarks_repo
import os
import yaml
import sys

class UpdateVerb(VerbExtension):
    """
    Update README.md files, first from benchmarks, then general
    """
    def update_general_readme_benchmarks(self, general_readme_path, new_content="This is the new content."):
        """Update benchmarks' general README.md file benchmarks
        """

        # Open the README.md file for reading and writing
        with open(general_readme_path, "r+") as f:
            # Read the contents of the file
            content = f.read()

            # Find the start and end markers
            start_marker = "<!-- perception-benchmarks-init -->"
            end_marker = "<!-- perception-benchmarks-fini -->"
            start_index = content.find(start_marker)
            end_index = content.find(end_marker) + len(end_marker)

            # Replace the content between the markers with new content
            updated_content = content[:start_index] + start_marker + "\n\n" + new_content + "\n\n" + end_marker + content[end_index:]
            
            # Move the file pointer to the beginning of the file and truncate the file
            f.seek(0)
            f.truncate()

            # Write the updated content to the file
            f.write(updated_content)        

    def main(self, *, args):
        """
        For each benchmarks in the workspace, update README.md file. Then, 
        update general README.md file.

        NOTE: each benchmark should be defined by its benchmark.yaml file.
        and follow the specification detailed at 
        https://github.com/robotperf/benchmarks/blob/main/benchmarks/README.md
        """
        benchmark_list = []

        ## For each benchmark, update README.md
        #
        # get paths of "benchmark.yaml" files for each benchmark
        benchmark_meta_paths = search_benchmarks()
        for meta in benchmark_meta_paths:
            benchmark = Benchmark(meta)
            benchmark_list.append(benchmark)
            # get the corresponding README.md file for the benchmark
            readme_path = meta.replace("benchmark.yaml", "README.md")

            with open(readme_path, 'w') as file:
                file.write(benchmark.markdown())

        ## Update the general README.md
        benchmarks_repo_path = search_benchmarks_repo()
        if benchmarks_repo_path is not None:
            general_readme_path = benchmarks_repo_path + "/README.md"
        else:
            print('General README not found. Could not update it')
            sys.exit(1)

        md = "### Perception\n\n"        
        md += "| ID | Graph | Summary | Metric | Hardware | Value | Category | Timestamp | Note | Data Source |\n"
        md += "| --- | --- | -------- | --- | ----------- | --- | --- | --- | --- | --- |\n"

        for bench in benchmark_list:
            md+= bench.markdown_general()

        # self.update_general_readme_benchmarks(general_readme_path, new_content=md)
        self.update_general_readme_benchmarks(general_readme_path, new_content="")