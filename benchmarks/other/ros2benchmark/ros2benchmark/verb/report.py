# Copyright (C) Acceleration Robotics S.L.U. - All Rights Reserved
#
# Written by Víctor Mayoral-Vilches <victor@accelerationrobotics.com>
# Licensed under the Apache License, Version 2.0

from ros2cli.node.strategy import add_arguments as add_strategy_node_arguments
from ros2cli.node.strategy import NodeStrategy
from ros2benchmark.verb import VerbExtension, Benchmark, run, search_benchmarks
from ros2benchmark.verb.summary import SummaryVerb
from collections import defaultdict
import os
import yaml
import pprint
import matplotlib.pyplot as plt
import numpy as np
import arrow
import random
import sys
import seaborn as sns


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
        # if data_sample['category'] == "workstation":
        #     return "🖥️" + data_sample['name']
        # elif data_sample['category'] == "edge":
        #     return "📟" + data_sample['name']            
        # elif data_sample['category'] == "cloud":
        #     return "⛅" + data_sample['name']            
        # elif data_sample['category'] == "datacenter":
        #     return "🗄" + data_sample['name']            
        # else:
        #     return data_sample['name']
        return data_sample['name']

    @staticmethod
    def plot_function_names_forid(data_sample):
        """
        Extract the "name" from a data_sample

        :param data_sample: dict with the data
        """

        return_str = ""         
        # return_str += data_sample['hardware'] + " (" + data_sample['timestampt'] + ")"

        if "ROBOTCORE®" in data_sample['hardware']:
            return_str += data_sample['hardware'].replace("ROBOTCORE®", "$\mathbf{ROBOTCORE®}$")
        else:
            return_str += data_sample['hardware']


        if data_sample['type'].lower() == "grey":
            return_str += "⚪"
        elif data_sample['type'].lower() == "black":
            return_str += "⚫"
        else:
            return_str += "?"

        return return_str

    @staticmethod
    def plot_function_id_withbenchtype(data_sample):
        """
        Extract the "name" from a data_sample

        :param data_sample: dict with the data
        """

        return_str = ""         
        return_str += data_sample['id']


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
                  xlabel,
                  ylabel,
                  name_function=plot_function_names,
                  value_function=plot_function_values, 
                  filter=None, 
                  unique=False,
                  sortedata=False, 
                  sortedatareverse=False,
                  filterout=None,
                  colors_dict=None):
        """
        Plot the data in a bar plot and save it to a file.

        :param data: list of dicts with the data to plot
        :param title: title of the plot
        :param name_function: function to extract the name from the data
        :param value_function: function to extract the value from the data
        :param filter: function to filter the data
        :param unique: if True, only the most recent entry for each 'name', 'hardware', 'type', 'datasource' combination is plotted
        :param sortedata: if True, the data is sorted by value
        :param sortedatareverse: if True, the data is sorted by value in reverse order
        :param filterout: list of tuples (hardware, type) to filter out and not consider
        :param colors_dict: dict of colors to use for each hardware as key
        """

        plotpath = '/tmp/report-' + title + '.png'
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
            
                # NOTE: condition 1: if not in dict or more recent than the one in the dict
                # if name not in filtered_dict or arrow.get(entry['timestampt'], ['D-M-YYYY', 'YYYY-MM-DD HH:mm:ss']).datetime > arrow.get(filtered_dict[name]['timestampt'], ['D-M-YYYY', 'YYYY-MM-DD HH:mm:ss']).datetime:

                # NOTE: condition 2: if not in dict or lower "value" than the one in the dict
                # not in filterout and greater then 0.001 (heuristic to remove outliers, close to zero)
                if (name not in filtered_dict or entry['value'] < filtered_dict[name]['value']) and (filterout is None or (entry['hardware'], entry['type']) not in filterout) and (entry['value'] > 0.001):
                    filtered_dict[name] = entry

            filtered_data = filtered_dict.values()            

        # order list using "value"
        if sortedata:
            filtered_data = sorted(filtered_data, key=lambda x: x['value'])
        if sortedatareverse:
            filtered_data = sorted(filtered_data, key=lambda x: x['value'], reverse=True)

        # extac
        names = [name_function(d) for d in filtered_data]
        values = [value_function(d) for d in filtered_data]


        # # debug
        # print(names)
        # print(values)
        # # print(filtered_data)
        
        # colors = [plt.cm.viridis(random.random()) for _ in range(len(names))]
        colors = [plt.cm.viridis(random.random(), alpha=0.2 if (data['type'].lower() == "grey") else 1.0) for data in filtered_data]

        # use colors_dict to determine the colors list
        if colors_dict:
            colors = [colors_dict[data['hardware']] for data in filtered_data]
            # add alpha=0.2 for grey
            colors = [color if (data['type'].lower() == "black") else (color[0], color[1], color[2], 0.2) for color, data in zip(colors, filtered_data)]

        # Create a bar plot
        plt.figure(figsize=(10, 5))
        # plt.bar(names, values, color='blue')
        plt.bar(names, values, color=colors)
        plt.title(title)
        plt.xlabel(xlabel)
        plt.ylabel(ylabel)
        plt.xticks(rotation=87)  # Rotate x-axis labels for better visibility

        # Save the figure and close
        plt.savefig(plotpath, bbox_inches='tight')
        plt.close()
        
        return plotpath


    @staticmethod
    def boxplot_plot_data(data, 
                          title,
                          xlabel,
                          ylabel,
                          name_function,
                          value_function, 
                          filter=None, 
                          unique=False,
                          sortedata=False, 
                          sortedatareverse=False,
                          filterout=None,
                          colors_dict=None):
        """
        Plot the data in a box plot and save it to a file.

        :param data: list of dicts with the data to plot
        :param title: title of the plot
        :param name_function: function to extract the name from the data
        :param value_function: function to extract the value from the data
        :param filter: function to filter the data
        :param unique: if True, only the most recent entry for each 'name', 'hardware', 'type', 'datasource' combination is plotted
        :param sortedata: if True, the data is sorted by value
        :param sortedatareverse: if True, the data is sorted by value in reverse order
        :param filterout: list of tuples (hardware, type) to filter out and not consider        
        """

        plotpath = '/tmp/report-boxplot' + title + '.png'

        # colors_dict
        if not colors_dict:
            colormap = plt.cm.tab20
            list_hardware = list(set([d['hardware'] for d in data]))
            colors = [colormap(i) for i in np.linspace(0, 1, len(list_hardware))]        
            colors_dict = dict(zip(list_hardware, colors))  # explicit dict to ensure order

        if filter:
            # Filter data using the provided function
            filtered_data = [d for d in data if filter(d)]
        else:
            filtered_data = data

        if filterout:
            # Filter out data using the provided list
            filtered_data = [d for d in filtered_data if (d['hardware'], d['type']) not in filterout]

        # order list using "value"
        if sortedata:
            filtered_data = sorted(filtered_data, key=lambda x: x['value'])
        if sortedatareverse:
            filtered_data = sorted(filtered_data, key=lambda x: x['value'], reverse=True)

        # Extract names and values
        names = [name_function(d) for d in filtered_data]
        values = [value_function(d) for d in filtered_data]

        # Group values by their corresponding names
        grouped_values = defaultdict(list)
        for n, v in zip(names, values):
            grouped_values[n].append(v)

        # # Sort names for better visualization
        # sorted_names = sorted(grouped_values.keys())

        # Compute average for each group
        averages = {name: np.mean(values) for name, values in grouped_values.items()}

        # Sort names by their average values
        sorted_names = sorted(grouped_values.keys(), key=lambda x: averages[x])

        # Extract grouped values in the order of sorted names
        box_data = [grouped_values[name] for name in sorted_names]

        # Create a box plot
        plt.figure(figsize=(10, 5))
        sns.set_style("whitegrid")  # Use seaborn's whitegrid style for better aesthetics
        bp = plt.boxplot(box_data, vert=True,  
                    patch_artist=True, 
                    labels=sorted_names,                     
                    )
        
        # Color boxes using colors_dict
        for patch, name in zip(bp['boxes'], sorted_names):
            hardware = name.split('⚫')[0].split('⚪')[0].strip()  # Extract hardware name from the label
            if hardware in colors_dict:
                patch.set_facecolor(colors_dict[hardware])
                # add alpha=0.2 for grey
                if name.endswith('⚪'):
                    patch.set_alpha(0.2)

        # Overlay individual data points on the boxplot
        for i, line_data in enumerate(box_data, start=1):  # start=1 because boxplot indices start at 1
            y = [i] * len(line_data)  # y-values are the same for a given box
            plt.plot(y, line_data, 'b.', alpha=0.3)  # 'r.' specifies red dots

        plt.title(title)
        plt.xlabel(xlabel)
        plt.ylabel(ylabel)
        plt.xticks(rotation=87)  # Rotate x-axis labels for better visibility

        # Save the figure and close
        plt.tight_layout()  # Ensure that labels and titles fit within the figure boundaries
        plt.gcf().set_size_inches(10, 5)
        plt.savefig(plotpath, bbox_inches='tight')
        # plt.savefig(plotpath, dpi=300)  # Adjust dpi as needed
        plt.close()        
        
        return plotpath

    @staticmethod
    def radar_plot_data(data, 
                  title,
                  xlabel,
                  ylabel,
                  name_function=plot_function_names,
                  value_function=plot_function_values, 
                  filter=None, 
                  unique=False,
                  sortedata=False, 
                  sortedatareverse=False,
                  filterout=None):
        """
        Produces a radar plot of the data saves it to a file.

        :param data: list of dicts with the data to plot
        :param title: title of the plot
        :param name_function: function to extract the name from the data
        :param value_function: function to extract the value from the data
        :param filter: function to filter the data
        :param unique: if True, only the most recent entry for each 'name', 'hardware', 'type', 'datasource' combination is plotted
        :param sortedata: if True, the data is sorted by value
        :param sortedatareverse: if True, the data is sorted by value in reverse order
        :param filterout: list of tuples (hardware, type) to filter out and not consider
        """

        plotpath = '/tmp/report-' + title + '-radar.png'
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
            
                # NOTE: condition 1: if not in dict or more recent than the one in the dict
                # if name not in filtered_dict or arrow.get(entry['timestampt'], ['D-M-YYYY', 'YYYY-MM-DD HH:mm:ss']).datetime > arrow.get(filtered_dict[name]['timestampt'], ['D-M-YYYY', 'YYYY-MM-DD HH:mm:ss']).datetime:

                # NOTE: condition 2: if not in dict or lower "value" than the one in the dict
                # not in filterout and greater then 0.001 (heuristic to remove outliers, close to zero)
                if (name not in filtered_dict or entry['value'] < filtered_dict[name]['value']) and (filterout is None or (entry['hardware'], entry['type']) not in filterout) and (entry['value'] > 0.001):
                    filtered_dict[name] = entry

            filtered_data = filtered_dict.values()            

        # order list using "value"
        if sortedata:
            filtered_data = sorted(filtered_data, key=lambda x: x['value'])
        if sortedatareverse:
            filtered_data = sorted(filtered_data, key=lambda x: x['value'], reverse=True)

        # extract
        names = [name_function(d) for d in filtered_data]
        values = [value_function(d) for d in filtered_data]

        # Calculate the number of variables
        num_vars = len(names)

        # Compute angle for each axis
        angles = np.linspace(0, 2 * np.pi, num_vars, endpoint=False).tolist()

        if len(angles) != len(values):
            print("WARNING: angles and values have different lengths for " + title)
            return
        elif len(angles) < 1:
            print("WARNING: angles and values have zero length for " + title)
            return

        # Radar plot
        fig, ax = plt.subplots(figsize=(6, 6), subplot_kw=dict(polar=True))

        ax.xaxis.grid(True, color="#888888", linestyle='solid', linewidth=0.5)
        ax.yaxis.grid(True, color="#888888", linestyle='dashed', linewidth=0.5)
        ax.spines["polar"].set_color("#222222")
        ax.set_facecolor("#FAFAFA")
        # Close the loop by connecting the start and end points            
        ax.plot([angles[0], angles[-1]], [values[0], values[-1]], linewidth=2, color='#4d4cf5') 

        # Plot the data
        ax.plot(angles, values, color='#4d4cf5', linewidth=2)
        ax.fill(angles, values, color='#4d4cf5', alpha=0.25)

        ax.fill(angles, values, color='#4d4cf5', alpha=0.25)
        ax.set_yticklabels([])
        ax.set_xticks(angles)                
        ax.set_xticklabels(names, fontsize=6)  # Adjust fontsize here        
        plt.title(title, size=15, color='black', y=1.1)  # Adjust title fontsize here
        ax.set_rlabel_position(30)

        plt.savefig(plotpath, bbox_inches='tight')
        plt.close()

        return plotpath

    @staticmethod
    def radar_plot_data_byid(data_dict, title, xlabel, ylabel, name_function=plot_function_names,
                                value_function=plot_function_values, filter=None, unique=False,
                                sortedata=False, sortedatareverse=False, filterout=None,
                                benchmark_ids=["a5", "a2", "a1"]):
        """
        Produces a radar plot with multiple areas corresponding to multiple data sets.

        :param data_dict: dict of data (each key corresponds with a list of dicts)
        :param title: title of the plot
        :param name_function: function to extract the name from the data
        :param value_function: function to extract the value from the data
        :param filter: function to filter the data
        :param unique: if True, only the most recent entry for each 'name', 'hardware', 'type', 'datasource' combination is plotted
        :param sortedata: if True, the data is sorted by value
        :param sortedatareverse: if True, the data is sorted by value in reverse order
        :param filterout: list of tuples (hardware, type) to filter out and not consider
        :param benchmark_ids: list of benchmark ids to plot
        """

        # Filter data for benchmarks tested
        filtered_data_dict = {key: data_dict[key] for key in benchmark_ids if key in data_dict}
        data_dict = filtered_data_dict        

        # Initialize the figure
        fig, ax = plt.subplots(figsize=(8, 8), subplot_kw=dict(polar=True))
        ax.xaxis.grid(True, color="#888888", linestyle='solid', linewidth=0.5)
        ax.yaxis.grid(True, color="#888888", linestyle='dashed', linewidth=0.5)
        ax.spines["polar"].set_color("#222222")
        ax.set_facecolor("#FAFAFA")
        plt.title(title, size=14, color='black', y=1.15)

        # filter out a common list of names, so that angles are the same for all datasets
        common_names_sets = [set(name_function(d) for d in data_dict[key]) for key in data_dict.keys()]
        common_names = set.intersection(*common_names_sets)

        if len(common_names) == 0:
            print("No common names found. Not producing radar plot for " + title + ", metric name: " + ylabel)
            return
        
        # # debug
        # print("common_names: ", common_names)
        max_value = max([entry['value'] for sublist in data_dict.values() for entry in sublist if name_function(entry) in common_names])
        # NOTE: this value must be updated afterwards, after all the filtering and sorting

        # Get a lits of colors
        # colormap = plt.cm.viridis
        colormap = plt.cm.tab10

        # Plots a radar plot with multiple areas, each corresponding to
        #     hardware:
        #         ['AMD Ryzen 5 PRO 4650G⚪',
        #         'AMD Ryzen 5 PRO 4650G⚫',
        #         ...
        #         'NVIDIA Jetson Nano⚫']
        #     values:
        #         [0.0011886436060327226,
        #         ...
        #         0.00771485232493554,
        #         0.007821293572182123]
        #     angles:
        #         [0.0,
        #         ...
        #         5.654866776461628]

        colors = [colormap(i) for i in np.linspace(0, 1, len(data_dict))]
        # Process each dataset in data_dict and plot it
        for idx, datakey in enumerate(data_dict.keys()):

            if filter:
                # Filter datakey using the provided function
                filtered_data = [d for d in data_dict[datakey] if filter(d)]
            else:
                filtered_data = data_dict[datakey]

            # re-filter for common names
            filtered_data = [entry for entry in filtered_data if name_function(entry) in common_names]

            if unique:
                # Use a dictionary to hold the most recent entry for each 'name', 
                # 'hardware', 'type', 'datasource' combination
                filtered_dict = {}
                for entry in filtered_data:
                    name = entry['name'] + entry['hardware'] + entry['type'] + entry['datasource']
                
                    # NOTE: condition 1: if not in dict or more recent than the one in the dict
                    # if name not in filtered_dict or arrow.get(entry['timestampt'], ['D-M-YYYY', 'YYYY-MM-DD HH:mm:ss']).datetime > arrow.get(filtered_dict[name]['timestampt'], ['D-M-YYYY', 'YYYY-MM-DD HH:mm:ss']).datetime:

                    # NOTE: condition 2: if not in dict or lower "value" than the one in the dict
                    # not in filterout and greater then 0.001 (heuristic to remove outliers, close to zero)
                    #
                    # IMPORTANT: This makes sense for latency and power, but not for throughput
                    # TODO: reverse this if for throughput
                    #
                    if (name not in filtered_dict or entry['value'] < filtered_dict[name]['value']) and (filterout is None or (entry['hardware'], entry['type']) not in filterout) and (entry['value'] > 0.001):
                        filtered_dict[name] = entry

                filtered_data = filtered_dict.values()            

            # order list using "hardware", to avoid Order of Processing issues
            if sortedata:
                filtered_data = sorted(filtered_data, key=lambda x: x['hardware'])
            if sortedatareverse:
                filtered_data = sorted(filtered_data, key=lambda x: x['hardware'], reverse=True)

            # extract
            # names = [name_function(d) for d in filtered_data]  # id
            names = [d["name"] for d in filtered_data]
            # Normalize values
            real_values = [value_function(d) for d in filtered_data]
            values = [value_function(d) / max_value for d in filtered_data]

            a = 500  # You can adjust this value
            real_values = [value_function(d) for d in filtered_data]
            values = [np.log(1 + a * (value_function(d) / max_value)) for d in filtered_data]

            # # Radar plot requires the data to be circular so append the first value to the end of the dataset
            # values += values[:1]
            # names += names[:1]            

            # Calculate the number of variables
            num_vars = len(names)

            # Compute angle for each axis
            angles = np.linspace(0, 2 * np.pi, num_vars, endpoint=False).tolist()

            if len(angles) != len(values):
                print("WARNING: angles and values have different lengths for " + title + " " + datakey)
                continue
            elif len(angles) < 1:
                print("WARNING: angles and values have zero length for " + title + " " + datakey)
                continue

            # pprint.pprint(names)
            # pprint.pprint(values)
            # pprint.pprint(angles)

            # Plot the data for this dataset
            current_color = colors[idx]
            ax.plot(angles, values, linewidth=2, label=datakey, color=current_color)
            ax.fill(angles, values, alpha=0.25, color=current_color)
            # Close the loop by connecting the start and end points
            ax.plot([angles[0], angles[-1]], [values[0], values[-1]], linewidth=2, color=current_color)                

            # Constants for the offsets. You can tune these for better aesthetics.
            offset_scale = 5
            # Annotate the points with their real values
            for angle, value, real_value in zip(angles, values, real_values):
                
                # Calculate x and y
                x = value * np.cos(angle)
                y = value * np.sin(angle)
                
                # Normalize x and y
                norm_x = x / np.sqrt(x**2 + y**2)
                norm_y = y / np.sqrt(x**2 + y**2)
                
                # Determine the offsets based on normalized x and y and the scale
                distancex = offset_scale * norm_x
                distancey = offset_scale * norm_y
                
                # Decide on horizontal alignment based on x
                ha = 'right' if x < 0 else 'left'
                
                ax.annotate(str(round(real_value, 1)), 
                            xy=(angle, value),
                            xytext=(distancex, distancey),  # proportional distance from the point
                            textcoords='offset points',
                            horizontalalignment=ha,
                            verticalalignment='center',
                            fontsize=6,
                            color=current_color)


        # Add legend
        # NOTE: using the last angles and names        
        
        # define y-ticks
        ytick_values = ax.get_yticks()
        # Use the inverse of the modified log function to get the original values for the y-ticks
        def inverse_modified_log(y, a=1000):
            return (np.exp(y) - 1) / a
        transformed_ytick_labels = [f'{inverse_modified_log(val, a) * max_value:.2f}' for val in ytick_values]
        ax.set_yticklabels(transformed_ytick_labels, fontsize=6)     

        ax.set_xticks(angles)
        ax.set_xticklabels(names, fontsize=6)
        ax.legend(loc='upper right', bbox_to_anchor=(1.1, 1.1))

        title_file = title.lower().replace(" ", "-")
        plotpath = '/tmp/report-' + title_file + '-multi-radar.png'
        plt.savefig(plotpath, bbox_inches='tight')
        plt.close()

        return plotpath

    @staticmethod
    def radar_plot_data_byhardware(data_dict, title, xlabel, ylabel, name_function=plot_function_names,
                                value_function=plot_function_values, filter=None, unique=False,
                                sortedata=False, sortedatareverse=False, filterout=None,
                                benchmark_ids=["a5", "a2", "a1"], benchmark_hardware=None, 
                                decimal_resolution=1, proportional_independent=False,
                                colors_dict=None, a=750):
        """
        Produces a radar plot with multiple areas corresponding to multiple data sets.

        :param data_dict: dict of data (each key corresponds with a list of dicts)
        :param title: title of the plot
        :param name_function: function to extract the name from the data
        :param value_function: function to extract the value from the data
        :param filter: function to filter the data
        :param unique: if True, only the most recent entry for each 'name', 'hardware', 'type', 'datasource' combination is plotted
        :param sortedata: if True, the data is sorted by value
        :param sortedatareverse: if True, the data is sorted by value in reverse order
        :param filterout: list of tuples (hardware, type) to filter out and not consider        
        :param benchmark_ids: list of benchmark ids to plot
        :param benchmark_hardware: list of benchmark hardware to plot
        :param decimal_resolution: number of decimal places to round the values to
        :param proportional_independent: if True, the values are normalized by the max value of the same type of benchmark (e.g., "c1" or "a2") and not by the max value of all benchmarks
        :param colors_dict: dict of colors to use for each hardware as key
        :param a: scaling factor for normalization
        """

        # Filter data for benchmarks tested
        filtered_data_dict = {key: data_dict[key] for key in benchmark_ids if key in data_dict}
        data_dict = filtered_data_dict        

        # Initialize the figure
        fig, ax = plt.subplots(figsize=(8, 8), subplot_kw=dict(polar=True))
        ax.xaxis.grid(True, color="#888888", linestyle='solid', linewidth=0.5)
        ax.yaxis.grid(True, color="#888888", linestyle='dashed', linewidth=0.5)
        ax.spines["polar"].set_color("#222222")
        ax.set_facecolor("#FAFAFA")
        plt.title(title, size=18, color='black', y=1.15)

        # filter out a common list of names, so that angles are the same for all datasets
        common_names_sets = [set(name_function(d) for d in data_dict[key]) for key in data_dict.keys()]
        common_names = set.intersection(*common_names_sets)

        if len(common_names) == 0:
            print("WARNING: No common names found. Not producing radar plot for " + title + ", metric name: " + ylabel)
            return
        
        # TODO: Implement the concepto of "proportionalindependent" normalization,
        # wherein the values are normalized by the max value of the same type of benchmark
        # (e.g., "c1" or "a2") and not by the max value of all benchmarks
        if proportional_independent:
            max_values = {}
            for id in benchmark_ids:
                max_values[id] = max([entry['value'] for sublist in data_dict.values() for entry in sublist
                                      if name_function(entry) in common_names and id in entry['id']])
                
        else:
            max_value = max([entry['value'] for sublist in data_dict.values() for entry in sublist if name_function(entry) in common_names])
                                

        # Get a lits of colors
        if colors_dict is None:
            common_names_sets = [set(d["hardware"] for d in data_dict[key]) for key in data_dict.keys()]
            list_hardware = list(set.intersection(*common_names_sets))            
            # colormap = plt.cm.tab10
            colormap = plt.cm.viridis
            colors = [colormap(i) for i in np.linspace(0, 1, len(list_hardware))]
            colors_dict = dict(zip(list_hardware, colors))  # explicit dict to ensure order

        # Plots a radar plot with multiple areas, each corresponding to
        #     benchmark id:
        #         ['a1',
        #         'a2',
        #         ...
        #         'a3']
        #     values:
        #         [0.0011886436060327226,
        #         ...
        #         0.00771485232493554,
        #         0.007821293572182123]
        #     angles:
        #         [0.0,
        #         ...
        #         5.654866776461628]                    

        # transform data_dict to have the hardware as keys
        merged_data_list = [item for sublist in data_dict.values() for item in sublist if name_function(item) in common_names]
        list_hardware = SummaryVerb.extract_unique_x(merged_data_list, "hardware")
        if benchmark_hardware is not None:
            list_hardware = [h for h in list_hardware if h in benchmark_hardware]
        
        if len(list_hardware) == 0:
            print("WARNING: No common hardware found. Not producing radar plot for " + title + ", metric name: " + ylabel)
            return            
            
        data_dict = defaultdict(list)
        for benchmark in merged_data_list:
            if benchmark["hardware"] in list_hardware:
                data_dict[benchmark["hardware"]].append(benchmark)

        # Process each dataset in data_dict and plot it
        # NOTE: datakey is the "hardware" value of each benchmark
        for idx, datakey in enumerate(data_dict.keys()):
            if filter:
                # Filter datakey using the provided function
                filtered_data = [d for d in data_dict[datakey] if filter(d)]
            else:
                filtered_data = data_dict[datakey]

            if unique:
                # Use a dictionary to hold the most recent entry for each 'name', 
                # 'hardware', 'datasource' combination
                # NOTE: does not consider "type"
                #
                filtered_dict = {}
                for entry in filtered_data:
                    name = entry['name'] + entry['hardware'] + entry['datasource']
                
                    # NOTE: condition 1: if not in dict or more recent than the one in the dict
                    # if name not in filtered_dict or arrow.get(entry['timestampt'], ['D-M-YYYY', 'YYYY-MM-DD HH:mm:ss']).datetime > arrow.get(filtered_dict[name]['timestampt'], ['D-M-YYYY', 'YYYY-MM-DD HH:mm:ss']).datetime:

                    # NOTE: condition 2: if not in dict or lower "value" than the one in the dict
                    # not in filterout and greater then 0.001 (heuristic to remove outliers, close to zero)
                    #
                    # IMPORTANT: This makes sense for latency and power, but not for throughput
                    # TODO: reverse this if for throughput
                    #
                    if (name not in filtered_dict or entry['value'] < filtered_dict[name]['value']) and (filterout is None or (entry['hardware'], entry['type']) not in filterout) and (entry['value'] > 0.001):
                        filtered_dict[name] = entry

                filtered_data = filtered_dict.values()
            
            # order list using "hardware", to avoid Order of Processing issues
            if sortedata:
                filtered_data = sorted(filtered_data, key=lambda x: x['id'])
            if sortedatareverse:
                filtered_data = sorted(filtered_data, key=lambda x: x['id'], reverse=True)

            # max_value = max([entry['value'] for entry in filtered_data])

            # extract
            # names = [ReportVerb.plot_function_id_withbenchtype(d) for d in filtered_data]    # NOTE this requires all
            #                                                                                # benchmarks to have all
            #                                                                                # types (grey, black)
            names = [d["name"] for d in filtered_data]
            
            
            # Normalize values and apply modified log transformation
            real_values = [value_function(d) for d in filtered_data]
            if proportional_independent:
                values = [np.log(1 + a * (value_function(d) / max_values[d["id"]])) for d in filtered_data]
            else:
                values = [np.log(1 + a * (value_function(d) / max_value)) for d in filtered_data]
                
            # Calculate the number of variables
            num_vars = len(names)

            # Compute angle for each axis
            angles = np.linspace(0, 2 * np.pi, num_vars, endpoint=False).tolist()
            
            if len(angles) != len(values):
                print("WARNING: angles and values have different lengths for " + title + ", " + datakey + ", " + ylabel)
                continue
            elif len(angles) < 1:
                print("WARNING: angles and values have zero length for " + title + ", " + datakey + ", " + ylabel)
                continue

            ## debug
            # pprint.pprint(names)
            # pprint.pprint(values)
            # pprint.pprint(angles)

            # Plot the data for this dataset
            current_color = colors_dict[datakey]
            ax.plot(angles, values, linewidth=2, label=datakey, color=current_color, alpha=0.7)
            ax.fill(angles, values, alpha=0.2, color=current_color)
            # Close the loop by connecting the start and end points            
            ax.plot([angles[0], angles[-1]], [values[0], values[-1]], linewidth=2, color=current_color)                
            # Add dots to each data point
            # ax.scatter(angles, values, color=current_color, s=50)  # You can adjust the size (s) as desired
            ax.scatter(angles, values, color=current_color, s=50, edgecolors='black', linewidths=0.5)

            # Constants for the offsets. You can tune these for better aesthetics.
            offset_scale = 5
            # Annotate the points with their real values
            for angle, value, real_value in zip(angles, values, real_values):
                
                # Calculate x and y
                x = value * np.cos(angle)
                y = value * np.sin(angle)
                
                # Normalize x and y
                norm_x = x / np.sqrt(x**2 + y**2)
                norm_y = y / np.sqrt(x**2 + y**2)
                
                # Determine the offsets based on normalized x and y and the scale
                distancex = offset_scale * norm_x
                distancey = offset_scale * norm_y
                
                # Decide on horizontal alignment based on x
                ha = 'right' if x < 0 else 'left'
                
                ax.annotate(str(round(real_value, decimal_resolution)), 
                            xy=(angle, value),
                            xytext=(distancex, distancey),  # proportional distance from the point
                            textcoords='offset points',
                            horizontalalignment=ha,
                            verticalalignment='center',
                            fontsize=12,
                            color=current_color)

        # Add legend

        # # highlighting
        # ## highlighting the group
        # highlighted_axes_indices = [0, 2]  # example indices of axes you want to highlight        
        # label_angle = np.mean([angles[i] for i in highlighted_axes_indices])
        # ax.annotate('Perception benchmarks', xy=(label_angle, ax.get_ylim()[1]*0.8), ha='center', color='red')

        # ## highlighting the axes
        # highlighted_axes_angles = [angles[0], angles[2]]  # replace with the angles of the axes you want to highlight
        # for angle in highlighted_axes_angles:
        #     # Create a line at the specified angle
        #     line = plt.Line2D([0, np.cos(angle) * ax.get_ylim()[1]], 
        #                     [0, np.sin(angle) * ax.get_ylim()[1]], 
        #                     transform=ax.transData._b, 
        #                     color='red', 
        #                     linewidth=2)  # or any other color
        #     ax.add_line(line)

        # ## highlighting the group
        # highlighted_axes_indices = [3, 5]  # example indices of axes you want to highlight        
        # label_angle = np.mean([angles[i] for i in highlighted_axes_indices])
        # ax.annotate('Control benchmarks', xy=(label_angle, ax.get_ylim()[1]*0.8), ha='center', color='blue')

        # ## highlighting the axes
        # highlighted_axes_angles = [angles[3], angles[5]]  # replace with the angles of the axes you want to highlight
        # for angle in highlighted_axes_angles:
        #     # Create a line at the specified angle
        #     line = plt.Line2D([0, np.cos(angle) * ax.get_ylim()[1]], 
        #                     [0, np.sin(angle) * ax.get_ylim()[1]], 
        #                     transform=ax.transData._b, 
        #                     color='blue', 
        #                     linewidth=2)  # or any other color
        #     ax.add_line(line)

        # ## highlighting the area
        # # TODO: implement this

        # define y-ticks
        ytick_values = ax.get_yticks()
        # Use the inverse of the modified log function to get the original values for the y-ticks
        def inverse_modified_log(y, a=1000):
            return (np.exp(y) - 1) / a
        if proportional_independent:
            transformed_ytick_labels = [f'{inverse_modified_log(val, a) * max_values[d["id"]]:.2f}' for val in ytick_values]
        else:
            transformed_ytick_labels = [f'{inverse_modified_log(val, a) * max_value:.2f}' for val in ytick_values]
        ax.set_yticklabels(transformed_ytick_labels, fontsize=12)

        # define x-ticks
        ax.set_xticks(angles)
        ax.set_xticklabels(names, fontsize=12)
        ax.legend(loc='upper right', bbox_to_anchor=(1.1, 1.1), fontsize=12)

        # title and save
        title_file = title.lower().replace(" ", "-")
        plotpath = '/tmp/report-' + title_file + '-multi-radar.png'
        plt.savefig(plotpath, bbox_inches='tight')
        plt.close()

        # grid style
        ax.xaxis.grid(True, color="#888888", linestyle='dashed', linewidth=1.5)
        ax.yaxis.grid(True, color="#888888", linestyle='dashed', linewidth=1.5)

        #background color
        ax.set_facecolor('white')

        # # Hide Spines, outer circular spine
        # ax.spines["polar"].set_visible(False)

        return plotpath

    @staticmethod
    def plot_color_coding(colors_dict):
        # Extract hardware names
        hardware_names = list(colors_dict.keys())

        # Create a new list of hardware_name adding the ⚫ and ⚪ (add both) symbols at the end of each name
        # this will duplicate the number of hardware names
        hardware_names = [hardware_name + '⚫' for hardware_name in hardware_names] + [hardware_name + '⚪' for hardware_name in hardware_names]

        # Create a figure and axis
        fig, ax = plt.subplots(figsize=(len(hardware_names) * 0.5, 6))  # Adjusting the width based on number of hardware items

        # For each hardware, plot a colored bar
        for i, hardware in enumerate(hardware_names):
            color = colors_dict[hardware[:-1]]
            alpha_val = 0.2 if '⚪' in hardware else 1  # Adjust alpha for grey boxed solutions
            ax.bar(i, 1, color=color, alpha=alpha_val)  # Setting height to 1 as it's just for visualization

        # Set x-ticks and labels
        ax.set_xticks(range(len(hardware_names)))
        ax.set_xticklabels(hardware_names, rotation=90)  # Rotate x-axis labels for better visibility

        # Remove y-axis
        ax.get_yaxis().set_visible(False)

        # Set title
        ax.set_title("Color Coding per Hardware")

        # Remove spines
        for spine in ax.spines.values():
            spine.set_visible(False)

        plt.tight_layout()
        plotpath = '/tmp/report-color-coding.png'
        plt.savefig(plotpath)
        plt.close()

        return plotpath



# #####################################################
# #####################################################
# #####################################################
# ##################################################### 

    def main(self, *, args):
        # get paths of "benchmark.yaml" files for each benchmark
        benchmark_meta_paths = search_benchmarks()
        list_results = SummaryVerb.preprocess(benchmark_meta_paths)
        list_ids = SummaryVerb.extract_unique_x(list_results, "id")
        alphabetical_list_ids = sorted(list_ids)

        # pprint.pprint(list_results)

        benchmark_id_report = ""
        # Add a title to the report indicating a timestamp of when it was generated
        benchmark_id_report += f"# [RobotPerf](https://robotperf.org/) report (generated on `{arrow.now().format('YYYY-MM-DD HH:mm:ss')}`)\n\n"

        # Generate a Table of Contents
        benchmark_id_report += "## Table of Contents\n"
        benchmark_id_report += "- [Intro](#intro)\n"
        benchmark_id_report += "- [Legend](#legend)\n"
        benchmark_id_report += "- [Summarized results](#summarized-results)\n"
        benchmark_id_report += "- [Benchmark results by `id`](#benchmark-results-by-id)\n"
        for benchmark_id in alphabetical_list_ids:
            benchmark_id_report += "  - [Benchmark `{}`](#benchmark-{})\n".format(benchmark_id, benchmark_id)
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
        # unique condition
        unique_condition = True
        
        # /////////////////////////////////////////////////
        benchmark_id_report += "## Summarized results\n"

        # group by metric
        latency_data_dict = {}
        for benchmark_id in alphabetical_list_ids:
            filtered_data = [item for item in list_results if (item['id'] == benchmark_id and item['metric'] == "latency")]
            sorted_filtered_data = sorted(filtered_data, key=lambda x: x['value'])
            latency_data_dict[benchmark_id] = sorted_filtered_data

        throughput_data_dict = {}
        for benchmark_id in alphabetical_list_ids:
            filtered_data = [item for item in list_results if (item['id'] == benchmark_id and item['metric'] == "throughput")]
            sorted_filtered_data = sorted(filtered_data, key=lambda x: x['value'])
            throughput_data_dict[benchmark_id] = sorted_filtered_data

        power_data_dict = {}
        for benchmark_id in alphabetical_list_ids:
            filtered_data = [item for item in list_results if (item['id'] == benchmark_id and item['metric'] == "power")]
            sorted_filtered_data = sorted(filtered_data, key=lambda x: x['value'])
            power_data_dict[benchmark_id] = sorted_filtered_data

        metric_names = ["Latency (ms)", "Throughput (FPS)", "Power (W)"]
        metric_groups = [latency_data_dict, throughput_data_dict, power_data_dict]

        # create category groups
        perception_group = ["a1", "a2", "a5"]
        localization_group = ["b1", "b2", "b3"]
        control_group = ["c1", "c2", "c3", "c4", "c5"]
        # manipulation_group = ["d1", "d2", "d3", "d4", "d5", "d6"]
        manipulation_group = ["d2", "d3", "d4", "d5", "d6"]

        category_groups = [perception_group, localization_group, control_group, manipulation_group]
        category_names = ["Perception", "Localization", "Control", "Manipulation"]


        # colors
        list_hardware = SummaryVerb.extract_unique_x(list_results, "hardware") # find all unique "hardware" values across benchmarks
        list_hardware = sorted(list_hardware)
        # colormap = plt.cm.tab10
        # colormap = plt.cm.viridis
        # colormap = plt.cm.bwr
        # colormap = plt.cm.seismic
        colormap = plt.cm.tab20
        colors = [colormap(i) for i in np.linspace(0, 1, len(list_hardware))]        
        colors_dict = dict(zip(list_hardware, colors))  # explicit dict to ensure order

        colorcoding_path = ReportVerb.plot_color_coding(colors_dict)
        benchmark_id_report += f"\n![{colorcoding_path}]({colorcoding_path})\n"            

        # header of table
        benchmark_id_report_aux = "|   | " + " | ".join("{{}}".format() for _ in category_names) + " |\n"        
        benchmark_id_report_aux += "|---| " + " | ".join("---" for _ in category_names) + " |\n"
        benchmark_id_report_aux = benchmark_id_report_aux.format(*category_names)
        benchmark_id_report += benchmark_id_report_aux

        for metric_name, metric_group in zip(metric_names, metric_groups):
            benchmark_id_report += "| " + str(metric_name) + " | "
            for idx, category in enumerate(category_groups):
                filter_out = [("Kria KR260", "black"), ("Kria KV260", "black")]
                # plotpath = (ReportVerb.radar_plot_data_byid(metric_group,
                #                                     xlabel="Hardware (acceleration kernel)",
                #                                     ylabel=metric_name,
                #                                     name_function=ReportVerb.plot_function_names_forid,
                #                                     value_function=ReportVerb.plot_function_values,
                #                                     title="RobotPerf benchmarking (by hardware) " + category_names[idx] + " " + metric_name,
                #                                     unique=unique_condition,
                #                                     sortedata=True,
                #                                     filterout = filter_out,
                #                                     benchmark_ids=category))
                # benchmark_id_report += f"\n![{plotpath}]({plotpath})\n"

                plotpath = (ReportVerb.radar_plot_data_byhardware(metric_group,
                                                    xlabel="Hardware (acceleration kernel)",
                                                    ylabel=metric_name,
                                                    name_function=ReportVerb.plot_function_names_forid,
                                                    value_function=ReportVerb.plot_function_values,
                                                    title="Benchmarking Robotics " + category_names[idx] + " " + metric_name,
                                                    unique=unique_condition,
                                                    sortedata=True,
                                                    filterout = filter_out,
                                                    benchmark_ids=category,
                                                    decimal_resolution=2,
                                                    proportional_independent=False,
                                                    # benchmark_hardware=['AMD Ryzen 5 PRO 4650G',
                                                    #                     'Intel i7-8700K',
                                                    #                     'NVIDIA AGX Orin Dev. Kit',
                                                    #                     'Kria KR260'],
                                                    # benchmark_hardware=['Kria KR260 (ROBOTCORE® Perception)',
                                                    #                     'Kria KR260'],
                                                    colors_dict=colors_dict,
                                                    a = 500
                                                    ))
                if plotpath:
                    benchmark_id_report += f"![{plotpath}]({plotpath}) | "
                else:
                    benchmark_id_report += f" | "
            benchmark_id_report += f"\n"

        # sys.exit(0)

        # /////////////////////////////////////////////////
        benchmark_id_report += "## Benchmark results by `id`\n"
        for benchmark_id in alphabetical_list_ids:            
            benchmark_id_report += f"### Benchmark `{benchmark_id}`\n"

            # ## all of it (metric-agnostic)
            # # get body
            # filtered_data = [item for item in list_results if item['id'] == benchmark_id]
            # benchmark_id_report += SummaryVerb.to_markdown_table(filtered_data, benchmark_id)
            # # plot
            # plotpath = (ReportVerb.plot_data(filtered_data, 
            #                                 name_function=ReportVerb.plot_function_names_forid,
            #                                 value_function=ReportVerb.plot_function_values,
            #                                 title=benchmark_id, 
            #                                 unique=False))
            # benchmark_id_report += f"\n![{plotpath}]({plotpath})"


            ## ⏱ latency
            filter_out = [("Kria KR260", "black"), ("Qualcomm RB5 Robotics Kit", "black")]
            filtered_data = [item for item in list_results if (item['id'] == benchmark_id and item['metric'] == "latency")]
            sorted_filtered_data = sorted(filtered_data, key=lambda x: x['value'])
            plotpath = (ReportVerb.plot_data(sorted_filtered_data,
                                             xlabel="Hardware (acceleration kernel)",
                                             ylabel="Latency (ms)",
                                             name_function=ReportVerb.plot_function_names_forid,
                                             value_function=ReportVerb.plot_function_values,
                                             title=benchmark_id + "-latency",
                                             unique=unique_condition,
                                             sortedata=True,
                                             filterout = filter_out,
                                             colors_dict=colors_dict))
            benchmark_id_report += f"\n![{plotpath}]({plotpath})\n"            
            plotpath = (ReportVerb.boxplot_plot_data(sorted_filtered_data,
                                             xlabel="Hardware (acceleration kernel)",
                                             ylabel="Latency (ms)",
                                             name_function=ReportVerb.plot_function_names_forid,
                                             value_function=ReportVerb.plot_function_values,
                                             title=benchmark_id + "-latency",
                                             unique=unique_condition,
                                             sortedata=True,
                                             filterout = filter_out,
                                             colors_dict=colors_dict))            
            benchmark_id_report += f"\n![{plotpath}]({plotpath})\n"

            plotpath_radar = (ReportVerb.radar_plot_data(sorted_filtered_data,
                                             xlabel="Hardware (acceleration kernel)",
                                             ylabel="Latency (ms)",
                                             name_function=ReportVerb.plot_function_names_forid,
                                             value_function=ReportVerb.plot_function_values,
                                             title=benchmark_id + "-latency",
                                             unique=unique_condition,
                                             sortedata=True,
                                             filterout = filter_out))
            benchmark_id_report += f"\n![{plotpath_radar}]({plotpath_radar})\n"

            benchmark_id_report += SummaryVerb.to_markdown_table(sorted_filtered_data, 
                                                                 benchmark_id+"-latency",
                                                                 unique=unique_condition,
                                                                 sortedata=True,
                                                                 filterout = filter_out)

            ## ⚡ power
            filter_out = []
            filtered_data = [item for item in list_results if (item['id'] == benchmark_id and item['metric'] == "power")]
            sorted_filtered_data = sorted(filtered_data, key=lambda x: x['value'])
            plotpath = (ReportVerb.plot_data(sorted_filtered_data,
                                             xlabel="Hardware (acceleration kernel)",
                                             ylabel="Power (W)",
                                             name_function=ReportVerb.plot_function_names_forid,
                                             value_function=ReportVerb.plot_function_values,
                                             title=benchmark_id + "-power",
                                             unique=unique_condition,
                                             sortedata=True,
                                             filterout = filter_out,
                                             colors_dict=colors_dict))
            # plotpath = (ReportVerb.boxplot_plot_data(sorted_filtered_data,
            #                                             xlabel="Hardware (acceleration kernel)",
            #                                             ylabel="Power (W)",
            #                                             name_function=ReportVerb.plot_function_names_forid,
            #                                             value_function=ReportVerb.plot_function_values,
            #                                             title=benchmark_id + "-power",
            #                                             unique=unique_condition,
            #                                             sortedata=True,
            #                                             filterout = filter_out,
            #                                             colors_dict=colors_dict))            
            benchmark_id_report += f"\n![{plotpath}]({plotpath})\n"

            plotpath_radar = (ReportVerb.radar_plot_data(sorted_filtered_data,
                                             xlabel="Hardware (acceleration kernel)",
                                             ylabel="Power (W)",
                                             name_function=ReportVerb.plot_function_names_forid,
                                             value_function=ReportVerb.plot_function_values,
                                             title=benchmark_id + "-power",
                                             unique=unique_condition,
                                             sortedata=True,
                                             filterout = filter_out))
            benchmark_id_report += f"\n![{plotpath_radar}]({plotpath_radar})\n"

            benchmark_id_report += SummaryVerb.to_markdown_table(sorted_filtered_data, 
                                                                 benchmark_id+"-power",
                                                                 unique=unique_condition,
                                                                 sortedata=True,
                                                                 filterout = filter_out)

            ## 📶 throughput
            filter_out = []
            filtered_data = [item for item in list_results if (item['id'] == benchmark_id and item['metric'] == "throughput")]
            sorted_filtered_data = sorted(filtered_data, key=lambda x: x['value'], reverse=True)
            plotpath = (ReportVerb.plot_data(sorted_filtered_data,
                                             xlabel="Hardware (acceleration kernel)",
                                             ylabel="Throughput (FPS)",
                                             name_function=ReportVerb.plot_function_names_forid,
                                             value_function=ReportVerb.plot_function_values,
                                             title=benchmark_id + "-throughput",
                                             unique=unique_condition,
                                             sortedatareverse=True,
                                             filterout = filter_out,
                                             colors_dict=colors_dict))
            # plotpath = (ReportVerb.boxplot_plot_data(sorted_filtered_data,
            #                                             xlabel="Hardware (acceleration kernel)",
            #                                             ylabel="Throughput (FPS)",
            #                                             name_function=ReportVerb.plot_function_names_forid,
            #                                             value_function=ReportVerb.plot_function_values,
            #                                             title=benchmark_id + "-throughput",
            #                                             unique=unique_condition,
            #                                             sortedatareverse=True,
            #                                             filterout = filter_out,
            #                                             colors_dict=colors_dict))            
            benchmark_id_report += f"\n![{plotpath}]({plotpath})\n"

            plotpath_radar = (ReportVerb.radar_plot_data(sorted_filtered_data,
                                             xlabel="Hardware (acceleration kernel)",
                                             ylabel="Throughput (FPS)",
                                             name_function=ReportVerb.plot_function_names_forid,
                                             value_function=ReportVerb.plot_function_values,
                                             title=benchmark_id + "-throughput",
                                             unique=unique_condition,
                                             sortedatareverse=True,
                                             filterout = filter_out))
            benchmark_id_report += f"\n![{plotpath_radar}]({plotpath_radar})\n"

            benchmark_id_report += SummaryVerb.to_markdown_table(sorted_filtered_data, 
                                                                 benchmark_id+"-throughput",
                                                                 unique=unique_condition,
                                                                 sortedatareverse=True,
                                                                 filterout = filter_out)

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
