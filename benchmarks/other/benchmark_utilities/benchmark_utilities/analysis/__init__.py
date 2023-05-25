# Copyright (C) Acceleration Robotics S.L.U. - All Rights Reserved
#
# Written by Víctor Mayoral Vilches <victor@accelerationrobotics.com>
# Written by Martiño Crespo <martinho@accelerationrobotics.com>
# Written by Alejandra Martínez Fariña <alex@accelerationrobotics.com>
# Licensed under the Apache License, Version 2.0

import bt2
import sys
import datetime
import os
import pandas as pd
import numpy as np
import pprint
import subprocess
import yaml
import time
from tabnanny import verbose
from turtle import width
from wasabi import color
from typing import List, Optional, Tuple, Union
from ros2benchmark.verb import VerbExtension, Benchmark, run, search_benchmarks
from bokeh.plotting.figure import figure, Figure
from bokeh.plotting import output_notebook, save, output_file
from bokeh.io import show, export_png
from bokeh.layouts import row
from bokeh.models import (
    ColumnDataSource,
    DatetimeTickFormatter,
    PrintfTickFormatter,
    Legend,
    Segment,
)
from bokeh.models.annotations import Label

# color("{:02x}".format(x), fg=16, bg="green")
# debug = True  # debug flag, set to True if desired


class BenchmarkAnalyzer:
    def __init__(self, benchmark_name, hardware_device_type="cpu"):
        self.benchmark_name = benchmark_name
        self.hardware_device_type = hardware_device_type

        # initialize arrays where tracing configuration will be stored
        self.target_chain = []
        self.target_chain_dissambiguous = []
        self.target_chain_colors_fg = []
        self.target_chain_colors_fg_bokeh = []
        self.target_chain_layer = []
        self.target_chain_label_layer = []
        self.target_chain_marker = []
        self.lost_msgs = 0  # lost messages counter, target_chain not fully met

    def add_target(self, target_dict):
        # targeted chain of messages for tracing
        # NOTE: there're not "publish" tracepoints because
        # graph's using inter-process communications

        self.target_chain.append(target_dict["name"])
        self.target_chain_dissambiguous.append(target_dict["name_disambiguous"])
        self.target_chain_colors_fg.append(target_dict["colors_fg"])
        self.target_chain_colors_fg_bokeh.append(target_dict["colors_fg_bokeh"])
        self.target_chain_layer.append(target_dict["layer"])
        self.target_chain_label_layer.append(target_dict["label_layer"])
        self.target_chain_marker.append(target_dict["marker"])

    def get_change(self, first, second):
        """
        Get change in percentage between two values
        """
        if first == second:
            return 0
        try:
            return (abs(first - second) / second) * 100.0
        except ZeroDivisionError:
            return float("inf")

    def add_durations_to_figure(
        self, 
        figure: Figure,
        segment_type: str,
        durations: List[Union[Tuple[datetime.datetime, datetime.datetime]]],
        color: str,
        line_width: int = 60,
        legend_label: Optional[str] = None,
    ) -> None:
        for duration in durations:
            duration_begin, duration_end, _ = duration
            base_kwargs = dict()
            if legend_label:
                base_kwargs["legend_label"] = legend_label
            figure.line(
                x=[duration_begin, duration_end],
                y=[segment_type, segment_type],
                color=color,
                line_width=line_width,
                **base_kwargs,
            )

    def add_markers_to_figure(
        self, 
        figure: Figure,
        segment_type: str,
        times: List[datetime.datetime],
        color: str,
        line_width: int = 60,
        legend_label: Optional[str] = None,
        size: int = 30,
        marker_type: str = "diamond",
    ) -> None:
        for time in times:
            base_kwargs = dict()
            if legend_label:
                base_kwargs["legend_label"] = legend_label
            if marker_type == "diamond":
                figure.diamond(
                    x=[time],
                    y=[segment_type],
                    fill_color=color,
                    line_color=color,
                    size=size,
                    **base_kwargs,
                )
            elif marker_type == "plus":
                figure.plus(
                    x=[time],
                    y=[segment_type],
                    fill_color=color,
                    line_color=color,
                    size=size,
                    **base_kwargs,
                )
            else:
                assert False, "invalid marker_type value"


    def msgsets_from_ctf_vtf_traces(self, ctf_trace, vtf_trace, debug=False):
        """
        Returns a list of message sets ready to be used
        for plotting them in various forms. Takes two inputs,
        corresponding with the absolute paths to a CTF and and 
        VTF (CTF format).

        NOTE: NOT coded for multiple Nodes running concurrently or multithreaded executors
        Classification expects events in the corresponding order.
        """

        msg_it = bt2.TraceCollectionMessageIterator(ctf_trace)
        # Iterate the trace messages and pick right ones
        ctf_msgs = []
        for msg in msg_it:
            # `bt2._EventMessageConst` is the Python type of an event message.
            if type(msg) is bt2._EventMessageConst:
                # An event message holds a trace event.
                event = msg.event
                # Only check `sched_switch` events.
                if event.name in self.target_chain:
                    ctf_msgs.append(msg)

        msg_it = bt2.TraceCollectionMessageIterator(vtf_trace)
        # Iterate the trace messages and pick right ones
        vtf_msgs = []
        for msg in msg_it:
            # `bt2._EventMessageConst` is the Python type of an event message.
            if type(msg) is bt2._EventMessageConst:
                # An event message holds a trace event.
                event = msg.event
                # Only check `sched_switch` events.
                if event.name in self.target_chain:
                    vtf_msgs.append(msg)

        all_msgs = ctf_msgs + vtf_msgs
        all_msgs_sorted = sorted(all_msgs, key= lambda x: x.default_clock_snapshot.ns_from_origin)

        # Form sets with each pipeline
        image_pipeline_msg_sets = []
        new_set = []  # used to track new complete sets
        chain_index = 0  # track where in the chain we are so far
        vpid_chain = -1  # used to track a set and differentiate from other callbacks

        # NOTE: NOT CODED FOR MULTIPLE NODES RUNNING CONCURRENTLY
        # this classification is going to miss the initial matches because
        # "ros2:callback_start" will not be associated with the target chain and it won't stop
        # being considered until a "ros2:callback_end" of that particular process is seen
        for index in range(len(all_msgs_sorted)):
            if all_msgs_sorted[index].event.name in self.target_chain:  # optimization

                # print("new: " + all_msgs_sorted[index].event.name)
                # print("expected: " + str(self.target_chain[chain_index]))
                # print("chain_index: " + str(chain_index))
                # print("---")

                if debug:
                    print("---")
                    print("new: " + all_msgs_sorted[index].event.name)
                    print("expected: " + str(self.target_chain[chain_index]))
                    print("chain_index: " + str(chain_index))

                # first one
                if (
                    chain_index == 0
                    and all_msgs_sorted[index].event.name == self.target_chain[chain_index]
                ):
                    new_set.append(all_msgs_sorted[index])
                    # vpid_chain = all_msgs_sorted[index].event.common_context_field.get(
                    #     "vpid"
                    # )
                    chain_index += 1
                    if debug:
                        print(color("Found first: " + str(all_msgs_sorted[index].event.name) + " - " + str([x.event.name for x in new_set]), fg="blue"))
                # last one
                elif (
                    all_msgs_sorted[index].event.name == self.target_chain[chain_index]
                    and self.target_chain[chain_index] == self.target_chain[-1]
                    and new_set[-1].event.name == self.target_chain[-2]
                    # and all_msgs_sorted[index].event.common_context_field.get("vpid")
                    # == vpid_chain
                ):
                    new_set.append(all_msgs_sorted[index])
                    image_pipeline_msg_sets.append(new_set)
                    if debug:
                        print(color("Found last: " + str(all_msgs_sorted[index].event.name) + " - " + str([x.event.name for x in new_set]), fg="blue"))
                    chain_index = 0  # restart
                    new_set = []  # restart
                # match
                elif (
                    all_msgs_sorted[index].event.name == self.target_chain[chain_index]
                    # and all_msgs_sorted[index].event.common_context_field.get("vpid")
                    # == vpid_chain
                ):
                    new_set.append(all_msgs_sorted[index])
                    chain_index += 1
                    if debug:
                        print(color("Found: " + str(all_msgs_sorted[index].event.name) + " - " + str([x.event.name for x in new_set]), fg="green"))
                # altered order
                elif (
                    all_msgs_sorted[index].event.name in self.target_chain
                    # and all_msgs_sorted[index].event.common_context_field.get("vpid")
                    # == vpid_chain
                ):
                    # pop ros2:callback_start in new_set, if followed by "ros2:callback_end"
                    # NOTE: consider case of disconnected series of:
                    #       "ros2:callback_start"
                    #       "ros2:callback_end"
                    if (all_msgs_sorted[index].event.name == "ros2:callback_end"
                        and self.target_chain[chain_index - 1] == "ros2:callback_start"):
                        new_set.pop()
                        chain_index -= 1
                    else:
                        new_set.append(all_msgs_sorted[index])
                        if debug:
                            print(color("Altered order: " + str([x.event.name for x in new_set]) + ", restarting", fg="red"))
                        chain_index = 0  # restart
                        new_set = []  # restart

        # print(len(image_pipeline_msg_sets))
        return image_pipeline_msg_sets


    def timestamp_identifier(self, msg):
        """
        Returns ROS message header timestamp as unique identifier
        from a CTF msg
        """        
        id_nanosecs = msg.event.payload_field["image_input_header_nsec"]
        id_sec = msg.event.payload_field["image_input_header_sec"]

        id = id_sec + id_nanosecs/1e9
        return id

    def msgsets_from_trace_identifier(
        self, 
        tracename, 
        unique_funq=None, 
        debug=False
    ):
        """
        Returns a list of message sets ready to be used
        for plotting them in various forms. Uses unique identifer
        function to 

        NOTE: A different implementation than msgsets_from_trace and which
        allows determining messages dropped or not propagated appropriately.

        Args:
            tracename (string): path for the trace file
            debug (bool, optional): [description]. Defaults to False.

        """
        if unique_funq is None:
            unique_funq = self.timestamp_identifier

        msg_it = bt2.TraceCollectionMessageIterator(tracename)

        # Iterate the trace messages and pick ros2 ones
        image_pipeline_msgs = []
        for msg in msg_it:
            # `bt2._EventMessageConst` is the Python type of an event message.
            if type(msg) is bt2._EventMessageConst:
                # An event message holds a trace event.
                event = msg.event
                # Only check `sched_switch` events.
                # if "ros2" in event.name or "robotperf" in event.name:
                if event.name in self.target_chain:
                    image_pipeline_msgs.append(msg)

        # Form sets with each pipeline
        image_pipeline_msg_dict = {}
        new_set = []  # used to track new complete sets
        chain_index = 0  # track where in the chain we are so far

        for msg in image_pipeline_msgs:
            id = unique_funq(msg)
            if id in image_pipeline_msg_dict.keys():
                if (len(image_pipeline_msg_dict[id]) < len(self.target_chain)):
                    image_pipeline_msg_dict[id].append(msg)
                else:
                    pass
                    if debug:
                        print(color("Message with id: " + str(id) + " already fully propagated, discarding - " + str(msg.event.name), fg="yellow"))
            else:
                image_pipeline_msg_dict[id] = [msg]


        del_list = []
        for key_id, value_list in image_pipeline_msg_dict.items():
            names_value_list = [msg.event.name for msg in value_list]
            if len(value_list) != len(self.target_chain):
                if debug:
                    print(color("Message with id: " + str(key_id) + " not fully propagated (missing number), discarding chain - " + str([x.event.name for x in value_list]), fg="orange"))
                # del image_pipeline_msg_dict[key_id]  # this leads to error:
                #                                      # dictionary changed size during iteration
                del_list.append(key_id)
                continue
            if not all(item in names_value_list for item in self.target_chain):
                if debug:
                    print(color("Message with id: " + str(key_id) + " does not have all tracepoints, discarding chain - " + str([x.event.name for x in value_list]), fg="red"))
                del_list.append(key_id)
        for key in del_list:
            del image_pipeline_msg_dict[key]
            self.lost_msgs += 1

        # survivors
        return list(image_pipeline_msg_dict.values())

    def msgsets_from_trace(self, tracename, debug=False):
        """
        Returns a list of message sets ready to be used
        for plotting them in various forms.

        NOTE: A brute-force implementation. Brings various issues when
        facing concurrent setups and/or machines with less capabilities.

        NOTE: NOT coded for multiple Nodes running concurrently or multithreaded executors
        Classification expects events in the corresponding order.
        """
        # Create a trace collection message iterator from the first command-line
        # argument.
        msg_it = bt2.TraceCollectionMessageIterator(tracename)

        # Iterate the trace messages and pick ros2 ones
        image_pipeline_msgs = []
        for msg in msg_it:
            # `bt2._EventMessageConst` is the Python type of an event message.
            if type(msg) is bt2._EventMessageConst:
                # An event message holds a trace event.
                event = msg.event
                # Only check `sched_switch` events.
                # if "ros2" in event.name or "robotperf" in event.name:
                if event.name in self.target_chain:
                    image_pipeline_msgs.append(msg)

        # Form sets with each pipeline
        image_pipeline_msg_sets = []
        new_set = []  # used to track new complete sets
        chain_index = 0  # track where in the chain we are so far
        vpid_chain = -1  # used to track a set and differentiate from other callbacks

        # NOTE: NOT CODED FOR MULTIPLE NODES RUNNING CONCURRENTLY
        # this classification is going to miss the initial matches because
        # "ros2:callback_start" will not be associated with the target chain and it won't stop
        # being considered until a "ros2:callback_end" of that particular process is seen
        for index in range(len(image_pipeline_msgs)):
            if image_pipeline_msgs[index].event.name in self.target_chain:  # optimization

                if debug:
                    print("---")
                    print("new: " + image_pipeline_msgs[index].event.name)
                    print("expected: " + str(self.target_chain[chain_index]))
                    print("chain_index: " + str(chain_index))

                # first one            
                if (
                    chain_index == 0
                    and image_pipeline_msgs[index].event.name == self.target_chain[chain_index]
                ):
                    new_set.append(image_pipeline_msgs[index])
                    vpid_chain = image_pipeline_msgs[index].event.common_context_field.get(
                        "vpid"
                    )
                    chain_index += 1
                    if debug:
                        print(color("Found: " + str(image_pipeline_msgs[index].event.name) + " - " + str([x.event.name for x in new_set]), fg="blue"))
                # last one
                elif (
                    image_pipeline_msgs[index].event.name == self.target_chain[chain_index]
                    and self.target_chain[chain_index] == self.target_chain[-1]
                    and new_set[-1].event.name == self.target_chain[-2]
                    and image_pipeline_msgs[index].event.common_context_field.get("vpid")
                    == vpid_chain
                ):
                    new_set.append(image_pipeline_msgs[index])
                    image_pipeline_msg_sets.append(new_set)
                    if debug:
                        print(color("Found: " + str(image_pipeline_msgs[index].event.name) + " - " + str([x.event.name for x in new_set]), fg="blue"))
                    chain_index = 0  # restart
                    new_set = []  # restart
                # match
                elif (
                    image_pipeline_msgs[index].event.name == self.target_chain[chain_index]
                    and image_pipeline_msgs[index].event.common_context_field.get("vpid")
                    == vpid_chain
                ):
                    new_set.append(image_pipeline_msgs[index])
                    chain_index += 1
                    if debug:
                        print(color("Found: " + str(image_pipeline_msgs[index].event.name) + " - " + str([x.event.name for x in new_set]), fg="green"))
                # altered order
                elif (
                    image_pipeline_msgs[index].event.name in self.target_chain
                    and image_pipeline_msgs[index].event.common_context_field.get("vpid")
                    == vpid_chain
                ):
                    # pop ros2:callback_start in new_set, if followed by "ros2:callback_end"
                    # NOTE: consider case of disconnected series of:
                    #       "ros2:callback_start"
                    #       "ros2:callback_end"
                    if (image_pipeline_msgs[index].event.name == "ros2:callback_end"
                        and self.target_chain[chain_index - 1] == "ros2:callback_start"):
                        new_set.pop()
                        chain_index -= 1
                    # # it's been observed that "robotperf_benchmarks:robotperf_image_input_cb_init" triggers
                    # # before "ros2_image_pipeline:image_proc_rectify_cb_fini" which leads to trouble
                    # # Skip this as well as the next event
                    # elif (image_pipeline_msgs[index].event.name == "robotperf_benchmarks:robotperf_image_input_cb_init"
                    #     and self.target_chain[chain_index - 3] == "ros2_image_pipeline:image_proc_rectify_cb_fini"):
                    #     print(color("Skipping: " + str(image_pipeline_msgs[index].event.name), fg="yellow"))
                    # elif (image_pipeline_msgs[index].event.name == "robotperf_benchmarks:robotperf_image_input_cb_fini"
                    #     and self.target_chain[chain_index - 3] == "ros2_image_pipeline:image_proc_rectify_cb_fini"):
                    #     print(color("Skipping: " + str(image_pipeline_msgs[index].event.name), fg="yellow"))
                    else:
                        new_set.append(image_pipeline_msgs[index])
                        if debug:
                            print(color("Altered order: " + str([x.event.name for x in new_set]) + ", restarting", fg="red"))
                        chain_index = 0  # restart
                        new_set = []  # restart
        return image_pipeline_msg_sets

    def barplot_all(self, image_pipeline_msg_sets, title="Barplot"):

        image_pipeline_msg_sets_ns = []
        for set_index in range(len(image_pipeline_msg_sets)):
            aux_set = []
            target_chain_ns = []
            for msg_index in range(len(image_pipeline_msg_sets[set_index])):
                target_chain_ns.append(
                    image_pipeline_msg_sets[set_index][
                        msg_index
                    ].default_clock_snapshot.ns_from_origin
                )
            init_ns = target_chain_ns[0]
            for msg_index in range(len(image_pipeline_msg_sets[set_index])):
                aux_set.append((target_chain_ns[msg_index] - init_ns) / 1e6)
            image_pipeline_msg_sets_ns.append(aux_set)

        df = pd.DataFrame(image_pipeline_msg_sets_ns)
        df.columns = self.target_chain_dissambiguous
        import plotly.express as px

        # pd.set_option("display.max_rows", None, "display.max_columns", None)
        # print(df)

        fig = px.box(
            df,
            points="all",
            template="plotly_white",
            title=title,
        )
        fig.update_xaxes(title_text="Trace event")
        fig.update_yaxes(title_text="Milliseconds")
        # fig.show()
        fig.write_image("/tmp/analysis/plot_barplot.png", width=1400, height=1000)    

    def traces_id(self, msg_set):
        # this method only works for hardcoded traces, specifically for the a1 benchmark
        # TODO: make this function generic so other benchmarks can also be plotted 

        # For some reason it seems to be displayed in the reverse order on the Y axis
        if self.hardware_device_type == "cpu":
            segment_types = ["rmw", "rcl", "rclcpp", "userland", "benchmark"]
        elif self.hardware_device_type == "fpga":
            segment_types = ["kernel", "rmw", "rcl", "rclcpp", "userland", "benchmark"]

        fig = figure(
            title="RobotPerf benchmark:" + self.benchmark_name,
            x_axis_label=f"Milliseconds",
            y_range=segment_types,
            plot_width=2000,
            plot_height=600,
        )
        fig.title.align = "center"
        fig.title.text_font_size = "20px"
        # fig.xaxis[0].formatter = DatetimeTickFormatter(milliseconds = ['%3Nms'])
        fig.xaxis[0].formatter = PrintfTickFormatter(format="%f ms")
        fig.xaxis[0].ticker.desired_num_ticks = 20
        fig.xaxis[0].axis_label_text_font_size = "30px"
        fig.yaxis[0].major_label_text_font_size = "25px"

        target_chain_ns = []
        for msg_index in range(len(msg_set)):
            target_chain_ns.append(msg_set[msg_index].default_clock_snapshot.ns_from_origin)
        init_ns = target_chain_ns[0]

        # print("1")

        # draw durations
        ## robotperf_image_input_cb_fini-robotperf_image_output_cb_init duration
        callback_start = (target_chain_ns[1] - init_ns) / 1e6
        callback_end = (target_chain_ns[10] - init_ns) / 1e6
        duration = callback_end - callback_start
        self.add_durations_to_figure(
            fig,
            self.target_chain_layer[1],  # index used in here
                                    # should match with the
                                    # one from the callback_start
            [(callback_start, callback_start + duration, duration)],
            "khaki",
        )

        ## rclcpp callbacks - robotperf_image_input_cb_init
        callback_start = (target_chain_ns[0] - init_ns) / 1e6
        callback_end = (target_chain_ns[1] - init_ns) / 1e6
        duration = callback_end - callback_start
        self.add_durations_to_figure(
            fig,
            self.target_chain_layer[0],  # index used in here
                                    # should match with the
                                    # one from the callback_start
            [(callback_start, callback_start + duration, duration)],
            "lightgray",
        )

        ## rclcpp callbacks - rectify
        callback_start = (target_chain_ns[2] - init_ns) / 1e6
        callback_end = (target_chain_ns[5] - init_ns) / 1e6
        duration = callback_end - callback_start
        self.add_durations_to_figure(
            fig,
            self.target_chain_layer[2],  # index used in here
                                    # should match with the
                                    # one from the callback_start
            [(callback_start, callback_start + duration, duration)],
            "lightgray",
        )

        ## rclcpp callbacks - resize
        callback_start = (target_chain_ns[6] - init_ns) / 1e6
        callback_end = (target_chain_ns[9] - init_ns) / 1e6
        duration = callback_end - callback_start
        self.add_durations_to_figure(
            fig,
            self.target_chain_layer[6],  # index used in here
                                    # should match with the
                                    # one from the callback_start
            [(callback_start, callback_start + duration, duration)],
            "lightgray",
        )

        ## rclcpp callbacks - robotperf_image_output_cb_init
        callback_start = (target_chain_ns[10] - init_ns) / 1e6
        callback_end = (target_chain_ns[11] - init_ns) / 1e6
        duration = callback_end - callback_start
        self.add_durations_to_figure(
            fig,
            self.target_chain_layer[10],  # index used in here
                                    # should match with the
                                    # one from the callback_start
            [(callback_start, callback_start + duration, duration)],
            "lightgray",
        )

        ## rectify op
        callback_start = (target_chain_ns[3] - init_ns) / 1e6
        callback_end = (target_chain_ns[4] - init_ns) / 1e6
        duration = callback_end - callback_start
        self.add_durations_to_figure(
            fig,
            self.target_chain_layer[3],  # index used in here
                                    # should match with the
                                    # one from the callback_start
            [(callback_start, callback_start + duration, duration)],
            "seashell",
        )

        ## resize op
        callback_start = (target_chain_ns[7] - init_ns) / 1e6
        callback_end = (target_chain_ns[8] - init_ns) / 1e6
        duration = callback_end - callback_start
        self.add_durations_to_figure(
            fig,
            self.target_chain_layer[7],  # index used in here
                                    # should match with the
                                    # one from the callback_start
            [(callback_start, callback_start + duration, duration)],
            "seashell",
        )

        for msg_index in range(len(msg_set)):
            #     self.add_markers_to_figure(fig, msg_set[msg_index].event.name, [(target_chain_ns[msg_index] - init_ns)/1e6], 'blue', marker_type='plus', legend_label='timing')
            # print("marker ms: " + str((target_chain_ns[msg_index] - init_ns) / 1e6))
            self.add_markers_to_figure(
                fig,
                self.target_chain_layer[msg_index],
                [(target_chain_ns[msg_index] - init_ns) / 1e6],
                self.target_chain_colors_fg_bokeh[msg_index],
                marker_type=self.target_chain_marker[msg_index],
                # legend_label=msg_set[msg_index].event.name,
                legend_label=self.target_chain_dissambiguous[msg_index],
                size=10,
            )        
            if "robotperf_image_input_cb_fini" in msg_set[msg_index].event.name:
                label = Label(
                    x=(target_chain_ns[msg_index] - init_ns) / 1e6,
                    y=self.target_chain_label_layer[msg_index],
                    x_offset=-40,
                    y_offset=-40,
                    text=self.target_chain_dissambiguous[msg_index].split(":")[-1],
                )

            elif "robotperf_image_output_cb_init" in msg_set[msg_index].event.name:
                label = Label(
                    x=(target_chain_ns[msg_index] - init_ns) / 1e6,
                    y=self.target_chain_label_layer[msg_index],
                    x_offset=-200,
                    y_offset=-40,
                    text=self.target_chain_dissambiguous[msg_index].split(":")[-1],
                )
            else:
                label = Label(
                    x=(target_chain_ns[msg_index] - init_ns) / 1e6,
                    y=self.target_chain_label_layer[msg_index],
                    x_offset=-30,
                    y_offset=-30,
                    # text=self.target_chain_dissambiguous[msg_index].split(":")[-1],
                    text="",
                )
            fig.add_layout(label)

        # hack legend to the right
        fig.legend.location = "right"
        new_legend = fig.legend[0]
        fig.legend[0] = None
        fig.add_layout(new_legend, "right")
        
        ## output
        # show(fig)  # show in browser    
        export_png(fig, filename="/tmp/analysis/plot_trace.png")

    def traces(self, msg_set):
        # this method only works for hardcoded traces, specifically for the a1 benchmark
        # TODO: make this function generic so other benchmarks can also be plotted 

        # For some reason it seems to be displayed in the reverse order on the Y axis
        if self.hardware_device_type == "cpu":
            segment_types = ["rmw", "rcl", "rclcpp", "userland", "benchmark"]
        elif self.hardware_device_type == "fpga":
            segment_types = ["kernel", "rmw", "rcl", "rclcpp", "userland", "benchmark"]

        fig = figure(
            title="RobotPerf benchmark:" + self.benchmark_name,
            x_axis_label=f"Milliseconds",
            y_range=segment_types,
            plot_width=2000,
            plot_height=600,
        )
        fig.title.align = "center"
        fig.title.text_font_size = "20px"
        # fig.xaxis[0].formatter = DatetimeTickFormatter(milliseconds = ['%3Nms'])
        fig.xaxis[0].formatter = PrintfTickFormatter(format="%f ms")
        fig.xaxis[0].ticker.desired_num_ticks = 20
        fig.xaxis[0].axis_label_text_font_size = "30px"
        fig.yaxis[0].major_label_text_font_size = "25px"

        target_chain_ns = []
        for msg_index in range(len(msg_set)):
            target_chain_ns.append(msg_set[msg_index].default_clock_snapshot.ns_from_origin)
        init_ns = target_chain_ns[0]

        # print("1")

        # draw durations
        ## robotperf_image_input_cb_fini-robotperf_image_output_cb_init duration
        callback_start = (target_chain_ns[2] - init_ns) / 1e6
        callback_end = (target_chain_ns[17] - init_ns) / 1e6
        duration = callback_end - callback_start
        self.add_durations_to_figure(
            fig,
            self.target_chain_layer[2],  # index used in here
                                    # should match with the
                                    # one from the callback_start
            [(callback_start, callback_start + duration, duration)],
            "khaki",
        )

        ## rclcpp callbacks - robotperf_image_input_cb_init
        callback_start = (target_chain_ns[0] - init_ns) / 1e6
        callback_end = (target_chain_ns[3] - init_ns) / 1e6
        duration = callback_end - callback_start
        self.add_durations_to_figure(
            fig,
            self.target_chain_layer[0],  # index used in here
                                    # should match with the
                                    # one from the callback_start
            [(callback_start, callback_start + duration, duration)],
            "lightgray",
        )

        ## rclcpp callbacks - rectify
        callback_start = (target_chain_ns[4] - init_ns) / 1e6
        callback_end = (target_chain_ns[9] - init_ns) / 1e6
        duration = callback_end - callback_start
        self.add_durations_to_figure(
            fig,
            self.target_chain_layer[0],  # index used in here
                                    # should match with the
                                    # one from the callback_start
            [(callback_start, callback_start + duration, duration)],
            "lightgray",
        )

        ## rclcpp callbacks - resize
        callback_start = (target_chain_ns[10] - init_ns) / 1e6
        callback_end = (target_chain_ns[15] - init_ns) / 1e6
        duration = callback_end - callback_start
        self.add_durations_to_figure(
            fig,
            self.target_chain_layer[10],  # index used in here
                                    # should match with the
                                    # one from the callback_start
            [(callback_start, callback_start + duration, duration)],
            "lightgray",
        )

        ## rclcpp callbacks - robotperf_image_output_cb_init
        callback_start = (target_chain_ns[16] - init_ns) / 1e6
        callback_end = (target_chain_ns[19] - init_ns) / 1e6
        duration = callback_end - callback_start
        self.add_durations_to_figure(
            fig,
            self.target_chain_layer[16],  # index used in here
                                    # should match with the
                                    # one from the callback_start
            [(callback_start, callback_start + duration, duration)],
            "lightgray",
        )

        ## rectify callback
        callback_start = (target_chain_ns[5] - init_ns) / 1e6
        callback_end = (target_chain_ns[8] - init_ns) / 1e6
        duration = callback_end - callback_start
        self.add_durations_to_figure(
            fig,
            self.target_chain_layer[5],  # index used in here
                                    # should match with the
                                    # one from the callback_start
            [(callback_start, callback_start + duration, duration)],
            "whitesmoke",
        )

        ## rectify op
        callback_start = (target_chain_ns[6] - init_ns) / 1e6
        callback_end = (target_chain_ns[7] - init_ns) / 1e6
        duration = callback_end - callback_start
        self.add_durations_to_figure(
            fig,
            self.target_chain_layer[6],  # index used in here
                                    # should match with the
                                    # one from the callback_start
            [(callback_start, callback_start + duration, duration)],
            "seashell",
        )

        ## resize callback
        callback_start = (target_chain_ns[11] - init_ns) / 1e6
        callback_end = (target_chain_ns[14] - init_ns) / 1e6
        duration = callback_end - callback_start
        self.add_durations_to_figure(
            fig,
            self.target_chain_layer[11],  # index used in here
                                    # should match with the
                                    # one from the callback_start
            [(callback_start, callback_start + duration, duration)],
            "whitesmoke",
        )
        ## resize op
        callback_start = (target_chain_ns[12] - init_ns) / 1e6
        callback_end = (target_chain_ns[13] - init_ns) / 1e6
        duration = callback_end - callback_start
        self.add_durations_to_figure(
            fig,
            self.target_chain_layer[12],  # index used in here
                                    # should match with the
                                    # one from the callback_start
            [(callback_start, callback_start + duration, duration)],
            "seashell",
        )

        ## robotperf_image_input_cb_init callback
        callback_start = (target_chain_ns[1] - init_ns) / 1e6
        callback_end = (target_chain_ns[2] - init_ns) / 1e6
        duration = callback_end - callback_start
        self.add_durations_to_figure(
            fig,
            self.target_chain_layer[1],  # index used in here
                                    # should match with the
                                    # one from the callback_start
            [(callback_start, callback_start + duration, duration)],
            "whitesmoke",
        )

        ## robotperf_image_output_cb_init callback
        callback_start = (target_chain_ns[17] - init_ns) / 1e6
        callback_end = (target_chain_ns[18] - init_ns) / 1e6
        duration = callback_end - callback_start
        self.add_durations_to_figure(
            fig,
            self.target_chain_layer[17],  # index used in here
                                    # should match with the
                                    # one from the callback_start
            [(callback_start, callback_start + duration, duration)],
            "whitesmoke",
        )
        
        for msg_index in range(len(msg_set)):
            #     self.add_markers_to_figure(fig, msg_set[msg_index].event.name, [(target_chain_ns[msg_index] - init_ns)/1e6], 'blue', marker_type='plus', legend_label='timing')
            # print("marker ms: " + str((target_chain_ns[msg_index] - init_ns) / 1e6))
            self.add_markers_to_figure(
                fig,
                self.target_chain_layer[msg_index],
                [(target_chain_ns[msg_index] - init_ns) / 1e6],
                self.target_chain_colors_fg_bokeh[msg_index],
                marker_type=self.target_chain_marker[msg_index],
                # legend_label=msg_set[msg_index].event.name,
                legend_label=self.target_chain_dissambiguous[msg_index],
                size=10,
            )        
            if "robotperf_image_input_cb_fini" in msg_set[msg_index].event.name:
                label = Label(
                    x=(target_chain_ns[msg_index] - init_ns) / 1e6,
                    y=self.target_chain_label_layer[msg_index],
                    x_offset=-40,
                    y_offset=-40,
                    text=self.target_chain_dissambiguous[msg_index].split(":")[-1],
                )

            elif "robotperf_image_output_cb_init" in msg_set[msg_index].event.name:
                label = Label(
                    x=(target_chain_ns[msg_index] - init_ns) / 1e6,
                    y=self.target_chain_label_layer[msg_index],
                    x_offset=-200,
                    y_offset=-40,
                    text=self.target_chain_dissambiguous[msg_index].split(":")[-1],
                )
            else:
                label = Label(
                    x=(target_chain_ns[msg_index] - init_ns) / 1e6,
                    y=self.target_chain_label_layer[msg_index],
                    x_offset=-30,
                    y_offset=-30,
                    # text=self.target_chain_dissambiguous[msg_index].split(":")[-1],
                    text="",
                )
            fig.add_layout(label)

        # hack legend to the right
        fig.legend.location = "right"
        new_legend = fig.legend[0]
        fig.legend[0] = None
        fig.add_layout(new_legend, "right")
        
        ## output
        # show(fig)  # show in browser    
        export_png(fig, filename="/tmp/analysis/plot_trace.png")


    def traces_fpga(self, msg_set):        
        # this method only works for hardcoded traces, specifically for the a1 fpga benchmark
        # TODO: make this function generic so other benchmarks can also be plotted         

        # For some reason it seems to be displayed in the reverse order on the Y axis
        if self.hardware_device_type == "cpu":
            segment_types = ["rmw", "rcl", "rclcpp", "userland", "benchmark"]
        elif self.hardware_device_type == "fpga":
            segment_types = ["kernel", "rmw", "rcl", "rclcpp", "userland", "benchmark"]

        fig = figure(
            title="RobotPerf benchmark: a1_perception_2nodes",
            x_axis_label=f"Milliseconds",
            y_range=segment_types,
            plot_width=2000,
            plot_height=600,
        )
        fig.title.align = "center"
        fig.title.text_font_size = "20px"
        # fig.xaxis[0].formatter = DatetimeTickFormatter(milliseconds = ['%3Nms'])
        fig.xaxis[0].formatter = PrintfTickFormatter(format="%f ms")
        fig.xaxis[0].ticker.desired_num_ticks = 20
        fig.xaxis[0].axis_label_text_font_size = "30px"
        fig.yaxis[0].major_label_text_font_size = "25px"

        target_chain_ns = []
        for msg_index in range(len(msg_set)):
            target_chain_ns.append(msg_set[msg_index].default_clock_snapshot.ns_from_origin)
        init_ns = target_chain_ns[0]

        # draw durations
        ## robotperf_image_input_cb_fini-robotperf_image_output_cb_init duration
        callback_start = (target_chain_ns[2] - init_ns) / 1e6
        callback_end = (target_chain_ns[21] - init_ns) / 1e6
        duration = callback_end - callback_start
        self.add_durations_to_figure(
            fig,
            self.target_chain_layer[2],  # index used in here
                                    # should match with the
                                    # one from the callback_start
            [(callback_start, callback_start + duration, duration)],
            "khaki",
        )

        ## rclcpp callbacks - robotperf_image_input_cb_init
        callback_start = (target_chain_ns[0] - init_ns) / 1e6
        callback_end = (target_chain_ns[3] - init_ns) / 1e6
        duration = callback_end - callback_start
        self.add_durations_to_figure(
            fig,
            self.target_chain_layer[0],  # index used in here
                                    # should match with the
                                    # one from the callback_start
            [(callback_start, callback_start + duration, duration)],
            "lightgray",
        )

        ## rclcpp callbacks - rectify
        callback_start = (target_chain_ns[4] - init_ns) / 1e6
        callback_end = (target_chain_ns[11] - init_ns) / 1e6
        duration = callback_end - callback_start
        self.add_durations_to_figure(
            fig,
            self.target_chain_layer[4],  # index used in here
                                    # should match with the
                                    # one from the callback_start
            [(callback_start, callback_start + duration, duration)],
            "lightgray",
        )

        ## rclcpp callbacks - resize
        callback_start = (target_chain_ns[12] - init_ns) / 1e6
        callback_end = (target_chain_ns[19] - init_ns) / 1e6
        duration = callback_end - callback_start
        self.add_durations_to_figure(
            fig,
            self.target_chain_layer[12],  # index used in here
                                    # should match with the
                                    # one from the callback_start
            [(callback_start, callback_start + duration, duration)],
            "lightgray",
        )

        ## rclcpp callbacks - robotperf_image_output_cb_init
        callback_start = (target_chain_ns[20] - init_ns) / 1e6
        callback_end = (target_chain_ns[23] - init_ns) / 1e6
        duration = callback_end - callback_start
        self.add_durations_to_figure(
            fig,
            self.target_chain_layer[20], # index used in here
                                    # should match with the
                                    # one from the callback_start
            [(callback_start, callback_start + duration, duration)],
            "lightgray",
        )

        ## rectify callback
        callback_start = (target_chain_ns[5] - init_ns) / 1e6
        callback_end = (target_chain_ns[10] - init_ns) / 1e6
        duration = callback_end - callback_start
        self.add_durations_to_figure(
            fig,
            self.target_chain_layer[5],  # index used in here
                                    # should match with the
                                    # one from the callback_start
            [(callback_start, callback_start + duration, duration)],
            "whitesmoke",
        )

        ## rectify op
        callback_start = (target_chain_ns[6] - init_ns) / 1e6
        callback_end = (target_chain_ns[9] - init_ns) / 1e6
        duration = callback_end - callback_start
        self.add_durations_to_figure(
            fig,
            self.target_chain_layer[6],  # index used in here
                                    # should match with the
                                    # one from the callback_start
            [(callback_start, callback_start + duration, duration)],
            "seashell",
        )

        ## resize callback
        callback_start = (target_chain_ns[13] - init_ns) / 1e6
        callback_end = (target_chain_ns[18] - init_ns) / 1e6
        duration = callback_end - callback_start
        self.add_durations_to_figure(
            fig,
            self.target_chain_layer[13], # index used in here
                                    # should match with the
                                    # one from the callback_start
            [(callback_start, callback_start + duration, duration)],
            "whitesmoke",
        )
        ## resize op
        callback_start = (target_chain_ns[14] - init_ns) / 1e6
        callback_end = (target_chain_ns[17] - init_ns) / 1e6
        duration = callback_end - callback_start
        self.add_durations_to_figure(
            fig,
            self.target_chain_layer[14], # index used in here
                                    # should match with the
                                    # one from the callback_start
            [(callback_start, callback_start + duration, duration)],
            "seashell",
        )

        ## robotperf_image_input_cb_init callback
        callback_start = (target_chain_ns[1] - init_ns) / 1e6
        callback_end = (target_chain_ns[2] - init_ns) / 1e6
        duration = callback_end - callback_start
        self.add_durations_to_figure(
            fig,
            self.target_chain_layer[1],  # index used in here
                                    # should match with the
                                    # one from the callback_start
            [(callback_start, callback_start + duration, duration)],
            "whitesmoke",
        )

        ## robotperf_image_output_cb_init callback
        callback_start = (target_chain_ns[21] - init_ns) / 1e6
        callback_end = (target_chain_ns[22] - init_ns) / 1e6
        duration = callback_end - callback_start
        self.add_durations_to_figure(
            fig,
            self.target_chain_layer[21], # index used in here
                                    # should match with the
                                    # one from the callback_start
            [(callback_start, callback_start + duration, duration)],
            "whitesmoke",
        )

        ## kernel_enqueue (rectify)
        callback_start = (target_chain_ns[7] - init_ns) / 1e6
        callback_end = (target_chain_ns[8] - init_ns) / 1e6
        duration = callback_end - callback_start
        self.add_durations_to_figure(
            fig,
            self.target_chain_layer[7], # index used in here
                                    # should match with the
                                    # one from the callback_start
            [(callback_start, callback_start + duration, duration)],
            "palegreen",
        )

        ## kernel_enqueue (resize)
        callback_start = (target_chain_ns[15] - init_ns) / 1e6
        callback_end = (target_chain_ns[16] - init_ns) / 1e6
        duration = callback_end - callback_start
        self.add_durations_to_figure(
            fig,
            self.target_chain_layer[15], # index used in here
                                    # should match with the
                                    # one from the callback_start
            [(callback_start, callback_start + duration, duration)],
            "palegreen",
        )


        for msg_index in range(len(msg_set)):
            #     add_markers_to_figure(fig, msg_set[msg_index].event.name, [(target_chain_ns[msg_index] - init_ns)/1e6], 'blue', marker_type='plus', legend_label='timing')
            # print("marker ms: " + str((target_chain_ns[msg_index] - init_ns) / 1e6))
            self.add_markers_to_figure(
                fig,
                self.target_chain_layer[msg_index],
                [(target_chain_ns[msg_index] - init_ns) / 1e6],
                self.target_chain_colors_fg_bokeh[msg_index],
                marker_type=self.target_chain_marker[msg_index],
                # legend_label=msg_set[msg_index].event.name,
                legend_label=self.target_chain_dissambiguous[msg_index],
                size=10,
            )        
            if "robotperf_image_input_cb_fini" in msg_set[msg_index].event.name:
                label = Label(
                    x=(target_chain_ns[msg_index] - init_ns) / 1e6,
                    y=self.target_chain_label_layer[msg_index],
                    x_offset=-40,
                    y_offset=-40,
                    text=self.target_chain_dissambiguous[msg_index].split(":")[-1],
                )

            elif "robotperf_image_output_cb_init" in msg_set[msg_index].event.name:
                label = Label(
                    x=(target_chain_ns[msg_index] - init_ns) / 1e6,
                    y=self.target_chain_label_layer[msg_index],
                    x_offset=-200,
                    y_offset=-40,
                    text=self.target_chain_dissambiguous[msg_index].split(":")[-1],
                )
            else:
                label = Label(
                    x=(target_chain_ns[msg_index] - init_ns) / 1e6,
                    y=self.target_chain_label_layer[msg_index],
                    x_offset=-30,
                    y_offset=-30,
                    # text=target_chain_dissambiguous[msg_index].split(":")[-1],
                    text="",
                )
            fig.add_layout(label)

        # hack legend to the right
        fig.legend.location = "right"
        new_legend = fig.legend[0]
        fig.legend[0] = None
        fig.add_layout(new_legend, "right")
        
        ## output
        # show(fig)  # show in browser    
        export_png(fig, filename="/tmp/analysis/plot_trace.png")

    def barchart_data_throughput(self, image_pipeline_msg_sets, option):
        """
        Converts a tracing message list into its corresponding 
        throughput list in bytes per second unit.
        - latency is measured relative (to the previous tracepoint) in
        millisecond units.
        - size is measured in bytes
        - count is measured in number messages

        Args:
            image_pipeline_msg_sets ([type]): [description]

        Returns:
            list: list of throughput in bytes/s
        """
        image_pipeline_msg_sets_ns = []
        image_pipeline_msg_sets_bytes = []
        image_pipeline_msg_sets_frames = []
        image_pipeline_msg_sets_msgs = []
        # if multidimensional:
        if type(image_pipeline_msg_sets[0]) == list:
            for set_index in range(len(image_pipeline_msg_sets)):
                aux_set = []
                target_chain_ns = []
                target_chain_bytes = []
                target_chain_msgs = []
                target_chain_frames = []
                for msg_index in range(len(image_pipeline_msg_sets[set_index])):
                    target_chain_ns.append(
                        image_pipeline_msg_sets[set_index][
                            msg_index
                        ].default_clock_snapshot.ns_from_origin
                    )
                    # search for message sizes
                    msg_size = 0
                    msg_count = 0
                    payload_fields = image_pipeline_msg_sets[set_index][msg_index].event.payload_field
                    for field_name, field_value in payload_fields.items():
                        if "msg_size" in field_name:
                            msg_size += image_pipeline_msg_sets[set_index][msg_index].event.payload_field[field_name]
                            msg_count += 1
                    target_chain_bytes.append(msg_size)
                    target_chain_msgs.append(msg_count)
                    if msg_count > 0:
                        target_chain_frames.append(1)
                    else:
                        target_chain_frames.append(0)
                for msg_index in range(len(image_pipeline_msg_sets[set_index])):
                    if msg_index == 0:
                        previous = target_chain_ns[0]
                    else:
                        previous = target_chain_ns[msg_index - 1]
                    aux_set.append((target_chain_ns[msg_index] - previous) / 1e6)
                image_pipeline_msg_sets_ns.append(aux_set)
                image_pipeline_msg_sets_bytes.append(target_chain_bytes)
                image_pipeline_msg_sets_msgs.append(target_chain_msgs)
                image_pipeline_msg_sets_frames.append(target_chain_frames)

        else:  # not multidimensional
            aux_set = []
            target_chain_ns = []
            target_chain_bytes = []
            target_chain_msgs = []
            target_chain_frames = []
            for msg_index in range(len(image_pipeline_msg_sets)):
                target_chain_ns.append(
                    image_pipeline_msg_sets[msg_index].default_clock_snapshot.ns_from_origin
                )
                # search for message sizes
                msg_size = 0
                msg_count = 0
                payload_fields = image_pipeline_msg_sets[msg_index].event.payload_field
                for field_name, field_value in payload_fields.items():
                    if "msg_size" in field_name:
                        msg_size += image_pipeline_msg_sets[msg_index].event.payload_field[field_name]
                        msg_count += 1
                target_chain_bytes.append(msg_size)
                target_chain_msgs.append(msg_count)
                if msg_count > 0:
                    target_chain_frames.append(1)
                else:
                    target_chain_frames.append(0)
            for msg_index in range(len(image_pipeline_msg_sets)):
                if msg_index == 0:
                    previous = target_chain_ns[0]
                else:
                    previous = target_chain_ns[msg_index - 1]
                aux_set.append((target_chain_ns[msg_index] - previous) / 1e6)
            image_pipeline_msg_sets_ns.append(aux_set)
            image_pipeline_msg_sets_bytes.append(target_chain_bytes)
            image_pipeline_msg_sets_msgs.append(target_chain_msgs)
            image_pipeline_msg_sets_frames.append(target_chain_frames)
        
        # Compute throughput from the output [-1]
        image_pipeline_msg_sets_megabyps = []
        image_pipeline_msg_sets_msgspers = []
        image_pipeline_msg_sets_fps = []

        if option == 'potential':
            for i in range(len(image_pipeline_msg_sets_ns)):
                tot_lat = 0
                for j in range(1,len(image_pipeline_msg_sets_ns[i])-2):
                    tot_lat += image_pipeline_msg_sets_ns[i][j]
                image_pipeline_msg_sets_megabyps.append(image_pipeline_msg_sets_bytes[i][-2]/tot_lat/1e6*1e3)
                image_pipeline_msg_sets_msgspers.append(image_pipeline_msg_sets_msgs[i][-2]/tot_lat*1e3)
                image_pipeline_msg_sets_fps.append(image_pipeline_msg_sets_frames[i][-2]/tot_lat*1e3)

        elif option == 'real':
            for i in range(len(image_pipeline_msg_sets_ns)):
                tot_lat = 0
                for j in range(len(image_pipeline_msg_sets_ns[i])):
                    tot_lat += image_pipeline_msg_sets_ns[i][j]
                image_pipeline_msg_sets_megabyps.append(image_pipeline_msg_sets_bytes[i][-2]/tot_lat/1e6*1e3)
                image_pipeline_msg_sets_msgspers.append(image_pipeline_msg_sets_msgs[i][-2]/tot_lat*1e3)
                image_pipeline_msg_sets_fps.append(image_pipeline_msg_sets_frames[i][-2]/tot_lat*1e3)

        return image_pipeline_msg_sets_megabyps, image_pipeline_msg_sets_fps


    def barchart_data_latency(self, image_pipeline_msg_sets):
        """
        Converts a tracing message list into its corresponding
        relative (to the previous tracepoint) latency list in
        millisecond units.

        Args:
            image_pipeline_msg_sets ([type]): [description]

        Returns:
            list: list of relative latencies, in ms
        """
        image_pipeline_msg_sets_ns = []
        # if multidimensional:
        if type(image_pipeline_msg_sets[0]) == list:
            for set_index in range(len(image_pipeline_msg_sets)):
                aux_set = []
                target_chain_ns = []
                for msg_index in range(len(image_pipeline_msg_sets[set_index])):
                    target_chain_ns.append(
                        image_pipeline_msg_sets[set_index][
                            msg_index
                        ].default_clock_snapshot.ns_from_origin
                    )
                for msg_index in range(len(image_pipeline_msg_sets[set_index])):
                    if msg_index == 0:
                        previous = target_chain_ns[0]
                    else:
                        previous = target_chain_ns[msg_index - 1]
                    aux_set.append((target_chain_ns[msg_index] - previous) / 1e6)
                image_pipeline_msg_sets_ns.append(aux_set)
        else:  # not multidimensional
            aux_set = []
            target_chain_ns = []
            for msg_index in range(len(image_pipeline_msg_sets)):
                target_chain_ns.append(
                    image_pipeline_msg_sets[msg_index].default_clock_snapshot.ns_from_origin
                )
            for msg_index in range(len(image_pipeline_msg_sets)):
                if msg_index == 0:
                    previous = target_chain_ns[0]
                else:
                    previous = target_chain_ns[msg_index - 1]
                aux_set.append((target_chain_ns[msg_index] - previous) / 1e6)
            image_pipeline_msg_sets_ns.append(aux_set)

        return image_pipeline_msg_sets_ns

    def print_timeline(self, image_pipeline_msg_sets):

        for msg_set in image_pipeline_msg_sets:
            if len(msg_set) != len(self.target_chain):
                print(
                    color(
                        "Not a complete set: " + str([x.event.name for x in msg_set]),
                        fg="red",
                    )
                )
                pass
            else:
                target_chain_ns = []
                for msg_index in range(len(msg_set)):
                    target_chain_ns.append(
                        msg_set[msg_index].default_clock_snapshot.ns_from_origin
                    )

                init_ns = target_chain_ns[0]
                fixed_target_chain_ns = [init_ns] + target_chain_ns
                # stringout = color("raw image → " + msg_set[0].event.name + " → ")
                stringout = color("raw image ")
                for msg_index in range(len(msg_set)):
                    stringout += " → " + color(
                        msg_set[msg_index].event.name
                        + " ({} ms) ".format(
                            (
                                fixed_target_chain_ns[msg_index + 1]
                                - fixed_target_chain_ns[msg_index]
                            )
                            / 1e6
                        ),
                        fg=self.target_chain_colors_fg[msg_index],
                        bg="black",
                    )
                    # stringout += " → " + msg_set[msg_index].event.name + \
                    #     " ({} ms) ".format((fixed_target_chain_ns[msg_index + 1] - fixed_target_chain_ns[msg_index])/1e6)

                stringout += color(
                    "total "
                    + " ({} ms) ".format((target_chain_ns[-1] - target_chain_ns[0]) / 1e6),
                    fg="black",
                    bg="white",
                )
                print(stringout)


    def rms(self, list):
        return np.sqrt(np.mean(np.array(list) ** 2))


    def mean(self, list):
        return np.mean(np.array(list))


    def max(self, list):
        return np.max(np.array(list))


    def min(self, list):
        return np.min(np.array(list))


    def rms_sets(self, image_pipeline_msg_sets, indices=None):
        """
        Root-Mean-Square (RMS) (in the units provided) for a
        given number of time trace sets.

        NOTE: last value of the lists should not include the total

        :param: image_pipeline_msg_sets, list of lists, each containing the time traces
        :param: indices, list of indices to consider on each set which will be summed
        for rms. By default, sum of all values on each set.
        """

        if indices:
            with_indices_sets = []
            for set in image_pipeline_msg_sets:
                indices_sum = 0
                for i in indices:
                    indices_sum += set[i]
                with_indices_sets.append(indices_sum)
            return self.rms(with_indices_sets)
        else:
            total_in_sets = [sum(set) for set in image_pipeline_msg_sets]
            return self.rms(total_in_sets)


    def mean_sets(self, image_pipeline_msg_sets, indices=None):
        """

        """
        if indices:
            with_indices_sets = []
            for set in image_pipeline_msg_sets:
                indices_sum = 0
                for i in indices:
                    indices_sum += set[i]
                with_indices_sets.append(indices_sum)
            return self.mean(with_indices_sets)
        else:
            total_in_sets = [sum(set) for set in image_pipeline_msg_sets]
            return self.mean(total_in_sets)


    def max_sets(self, image_pipeline_msg_sets, indices=None):
        if indices:
            with_indices_sets = []
            for set in image_pipeline_msg_sets:
                indices_sum = 0
                for i in indices:
                    indices_sum += set[i]
                with_indices_sets.append(indices_sum)
            return self.max(with_indices_sets)
        else:
            total_in_sets = [sum(set) for set in image_pipeline_msg_sets]
            return self.max(total_in_sets)


    def min_sets(self, image_pipeline_msg_sets, indices=None):
        if indices:
            with_indices_sets = []
            for set in image_pipeline_msg_sets:
                indices_sum = 0
                for i in indices:
                    indices_sum += set[i]
                with_indices_sets.append(indices_sum)
            return self.min(with_indices_sets)
        else:
            total_in_sets = [sum(set) for set in image_pipeline_msg_sets]
            return self.min(total_in_sets)


    def print_timeline_average(self, image_pipeline_msg_sets):
        """
        Doing averages may lead to negative numbers while substracting the previous average.
        This is only useful to get an intuition of the totals.
        """

        image_pipeline_msg_sets_ns = []
        for msg_set in image_pipeline_msg_sets:
            if len(msg_set) != len(self.target_chain):
                print(
                    color(
                        "Not a complete set: " + str([x.event.name for x in msg_set]),
                        fg="red",
                    )
                )
                pass
            else:
                target_chain_ns = []
                final_target_chain_ns = []
                for msg_index in range(len(msg_set)):
                    target_chain_ns.append(
                        msg_set[msg_index].default_clock_snapshot.ns_from_origin
                    )
                init_ns = target_chain_ns[0]
                fixed_target_chain_ns = [init_ns] + target_chain_ns

                for msg_index in range(len(msg_set)):
                    final_target_chain_ns.append(
                        (
                            fixed_target_chain_ns[msg_index + 1]
                            - fixed_target_chain_ns[msg_index]
                        )
                    )
                final_target_chain_ns.append(
                    (fixed_target_chain_ns[-1] - fixed_target_chain_ns[0])
                )  # total
                image_pipeline_msg_sets_ns.append(final_target_chain_ns)

        image_pipeline_msg_ns_average = [
            sum(x) / len(x) for x in zip(*image_pipeline_msg_sets_ns)
        ]
        # print(image_pipeline_msg_ns_average)
        stringout = color("raw image ")
        for msg_index in range(len(image_pipeline_msg_ns_average[:-1])):
            stringout += " → " + color(
                image_pipeline_msg_sets[0][msg_index].event.name
                + " ({} ms) ".format(
                    (
                        image_pipeline_msg_ns_average[msg_index + 1]
                        - image_pipeline_msg_ns_average[msg_index]
                    )
                    / 1e6
                ),
                fg=self.target_chain_colors_fg[msg_index],
                bg="black",
            )

        stringout += color(
            "total "
            + " ({} ms) ".format(
                (image_pipeline_msg_ns_average[-1] - image_pipeline_msg_ns_average[0]) / 1e6
            ),
            fg="black",
            bg="white",
        )
        print(stringout)


    def statistics(self, image_pipeline_msg_sets_ms, verbose=False):

        mean_ = self.mean_sets(image_pipeline_msg_sets_ms)
        rms_ = self.rms_sets(image_pipeline_msg_sets_ms)
        min_ = self.min_sets(image_pipeline_msg_sets_ms)
        max_ = self.max_sets(image_pipeline_msg_sets_ms)

        # first_target = "ros2:callback_end"
        first_target = "robotperf_benchmarks:robotperf_image_input_cb_init"
        last_target = "robotperf_benchmarks:robotperf_image_output_cb_init"
        
        # # NOTE: we can particularizations for first_target and last_target
        # #       as needed, e.g.
        # if self.benchmark_name == "a3_stereo_image_proc":
        #     first_target = "robotperf_benchmarks:robotperf_image_input_cb_init"
        #
        # NOTE 2: find a way to parametrize this into the class

        indices = [i for i in range(
                    self.target_chain_dissambiguous.index(first_target),
                    1 + self.target_chain_dissambiguous.index(last_target),
                    )
                ]

        mean_benchmark = self.mean_sets(image_pipeline_msg_sets_ms,indices)
        rms_benchmark = self.rms_sets(image_pipeline_msg_sets_ms, indices)
        max_benchmark = self.max_sets(image_pipeline_msg_sets_ms, indices)
        min_benchmark = self.min_sets(image_pipeline_msg_sets_ms, indices)

        if verbose:
            print(color("mean: " + str(mean_), fg="yellow"))
            print("rms: " + str(rms_))
            print("min: " + str(min_))
            print(color("max: " + str(max_), fg="red"))

            print(color("mean benchmark: " + str(mean_benchmark), fg="yellow"))
            print("rms benchmark: " + str(rms_benchmark))
            print("min benchmark: " + str(min_benchmark))
            print(color("max benchmark: " + str(max_benchmark), fg="red"))

        return [
            mean_benchmark,
            rms_benchmark,
            max_benchmark,
            min_benchmark,
            mean_,
            rms_,
            max_,
            min_,
        ]
    
    def statistics_1d(self, image_pipeline_msg_sets_ms, verbose=False):

        mean_benchmark = self.mean(image_pipeline_msg_sets_ms)
        rms_benchmark = self.rms(image_pipeline_msg_sets_ms)
        max_benchmark = self.max(image_pipeline_msg_sets_ms)
        min_benchmark = self.min(image_pipeline_msg_sets_ms)

        if verbose:
            print(color("mean benchmark: " + str(mean_benchmark), fg="yellow"))
            print("rms benchmark: " + str(rms_benchmark))
            print("min benchmark: " + str(min_benchmark))
            print(color("max benchmark: " + str(max_benchmark), fg="red"))

        return [
            mean_benchmark,
            rms_benchmark,
            max_benchmark,
            min_benchmark,
        ]


    def print_markdown_table(self, list_sets, list_sets_names, from_baseline=True, units='ms'):
        """
        Creates a markdown table from a list of sets

        :param: list_sets: list of processed data (resulting from barchart_data_latency) to display
        :param: list_sets_names: list of names to display
        :param: from_baseline: whether to show % from baseline

        NOTE: assumes base is always the first set in list_sets, which
        is then used to calculate % of change.
        """

        list_statistics = []
        # generate statistics
        for sets in list_sets:
            list_statistics.append(self.statistics(sets))

        # Add name to each statistics list
        for stat_list_index in range(len(list_statistics)):
            list_statistics[stat_list_index].insert(0, list_sets_names[stat_list_index])

        # add headers
        list_statistics.insert(
            0,
            [
                "---",
                "---",
                "---",
                "---",
                "---",
                "---",
                "---",
                "---",
                "---",
            ],
        )
        list_statistics.insert(
            0,
            [
                " ",
                "Benchmark Mean",
                "Benchmark RMS",
                "Benchmark Max ",
                "Benchmark Min",
                "Mean",
                "RMS",
                "Max",
                "Min",
            ],
        )

        baseline = list_statistics[2]  # baseline for %

        # add missing messages at the end
        # NOTE: programmed for only 1 initial list_statistics, if more provided
        # consider extending the self.lost_msgs to a list
        if len(list_statistics) == 3:  # 1 initial, +2 headers
            list_statistics[0].append("Lost Messages")
            list_statistics[1].append("---")
            list_statistics[2].append("{:.2f} %".format((self.lost_msgs/len(self.image_pipeline_msg_sets))*100))

        length_list = [len(row) for row in list_statistics]
        column_width = max(length_list)
        count = 0
        for row in list_statistics:
            row_str = " | "
            if count == 2:
                for element_index in range(len(row)):
                    if type(row[element_index]) != str:
                        if from_baseline:
                            if row[element_index] > baseline[element_index]:
                                row_str += (
                                    "**{:.2f} {}**".format(row[element_index], units)
                                    + " (:small_red_triangle_down: `"
                                    + "{:.2f}".format(
                                        self.get_change(row[element_index], baseline[element_index])
                                    )
                                    + "`%) | "
                                )
                            else:
                                row_str += (
                                    "**{:.2f} {}**".format(row[element_index], units)
                                    + "{}**".format(units) 
                                    + " (`"
                                    + "{:.2f}".format(
                                        self.get_change(row[element_index], baseline[element_index])
                                    )
                                    + "`%) | "
                                )
                        else:
                            row_str += ("**{:.2f} {}**".format(row[element_index], units) + " | ")
                    else:
                        row_str += row[element_index] + " | "

            else:
                for element_index in range(len(row)):
                    if type(row[element_index]) != str:
                        if from_baseline:
                            if row[element_index] > baseline[element_index]:
                                row_str += (
                                    "{:.2f} {}".format(row[element_index], units) 
                                    + " (:small_red_triangle_down: `"
                                    + "{:.2f}".format(
                                        self.get_change(row[element_index], baseline[element_index])
                                    )
                                    + "`%) | "
                                )
                            else:
                                row_str += (
                                    "{:.2f} {}".format(row[element_index], units) 
                                    + " (`"
                                    + "{:.2f}".format(
                                        self.get_change(row[element_index], baseline[element_index])
                                    )
                                    + "`%) | "
                                )
                        else:
                            row_str += ("{:.2f} {}".format(row[element_index], units) + " | ")

                    else:
                        row_str += row[element_index] + " | "
            count += 1
            print(row_str)

            # if count == 2:
            #     row = "|" + "|".join("**{:.2f}**".format(row[element_index]) + " (`"
            #             + "{:.2f}".format(self.get_change(row[element_index], baseline[element_index])) + "`%)"
            #         if type(row[element_index]) != str
            #         else row[element_index]
            #             for element_index in range(len(row))) + "|"
            # else:
            #     row = "|" + "|".join("{:.2f}".format(row[element_index]) + " (`"
            #             + "{:.2f}".format(self.get_change(row[element_index], baseline[element_index])) + "`%)"
            #         if type(row[element_index]) != str else row[element_index]
            #             for element_index in range(len(row))) + "|"
            # count += 1
            # print(row)

    def print_markdown_table_1d(self, list_sets, list_sets_names, from_baseline=True, units='ms'):
        """
        Creates a markdown table from a list of sets

        :param: list_sets: list of processed data (resulting from barchart_data_latency) to display
        :param: list_sets_names: list of names to display
        :param: from_baseline: whether to show % from baseline

        NOTE: assumes base is always the first set in list_sets, which
        is then used to calculate % of change.
        """

        list_statistics = []
        # generate statistics
        for sets in list_sets:
            list_statistics.append(self.statistics_1d(sets))

        # Add name to each statistics list
        for stat_list_index in range(len(list_statistics)):
            list_statistics[stat_list_index].insert(0, list_sets_names[stat_list_index])

        # add headers
        list_statistics.insert(
            0,
            [
                "---",
                "---",
                "---",
                "---",
                "---",
                "---",
                "---",
                "---",
                "---",
            ],
        )
        list_statistics.insert(
            0,
            [
                " ",
                "Benchmark Mean",
                "Benchmark RMS",
                "Benchmark Max ",
                "Benchmark Min",
            ],
        )

        baseline = list_statistics[2]  # baseline for %

        # add missing messages at the end
        # NOTE: programmed for only 1 initial list_statistics, if more provided
        # consider extending the self.lost_msgs to a list
        if len(list_statistics) == 3:  # 1 initial, +2 headers
            list_statistics[0].append("Lost Messages")
            list_statistics[1].append("---")
            list_statistics[2].append("{:.2f} %".format((self.lost_msgs/len(self.image_pipeline_msg_sets))*100))

        length_list = [len(row) for row in list_statistics]
        column_width = max(length_list)
        count = 0
        for row in list_statistics:
            row_str = " | "
            if count == 2:
                for element_index in range(len(row)):
                    if type(row[element_index]) != str:
                        if from_baseline:
                            if row[element_index] > baseline[element_index]:
                                row_str += (
                                    "**{:.2f} {}**".format(row[element_index], units)
                                    + " (:small_red_triangle_down: `"
                                    + "{:.2f}".format(
                                        self.get_change(row[element_index], baseline[element_index])
                                    )
                                    + "`%) | "
                                )
                            else:
                                row_str += (
                                    "**{:.2f} {}**".format(row[element_index], units)
                                    + "{}**".format(units) 
                                    + " (`"
                                    + "{:.2f}".format(
                                        self.get_change(row[element_index], baseline[element_index])
                                    )
                                    + "`%) | "
                                )
                        else:
                            row_str += ("**{:.2f} {}**".format(row[element_index], units) + " | ")
                    else:
                        row_str += row[element_index] + " | "

            else:
                for element_index in range(len(row)):
                    if type(row[element_index]) != str:
                        if from_baseline:
                            if row[element_index] > baseline[element_index]:
                                row_str += (
                                    "{:.2f} {}".format(row[element_index], units) 
                                    + " (:small_red_triangle_down: `"
                                    + "{:.2f}".format(
                                        self.get_change(row[element_index], baseline[element_index])
                                    )
                                    + "`%) | "
                                )
                            else:
                                row_str += (
                                    "{:.2f} {}".format(row[element_index], units) 
                                    + " (`"
                                    + "{:.2f}".format(
                                        self.get_change(row[element_index], baseline[element_index])
                                    )
                                    + "`%) | "
                                )
                        else:
                            row_str += ("{:.2f} {}".format(row[element_index], units) + " | ")

                    else:
                        row_str += row[element_index] + " | "
            count += 1
            print(row_str)

            # if count == 2:
            #     row = "|" + "|".join("**{:.2f}**".format(row[element_index]) + " (`"
            #             + "{:.2f}".format(self.get_change(row[element_index], baseline[element_index])) + "`%)"
            #         if type(row[element_index]) != str
            #         else row[element_index]
            #             for element_index in range(len(row))) + "|"
            # else:
            #     row = "|" + "|".join("{:.2f}".format(row[element_index]) + " (`"
            #             + "{:.2f}".format(self.get_change(row[element_index], baseline[element_index])) + "`%)"
            #         if type(row[element_index]) != str else row[element_index]
            #             for element_index in range(len(row))) + "|"
            # count += 1
            # print(row)



    def results(self, sets):
        """
        Builds a dictionary of results from a list of sets.

        :param: sets: list of processed data

        NOTE: Syntax should follow the following format:
            {
                "hardware": "kr260",
                "category": "perception",
                "timestampt": time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(time.time())),
                "value": 15.2,
                "note": "Note",
                "datasource": "perception/image"
            }    
        """

        # mean_benchmark, rms_benchmark, max_benchmark, min_benchmark, mean_, rms_, max_, min_
        # 0,                1,                  2,          3,          4,      5,   6,    7
        statistics_data = self.statistics(sets)

        # print(statistics_data[2])
        return {
                "hardware": os.environ.get('HARDWARE'),
                "category": os.environ.get('CATEGORY'),
                "timestampt": time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(time.time())),
                "value": float(statistics_data[2]),
                "note": "mean_benchmark {}, rms_benchmark {}, max_benchmark {}, min_benchmark {}, lost messages {:.2f} %".format(statistics_data[0], statistics_data[1], statistics_data[2], statistics_data[3], (self.lost_msgs/len(self.image_pipeline_msg_sets))*100),
                "datasource": os.environ.get('ROSBAG')
            }


    def run(self, cmd, shell=False, timeout=1):
        """
        Spawns a new processe launching cmd, connect to their input/output/error pipes, and obtain their return codes.
        :param cmd: command split in the form of a list
        :returns: stdout
        """
        proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, shell=shell)
        try:
            outs, errs = proc.communicate(timeout=timeout)
        except subprocess.TimeoutExpired:
            proc.kill()
            outs, errs = proc.communicate()

        # decode, or None
        if outs:
            outs = outs.decode("utf-8").strip()
        else:
            outs = None

        if errs:
            errs = errs.decode("utf-8").strip()
        else:
            errs = None
        return outs, errs

    def get_target_chain_traces(self, trace_path):
        if trace_path:
            # self.image_pipeline_msg_sets \
            #     = self.msgsets_from_trace(trace_path, True)
            self.image_pipeline_msg_sets \
                = self.msgsets_from_trace_identifier(trace_path, debug=True)
        else:
            if self.hardware_device_type == "cpu":
                # self.image_pipeline_msg_sets = self.msgsets_from_trace(
                #     # os.getenv("HOME") + "/.ros/tracing/" + self.benchmark_name,
                #     "/tmp/analysis/trace/trace_cpu_ctf",
                #     True)
                self.image_pipeline_msg_sets \
                    = self.msgsets_from_trace_identifier(
                        "/tmp/analysis/trace/trace_cpu_ctf", 
                        debug=True)
            elif self.hardware_device_type == "fpga":
                # NOTE: can't use msgsets_from_trace_identifier because vtf traces
                # won't have the unique identifier
                self.image_pipeline_msg_sets = self.msgsets_from_ctf_vtf_traces(
                    "/tmp/analysis/trace/trace_cpu_ctf",
                    "/tmp/analysis/trace/trace_fpga_vtf_ctf_fix",
                    True)

    def get_index_to_plot_latency(self):
        """ Obtain the index to plot given a series of sets

        # Implementation 1
        Obtains the Panda DataFrame of the corresponding sets,
        calculates the sum of latencies, obtains the max and 
        fetches the index.

        # Implemetation 2
        Index at the middle of the sets
        """

        # Implementation 1
        # figure out the index of the set with the max value (longest, latency-wise)
        max_sum = sum(self.image_pipeline_msg_sets_barchart[0])
        max_index = 0
        for i, lst in enumerate(self.image_pipeline_msg_sets_barchart):
            current_sum = sum(lst)            
            if current_sum > max_sum:
                max_sum = current_sum
                max_index = i       

        index_to_plot =  max_index
        # # debug
        # print(self.image_pipeline_msg_sets_barchart[max_index])
        # print(sum(self.image_pipeline_msg_sets_barchart[max_index]))

        # # Implementation 2
        # index_to_plot = len(self.image_pipeline_msg_sets)//2
        # if len(self.image_pipeline_msg_sets) < 1:
        #     print(color("No msg sets found", fg="red"))
        #     sys.exit(1)

        return index_to_plot

    def print_timing_pipeline(self):
        if self.image_pipeline_msg_sets: 
            self.print_timeline([self.image_pipeline_msg_sets[self.index_to_plot]])     # timeline of max
            # self.print_timeline(self.image_pipeline_msg_sets)                         # all timelines
            # self.print_timeline_average(self.image_pipeline_msg_sets)                 # timeline of averages, NOTE only totals are of interest

    def draw_tracepoints(self):        
        msg_set = self.image_pipeline_msg_sets[self.index_to_plot]
        if self.benchmark_name == "a1_perception_2nodes":
            # self.traces(msg_set)
            self.traces_id(msg_set)
        elif self.benchmark_name == "a1_perception_2nodes_fpga":
            self.traces_fpga(msg_set)

    def bar_charts_latency(self):
        self.image_pipeline_msg_sets_barchart = self.barchart_data_latency(self.image_pipeline_msg_sets)


    def plot_latency_results(self):
        # Plot, either averages or latest, etc

        # # TODO: the plotting code below doesn't work with a5_resize
        # if self.benchmark_name != "a1_perception_2nodes":
        #     sys.exit()

        image_pipeline_msg_sets_mean = pd.DataFrame(self.image_pipeline_msg_sets_barchart).mean()        
        image_pipeline_msg_sets_max = pd.DataFrame(self.image_pipeline_msg_sets_barchart).max()
        image_pipeline_msg_sets_index = pd.DataFrame(self.barchart_data_latency(self.image_pipeline_msg_sets[self.index_to_plot])).transpose()[0]
        image_pipeline_msg_sets_index = image_pipeline_msg_sets_index.rename(None)

        df_mean = pd.concat(
            [
                image_pipeline_msg_sets_index,
                image_pipeline_msg_sets_mean,
                image_pipeline_msg_sets_max,
            ], axis=1).transpose()
        df_mean.columns = self.target_chain_dissambiguous
        substrates = pd.DataFrame({'substrate':
            [
                "RobotPerf benchmark:" + self.benchmark_name + "(instance)",
                "RobotPerf benchmark:" + self.benchmark_name + "(mean)",
                "RobotPerf benchmark:" + self.benchmark_name + "(max)",
            ]})
        df_mean = df_mean.join(substrates)

        import plotly.express as px
        fig = px.bar(
            df_mean,
            template="plotly_white",
            x="substrate",
            y=self.target_chain_dissambiguous,
            color_discrete_sequence=px.colors.sequential.Inferno + px.colors.diverging.BrBG,
            # colors at https://plotly.com/python/discrete-color/
        )
        fig.update_xaxes(title_text = "")
        fig.update_yaxes(title_text = "Milliseconds")
        # fig.show()
        fig.write_image("/tmp/analysis/plot_barchart.png", width=1400, height=1000)


        # ///////////////////
        # Add results into robotperf/benchmarks repo

        path_repo = "/tmp/benchmarks"
        branch_name = ""
        result = self.results(self.image_pipeline_msg_sets_barchart)

        # # fetch repo
        # run('if [ -d "/tmp/benchmarks" ]; then cd ' + path_repo +  ' && git pull; \
        #         else cd /tmp && git clone https://github.com/robotperf/benchmarks; fi',
        #     shell=True)

        if os.path.exists(path_repo):
            benchmark_meta_paths = search_benchmarks(searchpath="/tmp/benchmarks")
            for meta in benchmark_meta_paths:
                benchmark = Benchmark(meta)
                if benchmark.name == self.benchmark_name:
                    benchmark.results.append(result)
                    branch_name = benchmark.id + "-" + str(len(benchmark.results))
                    with open(meta, 'w') as file:
                        file.write(str(benchmark))
                    print(benchmark)

    def upload_results():
        # commit and push in a new branch called "branch_name" and drop instructions to create a PR
        # NOTE: conflicts with permissions
        #   - fatal: could not read Username for 'https://github.com': No such device or address
        #   - Try authenticating with:  gh auth login
        run('cd /tmp/benchmarks && git checkout -b ' + branch_name + ' \
            && git add . \
            && git config --global user.email "victor@accelerationrobotics.com" \
            && git config --global user.name "Víctor Mayoral-Vilches" \
            && git commit -m "' + self.benchmark_name + ' results for ' + os.environ.get('HARDWARE') + ' (' + str(result["value"]) + ')\n \
            - CI_PIPELINE_URL: ' + os.environ.get('CI_PIPELINE_URL') + '\n \
            - CI_JOB_URL: ' + os.environ.get('CI_JOB_URL') + '"'
            , shell=True)
            # && git push origin ' + branch_name + ' \
            # && gh pr create --title "Add result" --body "Add result"'

        # show message of last git commit
        outs, err = run('cd /tmp/benchmarks && git log -1', shell=True)
        print(outs)

    def analyze_latency(self, tracepath=None):
        """Analyze latency of the image pipeline

        Args:
            tracepath (string, optional):
                Path of the CTF tracefiles. Defaults to None.
        """
        self.get_target_chain_traces(tracepath)        
        self.bar_charts_latency()
        self.index_to_plot = self.get_index_to_plot_latency()
        self.print_timing_pipeline()
        self.draw_tracepoints()
        self.print_markdown_table(
            [self.image_pipeline_msg_sets_barchart],
            ["RobotPerf benchmark"],
            from_baseline=False,
            units='ms'
        )
        self.plot_latency_results()
        # self.upload_results()  # performed in CI/CD pipelines instead


    def analyze_throughput(self, tracepath=None):
        """Analyze throughput of the image pipeline

        Args:
            tracepath (string, optional):
                Path of the CTF tracefiles. Defaults to None.
        """
        self.get_target_chain_traces(tracepath)        
        barcharts_through_megabys, barcharts_through_fps = self.barchart_data_throughput(self.image_pipeline_msg_sets, 'potential')
        
        self.print_markdown_table_1d(
            [barcharts_through_megabys],
            ["RobotPerf potential throughput"],
            from_baseline=False,
            units='MB/s'
        )

        self.print_markdown_table_1d(
            [barcharts_through_fps],
            ["RobotPerf potential throughput"],
            from_baseline=False,
            units='fps'
        )

        barcharts_through_megabys, barcharts_through_fps = self.barchart_data_throughput(self.image_pipeline_msg_sets, 'real')
        
        self.print_markdown_table_1d(
            [barcharts_through_megabys],
            ["RobotPerf real throughput"],
            from_baseline=False,
            units='MB/s'
        )

        self.print_markdown_table_1d(
            [barcharts_through_fps],
            ["RobotPerf real throughput"],
            from_baseline=False,
            units='fps'
        )

        