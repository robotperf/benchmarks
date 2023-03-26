#
#    @@@@@@@@@@@@@@@@@@@@
#    @@@@@@@@@&@@@&&@@@@@
#    @@@@@ @@  @@    @@@@
#    @@@@@ @@  @@    @@@@
#    @@@@@ @@  @@    @@@@ Copyright (c) 2023, Acceleration Robotics®
#    @@@@@ @@  @@    @@@@ Author: Víctor Mayoral Vilches <victor@accelerationrobotics.com>
#    @@@@@ @@  @@    @@@@
#    @@@@@@@@@&@@@@@@@@@@
#    @@@@@@@@@@@@@@@@@@@@
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

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
from launch import LaunchDescription
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

def get_change(first, second):
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


def msgsets_from_trace(tracename, debug=False):
    """
    Returns a list of message sets ready to be used
    for plotting them in various forms.

    NOTE: NOT coded for multiple Nodes running concurrently or multithreaded executors
    Classification expects events in the corresponding order.
    """
    global target_chain

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
            if "ros2" in event.name or "robotperf" in event.name:
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
        if image_pipeline_msgs[index].event.name in target_chain:  # optimization

            if debug:
                print("---")
                print("new: " + image_pipeline_msgs[index].event.name)
                print("expected: " + str(target_chain[chain_index]))
                print("chain_index: " + str(chain_index))

            # first one            
            if (
                chain_index == 0
                and image_pipeline_msgs[index].event.name == target_chain[chain_index]
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
                image_pipeline_msgs[index].event.name == target_chain[chain_index]
                and target_chain[chain_index] == target_chain[-1]
                and new_set[-1].event.name == target_chain[-2]
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
                image_pipeline_msgs[index].event.name == target_chain[chain_index]
                and image_pipeline_msgs[index].event.common_context_field.get("vpid")
                == vpid_chain
            ):
                new_set.append(image_pipeline_msgs[index])
                chain_index += 1
                if debug:
                    print(color("Found: " + str(image_pipeline_msgs[index].event.name) + " - " + str([x.event.name for x in new_set]), fg="green"))
            # altered order
            elif (
                image_pipeline_msgs[index].event.name in target_chain
                and image_pipeline_msgs[index].event.common_context_field.get("vpid")
                == vpid_chain
            ):
                # pop ros2:callback_start in new_set, if followed by "ros2:callback_end"
                # NOTE: consider case of disconnected series of:
                #       "ros2:callback_start"
                #       "ros2:callback_end"
                if (image_pipeline_msgs[index].event.name == "ros2:callback_end"
                    and target_chain[chain_index - 1] == "ros2:callback_start"):
                    new_set.pop()
                    chain_index -= 1
                # # it's been observed that "robotperf_benchmarks:robotperf_image_input_cb_init" triggers
                # # before "ros2_image_pipeline:image_proc_rectify_cb_fini" which leads to trouble
                # # Skip this as well as the next event
                # elif (image_pipeline_msgs[index].event.name == "robotperf_benchmarks:robotperf_image_input_cb_init"
                #     and target_chain[chain_index - 3] == "ros2_image_pipeline:image_proc_rectify_cb_fini"):
                #     print(color("Skipping: " + str(image_pipeline_msgs[index].event.name), fg="yellow"))
                # elif (image_pipeline_msgs[index].event.name == "robotperf_benchmarks:robotperf_image_input_cb_fini"
                #     and target_chain[chain_index - 3] == "ros2_image_pipeline:image_proc_rectify_cb_fini"):
                #     print(color("Skipping: " + str(image_pipeline_msgs[index].event.name), fg="yellow"))
                else:
                    new_set.append(image_pipeline_msgs[index])
                    if debug:
                        print(color("Altered order: " + str([x.event.name for x in new_set]) + ", restarting", fg="red"))
                    chain_index = 0  # restart
                    new_set = []  # restart
    return image_pipeline_msg_sets

def barplot_all(image_pipeline_msg_sets, title="Barplot"):
    global target_chain
    global target_chain_dissambiguous

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
    df.columns = target_chain_dissambiguous
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


def traces(msg_set):
    global target_chain_colors_fg_bokeh
    global segment_types
    global target_chain_marker
    global target_chain
    global target_chain_layer

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

    # print("1")

    # draw durations
    ## robotperf_image_input_cb_fini-robotperf_image_output_cb_init duration
    callback_start = (target_chain_ns[2] - init_ns) / 1e6
    callback_end = (target_chain_ns[17] - init_ns) / 1e6
    duration = callback_end - callback_start
    add_durations_to_figure(
        fig,
        target_chain_layer[2],  # index used in here
                                # should match with the
                                # one from the callback_start
        [(callback_start, callback_start + duration, duration)],
        "khaki",
    )

    ## rclcpp callbacks - robotperf_image_input_cb_init
    callback_start = (target_chain_ns[0] - init_ns) / 1e6
    callback_end = (target_chain_ns[3] - init_ns) / 1e6
    duration = callback_end - callback_start
    add_durations_to_figure(
        fig,
        target_chain_layer[0],  # index used in here
                                # should match with the
                                # one from the callback_start
        [(callback_start, callback_start + duration, duration)],
        "lightgray",
    )

    ## rclcpp callbacks - rectify
    callback_start = (target_chain_ns[4] - init_ns) / 1e6
    callback_end = (target_chain_ns[9] - init_ns) / 1e6
    duration = callback_end - callback_start
    add_durations_to_figure(
        fig,
        target_chain_layer[0],  # index used in here
                                # should match with the
                                # one from the callback_start
        [(callback_start, callback_start + duration, duration)],
        "lightgray",
    )

    ## rclcpp callbacks - resize
    callback_start = (target_chain_ns[10] - init_ns) / 1e6
    callback_end = (target_chain_ns[15] - init_ns) / 1e6
    duration = callback_end - callback_start
    add_durations_to_figure(
        fig,
        target_chain_layer[10],  # index used in here
                                # should match with the
                                # one from the callback_start
        [(callback_start, callback_start + duration, duration)],
        "lightgray",
    )

    ## rclcpp callbacks - robotperf_image_output_cb_init
    callback_start = (target_chain_ns[16] - init_ns) / 1e6
    callback_end = (target_chain_ns[19] - init_ns) / 1e6
    duration = callback_end - callback_start
    add_durations_to_figure(
        fig,
        target_chain_layer[16],  # index used in here
                                # should match with the
                                # one from the callback_start
        [(callback_start, callback_start + duration, duration)],
        "lightgray",
    )

    ## rectify callback
    callback_start = (target_chain_ns[5] - init_ns) / 1e6
    callback_end = (target_chain_ns[8] - init_ns) / 1e6
    duration = callback_end - callback_start
    add_durations_to_figure(
        fig,
        target_chain_layer[5],  # index used in here
                                # should match with the
                                # one from the callback_start
        [(callback_start, callback_start + duration, duration)],
        "whitesmoke",
    )

    ## rectify op
    callback_start = (target_chain_ns[6] - init_ns) / 1e6
    callback_end = (target_chain_ns[7] - init_ns) / 1e6
    duration = callback_end - callback_start
    add_durations_to_figure(
        fig,
        target_chain_layer[6],  # index used in here
                                # should match with the
                                # one from the callback_start
        [(callback_start, callback_start + duration, duration)],
        "seashell",
    )

    ## resize callback
    callback_start = (target_chain_ns[11] - init_ns) / 1e6
    callback_end = (target_chain_ns[14] - init_ns) / 1e6
    duration = callback_end - callback_start
    add_durations_to_figure(
        fig,
        target_chain_layer[11],  # index used in here
                                # should match with the
                                # one from the callback_start
        [(callback_start, callback_start + duration, duration)],
        "whitesmoke",
    )
    ## resize op
    callback_start = (target_chain_ns[12] - init_ns) / 1e6
    callback_end = (target_chain_ns[13] - init_ns) / 1e6
    duration = callback_end - callback_start
    add_durations_to_figure(
        fig,
        target_chain_layer[12],  # index used in here
                                # should match with the
                                # one from the callback_start
        [(callback_start, callback_start + duration, duration)],
        "seashell",
    )

    ## robotperf_image_input_cb_init callback
    callback_start = (target_chain_ns[1] - init_ns) / 1e6
    callback_end = (target_chain_ns[2] - init_ns) / 1e6
    duration = callback_end - callback_start
    add_durations_to_figure(
        fig,
        target_chain_layer[1],  # index used in here
                                # should match with the
                                # one from the callback_start
        [(callback_start, callback_start + duration, duration)],
        "whitesmoke",
    )

    ## robotperf_image_output_cb_init callback
    callback_start = (target_chain_ns[17] - init_ns) / 1e6
    callback_end = (target_chain_ns[18] - init_ns) / 1e6
    duration = callback_end - callback_start
    add_durations_to_figure(
        fig,
        target_chain_layer[17],  # index used in here
                                # should match with the
                                # one from the callback_start
        [(callback_start, callback_start + duration, duration)],
        "whitesmoke",
    )

    # print("2")

    for msg_index in range(len(msg_set)):
        #     add_markers_to_figure(fig, msg_set[msg_index].event.name, [(target_chain_ns[msg_index] - init_ns)/1e6], 'blue', marker_type='plus', legend_label='timing')
        # print("marker ms: " + str((target_chain_ns[msg_index] - init_ns) / 1e6))
        add_markers_to_figure(
            fig,
            target_chain_layer[msg_index],
            [(target_chain_ns[msg_index] - init_ns) / 1e6],
            target_chain_colors_fg_bokeh[msg_index],
            marker_type=target_chain_marker[msg_index],
            # legend_label=msg_set[msg_index].event.name,
            legend_label=target_chain_dissambiguous[msg_index],
            size=10,
        )        
        if "robotperf_image_input_cb_fini" in msg_set[msg_index].event.name:
            label = Label(
                x=(target_chain_ns[msg_index] - init_ns) / 1e6,
                y=target_chain_label_layer[msg_index],
                x_offset=-40,
                y_offset=-40,
                text=target_chain_dissambiguous[msg_index].split(":")[-1],
            )

        elif "robotperf_image_output_cb_init" in msg_set[msg_index].event.name:
            label = Label(
                x=(target_chain_ns[msg_index] - init_ns) / 1e6,
                y=target_chain_label_layer[msg_index],
                x_offset=-200,
                y_offset=-40,
                text=target_chain_dissambiguous[msg_index].split(":")[-1],
            )
        else:
            label = Label(
                x=(target_chain_ns[msg_index] - init_ns) / 1e6,
                y=target_chain_label_layer[msg_index],
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


def barchart_data(image_pipeline_msg_sets):
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


def print_timeline(image_pipeline_msg_sets):
    global target_chain
    global target_chain_colors_fg

    for msg_set in image_pipeline_msg_sets:
        if len(msg_set) != len(target_chain):
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
                    fg=target_chain_colors_fg[msg_index],
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


def rms(list):
    return np.sqrt(np.mean(np.array(list) ** 2))


def mean(list):
    return np.mean(np.array(list))


def max(list):
    return np.max(np.array(list))


def min(list):
    return np.min(np.array(list))


def rms_sets(image_pipeline_msg_sets, indices=None):
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
        return rms(with_indices_sets)
    else:
        total_in_sets = [sum(set) for set in image_pipeline_msg_sets]
        return rms(total_in_sets)


def mean_sets(image_pipeline_msg_sets, indices=None):
    """

    """
    if indices:
        with_indices_sets = []
        for set in image_pipeline_msg_sets:
            indices_sum = 0
            for i in indices:
                indices_sum += set[i]
            with_indices_sets.append(indices_sum)
        return mean(with_indices_sets)
    else:
        total_in_sets = [sum(set) for set in image_pipeline_msg_sets]
        return mean(total_in_sets)


def max_sets(image_pipeline_msg_sets, indices=None):
    if indices:
        with_indices_sets = []
        for set in image_pipeline_msg_sets:
            indices_sum = 0
            for i in indices:
                indices_sum += set[i]
            with_indices_sets.append(indices_sum)
        return max(with_indices_sets)
    else:
        total_in_sets = [sum(set) for set in image_pipeline_msg_sets]
        return max(total_in_sets)


def min_sets(image_pipeline_msg_sets, indices=None):
    if indices:
        with_indices_sets = []
        for set in image_pipeline_msg_sets:
            indices_sum = 0
            for i in indices:
                indices_sum += set[i]
            with_indices_sets.append(indices_sum)
        return min(with_indices_sets)
    else:
        total_in_sets = [sum(set) for set in image_pipeline_msg_sets]
        return min(total_in_sets)


def print_timeline_average(image_pipeline_msg_sets):
    """
    Doing averages may lead to negative numbers while substracting the previous average.
    This is only useful to get an intuition of the totals.
    """
    global target_chain
    global target_chain_colors_fg

    image_pipeline_msg_sets_ns = []
    for msg_set in image_pipeline_msg_sets:
        if len(msg_set) != len(target_chain):
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
            fg=target_chain_colors_fg[msg_index],
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


def statistics(image_pipeline_msg_sets_ms, verbose=False):
    global target_chain_dissambiguous

    mean_ = mean_sets(image_pipeline_msg_sets_ms)
    rms_ = rms_sets(image_pipeline_msg_sets_ms)
    min_ = min_sets(image_pipeline_msg_sets_ms)
    max_ = max_sets(image_pipeline_msg_sets_ms)

    indices = [i for i in range(
                target_chain_dissambiguous.index("ros2:callback_end"),
                1 + target_chain_dissambiguous.index("robotperf_benchmarks:robotperf_image_output_cb_init"),
                )
              ]

    mean_benchmark = mean_sets(image_pipeline_msg_sets_ms,indices)
    rms_benchmark = rms_sets(image_pipeline_msg_sets_ms, indices)
    max_benchmark = max_sets(image_pipeline_msg_sets_ms, indices)
    min_benchmark = min_sets(image_pipeline_msg_sets_ms, indices)

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


def table(list_sets, list_sets_names, from_baseline=True):
    """
    Creates a markdown table from a list of sets

    :param: list_sets: list of processed data (resulting from barchart_data) to display
    :param: list_sets_names: list of names to display
    :param: from_baseline: whether to show % from baseline

    NOTE: assumes base is always the first set in list_sets, which
    is then used to calculate % of change.
    """

    list_statistics = []
    # generate statistics
    for sets in list_sets:
        list_statistics.append(statistics(sets))

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
                                "**{:.2f}** ms".format(row[element_index])
                                + " (:small_red_triangle_down: `"
                                + "{:.2f}".format(
                                    get_change(row[element_index], baseline[element_index])
                                )
                                + "`%) | "
                            )
                        else:
                            row_str += (
                                "**{:.2f}** ms".format(row[element_index])
                                + " (`"
                                + "{:.2f}".format(
                                    get_change(row[element_index], baseline[element_index])
                                )
                                + "`%) | "
                            )
                    else:
                        row_str += ("**{:.2f}** ms".format(row[element_index]) + " | ")
                else:
                    row_str += row[element_index] + " | "

        else:
            for element_index in range(len(row)):
                if type(row[element_index]) != str:
                    if from_baseline:
                        if row[element_index] > baseline[element_index]:
                            row_str += (
                                "{:.2f} ms".format(row[element_index])
                                + " (:small_red_triangle_down: `"
                                + "{:.2f}".format(
                                    get_change(row[element_index], baseline[element_index])
                                )
                                + "`%) | "
                            )
                        else:
                            row_str += (
                                "{:.2f} ms".format(row[element_index])
                                + " (`"
                                + "{:.2f}".format(
                                    get_change(row[element_index], baseline[element_index])
                                )
                                + "`%) | "
                            )
                    else:
                        row_str += ("{:.2f} ms".format(row[element_index]) + " | ")

                else:
                    row_str += row[element_index] + " | "
        count += 1
        print(row_str)

        # if count == 2:
        #     row = "|" + "|".join("**{:.2f}** ms".format(row[element_index]) + " (`"
        #             + "{:.2f}".format(get_change(row[element_index], baseline[element_index])) + "`%)"
        #         if type(row[element_index]) != str
        #         else row[element_index]
        #             for element_index in range(len(row))) + "|"
        # else:
        #     row = "|" + "|".join("{:.2f} ms".format(row[element_index]) + " (`"
        #             + "{:.2f}".format(get_change(row[element_index], baseline[element_index])) + "`%)"
        #         if type(row[element_index]) != str else row[element_index]
        #             for element_index in range(len(row))) + "|"
        # count += 1
        # print(row)


def results(sets):
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
    statistics_data = statistics(sets)

    print(statistics_data[2])
    return {
            "hardware": os.environ.get('HARDWARE'),
            "category": os.environ.get('CATEGORY'),
            "timestampt": time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(time.time())),
            "value": float(statistics_data[2]),
            "note": "mean_benchmark {}, rms_benchmark {}, max_benchmark {}, min_benchmark {}".format(statistics_data[0], statistics_data[1], statistics_data[2], statistics_data[3]),
            "datasource": os.environ.get('ROSBAG')
        }


def run(cmd, shell=False, timeout=1):
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
    


def generate_launch_description():
    return LaunchDescription()


##############################
##############################

# targeted chain of messages for tracing
# NOTE: there're not "publish" tracepoints because
# graph's using inter-process communications
#
target_chain = [
    "ros2:callback_start",
    "robotperf_benchmarks:robotperf_image_input_cb_init",
    "robotperf_benchmarks:robotperf_image_input_cb_fini",
    "ros2:callback_end",
    "ros2:callback_start",
    "ros2_image_pipeline:image_proc_rectify_cb_init",
    "ros2_image_pipeline:image_proc_rectify_init",
    "ros2_image_pipeline:image_proc_rectify_fini",
    # "ros2:rclcpp_publish",
    # "ros2:rcl_publish",
    # "ros2:rmw_publish",
    "ros2_image_pipeline:image_proc_rectify_cb_fini",
    "ros2:callback_end",
    "ros2:callback_start",
    "ros2_image_pipeline:image_proc_resize_cb_init",
    "ros2_image_pipeline:image_proc_resize_init",
    "ros2_image_pipeline:image_proc_resize_fini",
    # "ros2:rclcpp_publish",
    # "ros2:rcl_publish",
    # "ros2:rmw_publish",
    "ros2_image_pipeline:image_proc_resize_cb_fini",
    "ros2:callback_end",
    "ros2:callback_start",
    "robotperf_benchmarks:robotperf_image_output_cb_init",
    "robotperf_benchmarks:robotperf_image_output_cb_fini",
    "ros2:callback_end",
]
target_chain_dissambiguous = [
    "ros2:callback_start",
    "robotperf_benchmarks:robotperf_image_input_cb_init",
    "robotperf_benchmarks:robotperf_image_input_cb_fini",
    "ros2:callback_end",
    "ros2:callback_start (2)",
    "ros2_image_pipeline:image_proc_rectify_cb_init",
    "ros2_image_pipeline:image_proc_rectify_init",
    "ros2_image_pipeline:image_proc_rectify_fini",
    # "ros2:rclcpp_publish",
    # "ros2:rcl_publish",
    # "ros2:rmw_publish",
    "ros2_image_pipeline:image_proc_rectify_cb_fini",
    "ros2:callback_end (2)",
    "ros2:callback_start (3)",
    "ros2_image_pipeline:image_proc_resize_cb_init",
    "ros2_image_pipeline:image_proc_resize_init",
    "ros2_image_pipeline:image_proc_resize_fini",
    # "ros2:rclcpp_publish (2)",
    # "ros2:rcl_publish (2)",
    # "ros2:rmw_publish (2)",
    "ros2_image_pipeline:image_proc_resize_cb_fini",
    "ros2:callback_end (3)",
    "ros2:callback_start (4)",
    "robotperf_benchmarks:robotperf_image_output_cb_init",
    "robotperf_benchmarks:robotperf_image_output_cb_fini",
    "ros2:callback_end (4)",
]
target_chain_colors_fg = [
    "blue",
    "blue",
    "blue",
    "blue",
    "blue",
    "yellow",
    "red",
    "red",
    # "blue",
    # "blue",
    # "blue",
    "yellow",
    "blue",
    "blue",
    "yellow",
    "red",
    "red",
    # "blue",
    # "blue",
    # "blue",
    "yellow",
    "blue",
    "blue",
    "blue",
    "blue",
    "blue",    
]
target_chain_colors_fg_bokeh = [
    "lightgray",
    "silver",
    "darkgray",
    "gray",
    "lightsalmon",
    "salmon",
    "darksalmon",
    "lightcoral",
    # "indianred",
    # "crimson",
    # "firebrick",
    "darkred",
    "red",
    "lavender",
    "thistle",
    "plum",
    "fuchsia",
    # "mediumorchid",
    # "mediumpurple",
    # "darkmagenta",
    "indigo",
    "mediumslateblue",
    "chartreuse",
    "chocolate",
    "coral",
    "cornflowerblue",    
]
target_chain_layer = [
    "rclcpp",
    "userland",
    "benchmark",
    "rclcpp",
    "rclcpp",
    "userland",
    "userland",
    "userland",
    # "rclcpp",
    # "rcl",
    # "rmw",
    "userland",
    "rclcpp",
    "rclcpp",
    "userland",
    "userland",
    "userland",
    # "rclcpp",
    # "rcl",
    # "rmw",
    "userland",
    "rclcpp",
    "rclcpp",
    "benchmark",
    "userland",
    "rclcpp",    
]
target_chain_label_layer = [  # associated with the layer
    3,
    4,
    5,
    3,
    3,
    4,
    4,
    4,
    # 3,
    # 2,
    # 1,
    4,
    3,
    3,
    4,
    4,
    4,
    # 3,
    # 2,
    # 1,
    4,
    3,
    3,
    5,
    4,
    3,
]
target_chain_marker = [
    "diamond",
    "plus",
    "plus",
    "diamond",
    "diamond",
    "plus",
    "plus",
    "plus",
    # "plus",
    # "plus",
    # "plus",
    "plus",
    "diamond",
    "diamond",
    "plus",
    "plus",
    "plus",
    # "plus",
    # "plus",
    # "plus",
    "plus",
    "diamond",
    "diamond",
    "plus",
    "plus",
    "diamond",    
]
# For some reason it seems to be displayed in the reverse order on the Y axis
segment_types = ["rmw", "rcl", "rclcpp", "userland", "benchmark"]

image_pipeline_msg_sets = msgsets_from_trace("/tmp/analysis/trace/trace_cpu_ctf", True)
# image_pipeline_msg_sets = msgsets_from_trace("/tmp/benchmark_ws/src/benchmarks/trace_old/trace_cpu_ctf")
# image_pipeline_msg_sets = msgsets_from_trace("/tmp/benchmark_ws/src/benchmarks/trace/trace_cpu_ctf", True)
index_to_plot = len(image_pipeline_msg_sets)//2
if len(image_pipeline_msg_sets) < 1:
    print(color("No msg sets found", fg="red"))
    sys.exit(1)

####################
# print timing pipeline
####################
if image_pipeline_msg_sets: 
    print_timeline([image_pipeline_msg_sets[index_to_plot]])  # timeline of last message
    # print(len(image_pipeline_msg_sets))
    # print_timeline(image_pipeline_msg_sets)  # all timelines
    # print_timeline_average(image_pipeline_msg_sets)  # timeline of averages, NOTE only totals are of interest

######################
# draw tracepoints
######################
msg_set = image_pipeline_msg_sets[index_to_plot]
traces(msg_set)


######################
# draw bar charts
######################
image_pipeline_msg_sets_barchart = barchart_data(image_pipeline_msg_sets)

# ///////////////////
# Markdown Table results
# ///////////////////
table(
        [image_pipeline_msg_sets_barchart,],
        ["RobotPerf benchmark"],
        from_baseline=False
    )

#///////////////////
# Plot, either averages or latest, etc
#///////////////////

image_pipeline_msg_sets_mean = pd.DataFrame(image_pipeline_msg_sets_barchart).mean()
image_pipeline_msg_sets_max = pd.DataFrame(image_pipeline_msg_sets_barchart).max()
image_pipeline_msg_sets_index = pd.DataFrame(barchart_data(image_pipeline_msg_sets[index_to_plot])).transpose()[0]
image_pipeline_msg_sets_index = image_pipeline_msg_sets_index.rename(None)

df_mean = pd.concat(
    [
        image_pipeline_msg_sets_index,
        image_pipeline_msg_sets_mean,
        image_pipeline_msg_sets_max,
    ], axis=1).transpose()
df_mean.columns = target_chain_dissambiguous
substrates = pd.DataFrame({'substrate':
    [
        "RobotPerf benchmark: a1_perception_2nodes (instance)",
        "RobotPerf benchmark: a1_perception_2nodes (mean)",
        "RobotPerf benchmark: a1_perception_2nodes (max)",
    ]})
df_mean = df_mean.join(substrates)

import plotly.express as px
fig = px.bar(
    df_mean,
    template="plotly_white",
    x="substrate",
    y=target_chain_dissambiguous,
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
benchmark_name = "a1_perception_2nodes"
result = results(image_pipeline_msg_sets_barchart)
# fetch repo
run('if [ -d "/tmp/benchmarks" ]; then cd ' + path_repo +  ' && git pull; \
        else cd /tmp && git clone https://github.com/robotperf/benchmarks; fi',
    shell=True)

# add result
benchmark_meta_paths = search_benchmarks(searchpath="/tmp/benchmarks")
for meta in benchmark_meta_paths:
    benchmark = Benchmark(meta)
    if benchmark.name == benchmark_name:
        benchmark.results.append(result)
        branch_name = benchmark.id + "-" + str(len(benchmark.results))
        with open(meta, 'w') as file:
            file.write(str(benchmark))
        print(benchmark)


# # commit and push in a new branch called "branch_name" and drop instructions to create a PR
# # NOTE: conflicts with permissions
# #   - fatal: could not read Username for 'https://github.com': No such device or address
# #   - Try authenticating with:  gh auth login
run('cd /tmp/benchmarks && git checkout -b ' + branch_name + ' \
    && git add . \
    && git config --global user.email "victor@accelerationrobotics.com" \
    && git config --global user.name "Víctor Mayoral-Vilches" \
    && git commit -m "' + benchmark.id + ' results for ' + os.environ.get('HARDWARE') + ' (CI_PIPELINE_ID: ' + os.environ.get('CI_PIPELINE_ID') + ')"'
    , shell=True)
    # && git push origin ' + branch_name + ' \
    # && gh pr create --title "Add result" --body "Add result"'
