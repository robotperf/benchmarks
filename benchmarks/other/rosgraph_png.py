#!/usr/bin/env python3

import sys

import rclpy
import time
import graphviz

from rqt_graph.ros_graph import *
from rqt_graph.dotcode import \
    RosGraphDotcodeGenerator, NODE_NODE_GRAPH, NODE_TOPIC_ALL_GRAPH, NODE_TOPIC_GRAPH
from rqt_graph.rosgraph2_impl import Graph
from qt_dotgraph.pydotfactory import PydotFactory

dot_file = '/tmp/graph.dot'
time_update = 3

def main():

    if len(sys.argv) < 2:
        print('Usage: {} <output_png>, defaulting to /tmp/analysis/graph.png'.format(sys.argv[0]))  # e.g. rosgraph_png.py /tmp/benchmark_ws/src/benchmarks/graph.png
        output_png = "/tmp/analysis/graph"
    else:
        output_png = sys.argv[1].split('.png')[0]

    rclpy.init()
    node = rclpy.create_node('_my_node')
    
    dotcode_generator = RosGraphDotcodeGenerator(node)
    time.sleep(time_update)  # give some time to the node to discover the graph

    # create a new graph and dump it in dot format
    graph = Graph(node)
    graph.set_node_stale(5.0)
    graph.update()
    dotcode_factory = PydotFactory()

    dot = dotcode_generator.generate_dotcode(
        graph,
        "", # node filter 
        "", # topic filter
        "NODE_TOPIC_GRAPH",  # "NODE_TOPIC_ALL_GRAPH",
        dotcode_factory,
        hide_dead_end_topics=True,
        quiet=True,
        cluster_namespaces_level=2,
        unreachable=True,
        hide_dynamic_reconfigure=True,
        )

    # save dot string to file
    with open(dot_file, "w") as f:
        f.write(dot)

        # Create a new Graphviz graph from the DOT file
        graph = graphviz.Source.from_file(dot_file)
        # Render the graph to a PNG file
        graph.format = 'png'
        graph.render(filename=output_png, cleanup=True)    

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
