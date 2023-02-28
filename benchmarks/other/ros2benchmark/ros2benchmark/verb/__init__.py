# Copyright (C) Acceleration Robotics S.L.U. - All Rights Reserved
#
# Written by Víctor Mayoral-Vilches <victor@accelerationrobotics.com>
# Licensed under the Apache License, Version 2.0

import os
import subprocess
import sys
import yaml

from ros2cli.plugin_system import PLUGIN_SYSTEM_VERSION
from ros2cli.plugin_system import satisfies_version


class VerbExtension:
    """
    The extension point for 'acceleration' verb extensions.

    The following properties must be defined:
    * `NAME` (will be set to the entry point name)

    The following methods must be defined:
    * `main`

    The following methods can be defined:
    * `add_arguments`
    """

    NAME = None
    EXTENSION_POINT_VERSION = '0.1'

    def __init__(self):
        super(VerbExtension, self).__init__()
        satisfies_version(PLUGIN_SYSTEM_VERSION, '^0.1')

    def add_arguments(self, parser, cli_name):
        pass

    def main(self, *, args):
        raise NotImplementedError()

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
    

def black(text):
    print("\033[30m", text, "\033[0m", sep="")


def red(text):
    print("\033[31m", text, "\033[0m", sep="")


def redinline(text):
    print("\033[31m", text, "\033[0m", end="")


def green(text):
    print("\033[32m", text, "\033[0m", sep="")


def greeninline(text):
    print("\033[32m", text, "\033[0m", end='')


def yellow(text):
    print("\033[33m", text, "\033[0m", sep="")


def yellowinline(text):
    print("\033[33m", text, "\033[0m", end="")


def blue(text):
    print("\033[34m", text, "\033[0m", sep="")


def magenta(text):
    print("\033[35m", text, "\033[0m", sep="")


def cyan(text):
    print("\033[36m", text, "\033[0m", sep="")


def gray(text):
    print("\033[90m", text, "\033[0m", sep="")


def grayinline(text):
    print("\033[90m", text, "\033[0m", end="")


class Benchmark:
    def __init__(self, yaml_file):
        with open(yaml_file, "r") as f:
            yaml_data = yaml.safe_load(f)

        self.id = yaml_data["id"]
        self.name = yaml_data["name"]
        self.description = yaml_data["description"]
        self.short = yaml_data["short"]
        self.graph = yaml_data["graph"]
        self.metric = yaml_data["metric"]
        self.reproduction = yaml_data["reproduction"]
        self.results = []

        for result in yaml_data["results"]:
            hardware = result["result"]["hardware"]
            category = result["result"]["category"]
            timestampt = result["result"]["timestampt"]
            value = result["result"]["value"]
            note = result["result"]["note"]
            datasource = result["result"]["datasource"]

            self.results.append({
                "hardware": hardware,
                "category": category,
                "timestampt": timestampt,
                "value": value,
                "note": note,
                "datasource": datasource
            })

    def __str__(self):
        yaml_data = {
            "id": self.id,
            "name": self.name,
            "description": self.description,
            "short": self.short,
            "graph": self.graph,
            "metric": self.metric,
            "reproduction": self.reproduction,
            "results": [{"result": result} for result in self.results]
        }

        key_order = ["id", "name", "description", "short", "graph", "metric", "reproduction", "results"]
        return yaml.dump(yaml_data, sort_keys=key_order)