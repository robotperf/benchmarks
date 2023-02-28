# Copyright (C) Acceleration Robotics S.L.U. - All Rights Reserved
#
# Written by Víctor Mayoral-Vilches <victor@accelerationrobotics.com>
# Licensed under the Apache License, Version 2.0

from ros2cli.command import add_subparsers_on_demand
from ros2cli.command import CommandExtension


class BenchmarkCommand(CommandExtension):
    """Various hardware benchmark related sub-commands."""

    def add_arguments(self, parser, cli_name):
        self._subparser = parser
        # add arguments and sub-commands of verbs
        add_subparsers_on_demand(
            parser, cli_name, '_verb', 'ros2benchmark.verb', required=False)

    def main(self, *, parser, args):
        if not hasattr(args, '_verb'):
            # in case no verb was passed
            self._subparser.print_help()
            return 0

        extension = getattr(args, '_verb')
        # call the verb's main method
        return extension.main(args=args)
