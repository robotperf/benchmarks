# Copyright (C) Acceleration Robotics S.L.U. - All Rights Reserved
#
# Written by Víctor Mayoral-Vilches <victor@accelerationrobotics.com>
# Licensed under the Apache License, Version 2.0
#

from argparse import ArgumentTypeError
from time import sleep
from typing import Optional
from ament_index_python import (
        get_packages_with_prefixes, 
        get_package_prefix,
        PackageNotFoundError,
)


def get_package_names():
    return get_packages_with_prefixes().keys()


def get_prefix_path(package_name):
    try:
        prefix_path = get_package_prefix(package_name)
    except (PackageNotFoundError, ValueError):
        return None
    return prefix_path
