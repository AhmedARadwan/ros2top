#!/usr/bin/env python3
"""
ros2cli plugin exposing ros2top as `ros2 top`.

Discovered through the `ros2cli.command` entry point declared in
pyproject.toml, so installing ros2top alongside ROS 2 is all that is needed --
no changes to ros2cli itself.
"""

from ros2cli.command import CommandExtension

from ..main import DESCRIPTION, add_arguments, run


class TopCommand(CommandExtension):
    """Real-time monitor for ROS 2 nodes showing CPU, RAM, and GPU usage."""

    # ros2cli prints the first line of this docstring in `ros2 --help`, so it
    # doubles as the command's one-line summary.
    __doc__ = DESCRIPTION + '.'

    def add_arguments(self, parser, cli_name):
        add_arguments(parser)

    def main(self, *, parser, args):
        return run(args)
