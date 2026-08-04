#!/usr/bin/env python3
"""`ros2 top` must accept exactly what `ros2top` accepts, and share its code."""

import argparse
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from ros2top.main import add_arguments, create_argument_parser  # noqa: E402


def _options(parser):
    return {opt for action in parser._actions for opt in action.option_strings}


class TestRos2CliCommand(unittest.TestCase):

    def test_standalone_and_plugin_share_arguments(self):
        """Both entry points call the same add_arguments, so they cannot drift."""
        standalone = create_argument_parser()
        plugin_parser = argparse.ArgumentParser()
        add_arguments(plugin_parser)

        self.assertEqual(_options(standalone), _options(plugin_parser))
        self.assertIn('--refresh', _options(plugin_parser))
        self.assertIn('--no-auto-discovery', _options(plugin_parser))

    def test_defaults_match(self):
        plugin_parser = argparse.ArgumentParser()
        add_arguments(plugin_parser)
        a = plugin_parser.parse_args([])
        b = create_argument_parser().parse_args([])
        self.assertEqual(a.refresh, b.refresh)
        self.assertEqual(a.no_auto_discovery, b.no_auto_discovery)

    def test_entry_point_is_declared(self):
        """pyproject must advertise the command or ros2cli will never find it."""
        root = os.path.join(os.path.dirname(__file__), '..')
        with open(os.path.join(root, 'pyproject.toml')) as f:
            content = f.read()
        self.assertIn('[project.entry-points."ros2cli.command"]', content)
        self.assertIn('top = "ros2top.command.top:TopCommand"', content)

    @unittest.skipUnless(
        __import__('importlib').util.find_spec('ros2cli'),
        'ros2cli not installed (ROS 2 not sourced)')
    def test_command_extension_contract(self):
        """The plugin must satisfy the interface ros2cli calls into."""
        from ros2cli.command import CommandExtension
        from ros2top.command.top import TopCommand

        self.assertTrue(issubclass(TopCommand, CommandExtension))
        cmd = TopCommand()
        # ros2cli hands the verb a parser, then calls main(parser=, args=)
        parser = argparse.ArgumentParser()
        cmd.add_arguments(parser, 'top')
        self.assertIn('--refresh', _options(parser))
        self.assertTrue(TopCommand.__doc__.strip(),
                        'ros2 --help prints this docstring as the summary')


if __name__ == '__main__':
    unittest.main()
