#!/usr/bin/env python3
"""
Tests for the C++ integration path lookup, the diagnostic report, and registry
lock recovery.

These cover the three things that made the C++ side unusable: nobody could find
ros2topConfig.cmake, nothing explained why a node was missing, and a lock file
left behind by a killed process blocked every later registration.
"""

import os
import unittest
from pathlib import Path
from unittest.mock import patch

from ros2top import doctor, node_registry, paths


class TestPaths(unittest.TestCase):
    """Locating the header and the CMake config"""

    def test_cmake_dir_found(self):
        found = paths.cmake_dir()
        self.assertIsNotNone(found, 'ros2topConfig.cmake should be findable '
                                    'from a source checkout or an install')
        self.assertTrue((found / 'ros2topConfig.cmake').is_file())

    def test_include_dir_holds_the_header(self):
        found = paths.include_dir()
        self.assertIsNotNone(found)
        self.assertTrue((found / 'ros2top' / 'ros2top.hpp').is_file())

    def test_source_tree_fallback(self):
        """An editable install copies no data files, so the checkout is used"""
        found = paths.source_tree_cmake_dir()
        if found is not None:            # None when running from a wheel
            self.assertTrue((found / 'ros2topConfig.cmake').is_file())
            self.assertTrue((found.parent / 'include' / 'ros2top' / 'ros2top.hpp').is_file())

    def test_paths_are_absolute(self):
        """They are pasted into shell commands, so they must not be relative"""
        for path in (paths.cmake_dir(), paths.include_dir()):
            if path is not None:
                self.assertTrue(path.is_absolute(), f'{path} is not absolute')


class TestRegistryLocking(unittest.TestCase):
    """A lock must not outlive the process holding it"""

    def setUp(self):
        self.tmp = Path(os.environ.get('TMPDIR', '/tmp')) / f'ros2top-test-{os.getpid()}'
        self.tmp.mkdir(parents=True, exist_ok=True)
        self.lock = self.tmp / 'nodes.lock'
        self._patch = patch.object(node_registry, 'LOCK_FILE', str(self.lock))
        self._patch.start()

    def tearDown(self):
        self._patch.stop()
        if self.lock.exists():
            self.lock.unlink()
        self.tmp.rmdir()

    def test_acquires_when_free(self):
        self.assertTrue(node_registry._acquire_lock())
        self.assertTrue(self.lock.exists())

    def test_dead_owner_lock_is_stolen(self):
        # A PID that cannot be running. 0 is never a user process, and
        # pid_exists(0) is False on Linux.
        self.lock.write_text('999999999')
        self.assertTrue(node_registry._steal_stale_lock(),
                        'a lock owned by a dead PID should be removed')
        self.assertTrue(node_registry._acquire_lock(),
                        'and the lock should then be acquirable')

    def test_live_owner_lock_is_respected(self):
        self.lock.write_text(str(os.getpid()))   # we are alive by definition
        self.assertFalse(node_registry._steal_stale_lock())
        self.assertFalse(node_registry._acquire_lock(timeout=0.05),
                         'a lock held by a live process must not be stolen')

    def test_unreadable_lock_is_not_stolen(self):
        """Empty means mid-write by a live process; waiting is correct"""
        self.lock.write_text('')
        self.assertFalse(node_registry._steal_stale_lock())

    def test_acquire_times_out_rather_than_hanging(self):
        self.lock.write_text(str(os.getpid()))
        self.assertFalse(node_registry._acquire_lock(timeout=0.05))


class TestDoctor(unittest.TestCase):
    """The report has to be printable on any machine, ROS or not"""

    def test_runs_and_reports_every_section(self):
        # skip_discovery: a unit test must not stand up an rclpy node and join
        # the ROS graph.
        with patch('builtins.print') as printed:
            code = doctor.run(skip_discovery=True)

        self.assertEqual(code, 0)
        report = '\n'.join(str(call.args[0]) for call in printed.call_args_list)
        for section in ('Environment', 'ROS 2', 'Registry', 'C++ integration', 'GPU'):
            self.assertIn(section, report)

    def test_reports_the_cmake_command_to_run(self):
        with patch('builtins.print') as printed:
            doctor.run(skip_discovery=True)
        report = '\n'.join(str(call.args[0]) for call in printed.call_args_list)
        self.assertIn('-Dros2top_DIR=', report,
                      'the report should hand the user the exact flag they need')

    def test_survives_a_missing_registry(self):
        with patch.object(node_registry, 'REGISTRY_DIR', '/nonexistent/ros2top'):
            with patch('builtins.print'):
                self.assertEqual(doctor.run(skip_discovery=True), 0)


if __name__ == '__main__':
    unittest.main()
