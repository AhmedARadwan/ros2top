#!/usr/bin/env python3
"""Nodes sharing one PID (composable nodes in a container) must be measured once."""

import os
import subprocess
import sys
import time

import psutil

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from ros2top.node_monitor import NodeMonitor  # noqa: E402


def _busy_process():
    """A real child process that actually burns CPU, so cpu_percent() is non-zero."""
    return subprocess.Popen(
        [sys.executable, '-c', 'while True: pass'],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def test_shared_pid_measured_once():
    container = _busy_process()
    solo = _busy_process()
    try:
        monitor = NodeMonitor(refresh_interval=0.0)
        # Three composable nodes in one container + one standalone node.
        monitor._add_new_nodes_with_pids([
            ('/talker', container.pid),
            ('/listener', container.pid),
            ('/my_container', container.pid),
            ('/standalone', solo.pid),
        ])

        time.sleep(1.0)  # let cpu_percent() accumulate a real interval
        infos = monitor.get_node_info_list()
        by_name = {i.name: i for i in infos}

        assert len(infos) == 4, f'expected one row per node, got {len(infos)}'

        # every node still gets its own row
        assert set(by_name) == {'/talker', '/listener', '/my_container', '/standalone'}

        # the three container nodes are flagged as sharing a process
        for name in ('/talker', '/listener', '/my_container'):
            assert by_name[name].shared_count == 3, f'{name} shared_count'
            assert by_name[name].pid == container.pid
        assert by_name['/standalone'].shared_count == 1

        # sampled once => the shared rows are byte-identical, not three
        # independent (and differing) samples of the same process
        shared = [by_name[n] for n in ('/talker', '/listener', '/my_container')]
        assert len({i.cpu_percent for i in shared}) == 1, 'cpu differs across shared rows'
        assert len({i.ram_mb for i in shared}) == 1, 'ram differs across shared rows'
        assert shared[0].cpu_percent > 0, 'busy process should report cpu'

        # one sampler per PID, not one per node
        assert set(monitor._samplers) == {container.pid, solo.pid}, monitor._samplers

        # totals count each process once instead of inflating by 3x
        cpu, ram, _ = monitor.get_process_totals(infos)
        naive_cpu = sum(i.cpu_percent for i in infos)
        expected = by_name['/talker'].cpu_percent + by_name['/standalone'].cpu_percent
        assert abs(cpu - expected) < 1e-9, f'total cpu {cpu} != {expected}'
        assert naive_cpu > cpu, 'test is not exercising the inflation it guards'

        naive_ram = sum(i.ram_mb for i in infos)
        assert abs(ram - (by_name['/talker'].ram_mb
                          + by_name['/standalone'].ram_mb)) < 1e-9
        assert naive_ram > ram

        # dead processes must not leak samplers
        container.kill()
        container.wait()
        monitor.processes = {k: v for k, v in monitor.processes.items()
                             if not k.endswith(f':{container.pid}')}
        monitor.get_node_info_list()
        assert set(monitor._samplers) == {solo.pid}, monitor._samplers

        print('OK: shared-PID nodes sampled once, totals not inflated')
    finally:
        for p in (container, solo):
            if p.poll() is None:
                p.kill()
            p.wait()


def test_display_order_matches_kill_order():
    """The UI kills by row index, so table order must equal get_node_info_list()."""
    from unittest.mock import MagicMock
    from ros2top.ui.terminal_ui import TerminalUI

    container = _busy_process()
    solo = _busy_process()
    try:
        monitor = NodeMonitor(refresh_interval=0.0)
        monitor._add_new_nodes_with_pids([
            ('/zzz_composed_later', container.pid),
            ('/aaa_other_process', solo.pid),
            ('/mmm_container', container.pid),
        ])

        # The container is the eldest node in its process; the composed node is
        # loaded later. Names are chosen so alphabetical order would disagree.
        now = time.time()
        monitor._get_process_start_time = lambda name, proc: {
            '/mmm_container': now - 100,
            '/zzz_composed_later': now - 10,
            '/aaa_other_process': now - 50,
        }[name]

        infos = monitor.get_node_info_list()

        # grouped by pid, so same-process nodes are adjacent
        pids = [i.pid for i in infos]
        assert pids == sorted(pids), f'not grouped by pid: {pids}'

        # the container heads its group despite sorting last alphabetically
        group = [i.name for i in infos if i.pid == container.pid]
        assert group[0] == '/mmm_container', f'container should head group: {group}'

        ui = TerminalUI(monitor)
        captured = {}
        ui.nodes_table = MagicMock()
        ui.nodes_table.set_data.side_effect = lambda r: captured.setdefault('rows', r)
        ui.table_section = {'width': 100}
        ui._update_nodes_table()

        # last column is the node name, plus connector and/or count suffix
        names = [r[-1].split('(+')[0].strip().lstrip('- ') for r in captured['rows']]
        assert names == [i.name for i in infos], (
            f'display order {names} != kill order {[i.name for i in infos]}')

        # the container row announces the group; children hang off it
        header = [r[-1] for r in captured['rows'] if '/mmm_container' in r[-1]][0]
        assert header.startswith('/mmm_container') and '(+1 nodes)' in header, header
        child = [r[-1] for r in captured['rows'] if '/zzz_composed_later' in r[-1]][0]
        assert child.startswith('  - '), child
        print('OK: container heads group, display order matches kill order')
    finally:
        for p in (container, solo):
            if p.poll() is None:
                p.kill()
            p.wait()


def test_kill_removes_every_node_in_the_process():
    """Killing one composed node kills the container, so all its nodes must go."""
    container = _busy_process()
    solo = _busy_process()
    try:
        monitor = NodeMonitor(refresh_interval=0.0)
        monitor._add_new_nodes_with_pids([
            ('/my_container', container.pid),
            ('/talker', container.pid),
            ('/listener', container.pid),
            ('/talker', solo.pid),      # same name, different process
        ])
        monitor.get_node_info_list()  # populate samplers
        assert len(monitor.processes) == 4
        assert set(monitor._samplers) == {container.pid, solo.pid}

        # naming a PID must pick that process, not the first same-named node
        assert monitor.kill_process('/talker', pid=solo.pid, force=True)
        assert container.poll() is None, 'killed the wrong process'
        assert set(monitor._samplers) == {container.pid}, monitor._samplers
        assert [monitor._node_name_of(k) for k in monitor.processes] == [
            '/my_container', '/talker', '/listener']

        # killing any node of the container takes all three with it
        assert monitor.kill_process('/listener', pid=container.pid, force=True)
        assert monitor.processes == {}, monitor.processes
        assert monitor._samplers == {}, monitor._samplers
        print('OK: kill removes every node of the process it ends')
    finally:
        for p in (container, solo):
            if p.poll() is None:
                p.kill()
            p.wait()


def test_kill_tolerates_key_without_pid_suffix():
    """Keys are 'name:pid', but the API must not break if one lacks the suffix."""
    proc = _busy_process()
    try:
        monitor = NodeMonitor(refresh_interval=0.0)
        monitor.processes['/legacy_node'] = psutil.Process(proc.pid)
        assert monitor.kill_process('/legacy_node', force=True)
        assert monitor.processes == {}
        print('OK: kill handles a key with no :pid suffix')
    finally:
        if proc.poll() is None:
            proc.kill()
        proc.wait()


if __name__ == '__main__':
    test_shared_pid_measured_once()
    test_display_order_matches_kill_order()
    test_kill_removes_every_node_in_the_process()
    test_kill_tolerates_key_without_pid_suffix()
