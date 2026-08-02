#!/usr/bin/env python3
"""Nodes sharing one PID (composable nodes in a container) must be measured once."""

import os
import subprocess
import sys
import time

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


if __name__ == '__main__':
    test_shared_pid_measured_once()
