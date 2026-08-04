#!/usr/bin/env python3
"""Graph auto-discovery: GUID parsing, PID matching, and safe degradation.

The parsing and matching logic is deliberately pure, so none of this needs a
running ROS 2 system.
"""

import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from ros2top.graph_discovery import (  # noqa: E402
    DiscoveryMode, GraphDiscovery, carries_own_pid, host_id, pid_low16, resolve,
)


def _prefix(host=b'\xc9\x6e', pid=0, tail=b'\xab\xec\x00\x00\x00\x00'):
    """Build a Fast DDS style 12-byte GUID prefix"""
    return b'\x01\x0f' + host + (pid & 0xFFFF).to_bytes(2, 'little') + tail


def test_guid_layout_matches_observed_bytes():
    """Real capture: PID 3666305 produced prefix 01 0f c9 6e 81 f1 ab ec ..."""
    observed = bytes.fromhex('010fc96e81f1abec00000000')
    assert pid_low16(observed) == 3666305 & 0xFFFF
    assert host_id(observed) == b'\xc9\x6e'
    # only the low 16 bits are present, never the whole PID
    assert pid_low16(observed) != 3666305
    print('OK: GUID layout matches the captured bytes')


def test_self_test_detects_unsupported_middleware():
    assert carries_own_pid(_prefix(pid=os.getpid()), os.getpid())
    # Zenoh/Cyclone put something else there entirely
    assert not carries_own_pid(_prefix(pid=os.getpid() + 1), os.getpid())
    print('OK: self-test distinguishes PID-bearing GUIDs')


def test_resolves_composable_nodes_to_one_pid():
    host = b'\xc9\x6e'
    container = _prefix(host, 4242)
    standalone = _prefix(host, 4243)
    nodes = {
        container: {'/my_container', '/talker', '/listener'},
        standalone: {'/talker'},   # same name, different process
    }
    got = resolve(nodes, [4242, 4243], host)

    assert sorted(got) == sorted([
        ('/my_container', 4242), ('/talker', 4242), ('/listener', 4242),
        ('/talker', 4243),
    ]), got
    print('OK: composable nodes resolve to their container, duplicates kept apart')


def test_remote_host_is_rejected():
    """A node on another machine must never be matched to a local PID."""
    ours, theirs = b'\xc9\x6e', b'\xde\xad'
    nodes = {_prefix(theirs, 4242): {'/remote_node'}}
    assert resolve(nodes, [4242], ours) == [], 'matched a node from another host'
    # ... and the identical prefix on our own host does resolve
    assert resolve({_prefix(ours, 4242): {'/local'}}, [4242], ours) == [('/local', 4242)]
    print('OK: remote nodes rejected by host id')


def test_ambiguous_low16_reports_nothing():
    """16 bits collide; with two candidates there is no way to pick, so don't."""
    host = b'\xc9\x6e'
    colliding = [4242, 4242 + 0x10000]
    assert (4242 & 0xFFFF) == (colliding[1] & 0xFFFF)
    nodes = {_prefix(host, 4242): {'/ambiguous'}}
    assert resolve(nodes, colliding, host) == [], 'guessed between two candidates'
    print('OK: ambiguous PID low16 yields nothing rather than a guess')


def test_unknown_pid_reports_nothing():
    host = b'\xc9\x6e'
    nodes = {_prefix(host, 9999): {'/vanished'}}
    assert resolve(nodes, [4242], host) == []
    print('OK: unmatched node omitted')


def test_degrades_without_rclpy(monkeypatch=None):
    """No ROS sourced must mean MANUAL mode, not a crash."""
    import builtins
    real_import = builtins.__import__

    def no_rclpy(name, *a, **kw):
        if name == 'rclpy' or name.startswith('rclpy.'):
            raise ImportError('No module named rclpy')
        return real_import(name, *a, **kw)

    builtins.__import__ = no_rclpy
    try:
        d = GraphDiscovery()
        # the import happens on the worker, so give it a moment to report back
        for _ in range(50):
            if 'rclpy' in d.status.reason:
                break
            time.sleep(0.05)
        status = d.status
        assert status.mode is DiscoveryMode.MANUAL, status
        assert 'rclpy' in status.reason
        assert d.get_nodes_with_pids() == []
        d.shutdown()
    finally:
        builtins.__import__ = real_import
    print('OK: missing rclpy degrades to manual mode')


def test_short_rmw_names():
    from ros2top.graph_discovery import DiscoveryStatus
    s = DiscoveryStatus(DiscoveryMode.AUTO, 'rmw_fastrtps_cpp', '')
    assert s.short_rmw == 'fastrtps'
    assert DiscoveryStatus(DiscoveryMode.MANUAL, 'rmw_zenoh_cpp', '').short_rmw == 'zenoh'
    assert DiscoveryStatus(DiscoveryMode.MANUAL, 'unknown', '').short_rmw == 'unknown'
    print('OK: RMW names shortened for the status bar')


def test_starting_state_makes_no_claim():
    """Before the graph is reachable ros2top must not advise registering nodes."""
    from ros2top.graph_discovery import DiscoveryStatus

    starting = DiscoveryStatus(DiscoveryMode.STARTING, 'unknown', 'Looking for nodes...')
    assert not starting.is_settled, 'startup reported as a verdict'
    assert not starting.is_auto

    for mode in (DiscoveryMode.AUTO, DiscoveryMode.MANUAL):
        assert DiscoveryStatus(mode, 'rmw_x', '').is_settled, mode
    print('OK: startup state is distinct from a verdict')


def test_placeholder_and_internal_nodes_are_not_reported():
    """rmw emits sentinels for endpoints it has not matched yet; they are not nodes."""
    from ros2top.graph_discovery import _is_reportable

    assert not _is_reportable('_NODE_NAME_UNKNOWN_', '/'), 'phantom node reported'
    assert not _is_reportable('talker', '_NODE_NAMESPACE_UNKNOWN_')
    assert not _is_reportable('_ros2cli_daemon_62_abc', '/')
    assert not _is_reportable('ros2top_discovery_1234', '/'), 'another ros2top reported'
    # real nodes still pass
    assert _is_reportable('talker', '/')
    assert _is_reportable('my_container', '/robot')
    print('OK: placeholders, ros2cli daemon and other ros2top instances filtered')


def test_registry_wins_over_graph():
    """A node that registered must not also appear as an auto-discovered row."""
    from ros2top.node_monitor import NodeMonitor

    monitor = NodeMonitor(refresh_interval=0.0, auto_discovery=False)
    pid = os.getpid()
    # registry reports it, and the graph would report the very same (name, pid)
    monitor.discovery = _FakeDiscovery([('/dup', pid), ('/only_graph', pid)])
    import ros2top.node_monitor as nm
    real = nm.get_registered_nodes
    nm.get_registered_nodes = lambda: [('/dup', pid)]
    try:
        found = monitor._get_all_processes_to_monitor()
    finally:
        nm.get_registered_nodes = real

    assert found.count(('/dup', pid)) == 1, f'duplicated: {found}'
    assert ('/only_graph', pid) in found
    # only the graph-only node is marked
    assert monitor._auto_names == {('/only_graph', pid)}, monitor._auto_names
    print('OK: registry entry wins, graph-only node marked')


def test_container_heads_group_without_registration_times():
    """Auto-discovered nodes share a create_time, so cmdline breaks the tie."""
    from ros2top.node_monitor import NodeMonitor

    monitor = NodeMonitor(refresh_interval=0.0, auto_discovery=False)
    pid = os.getpid()
    # every node in a container reports the same process create time
    monitor._get_process_start_time = lambda name, proc: 1000.0
    monitor._cmdlines[pid] = 'component_container --ros-args -r __node:=my_container'
    monitor._add_new_nodes_with_pids([
        ('/listener', pid), ('/my_container', pid), ('/talker', pid)])

    names = [i.name for i in monitor.get_node_info_list()]
    assert names[0] == '/my_container', f'container not at head: {names}'
    print('OK: cmdline names the container when start times tie')


class _FakeDiscovery:
    def __init__(self, nodes):
        self._nodes = nodes

    def get_nodes_with_pids(self):
        return list(self._nodes)

    @property
    def status(self):
        from ros2top.graph_discovery import DiscoveryStatus
        return DiscoveryStatus(DiscoveryMode.AUTO, 'rmw_fake', '')

    def shutdown(self):
        pass


if __name__ == '__main__':
    test_guid_layout_matches_observed_bytes()
    test_self_test_detects_unsupported_middleware()
    test_resolves_composable_nodes_to_one_pid()
    test_remote_host_is_rejected()
    test_ambiguous_low16_reports_nothing()
    test_unknown_pid_reports_nothing()
    test_degrades_without_rclpy()
    test_short_rmw_names()
    test_starting_state_makes_no_claim()
    test_placeholder_and_internal_nodes_are_not_reported()
    test_registry_wins_over_graph()
    test_container_heads_group_without_registration_times()
