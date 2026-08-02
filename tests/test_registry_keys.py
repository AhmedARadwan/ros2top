#!/usr/bin/env python3
"""The registry must hold one entry per node, even when nodes share a PID."""

import json
import os
import sys
import tempfile

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import ros2top.node_registry as reg  # noqa: E402


def _isolate():
    d = tempfile.mkdtemp()
    reg.REGISTRY_DIR = d
    reg.REGISTRATION_FILE = os.path.join(d, 'nodes.json')
    reg.LOCK_FILE = os.path.join(d, 'nodes.lock')
    reg._registered_nodes.clear()


def test_composable_nodes_all_survive():
    """Three nodes registering from one process must not overwrite each other."""
    _isolate()
    for name in ('/talker', '/listener', '/my_container'):
        assert reg.register_node(name), f'register {name}'

    nodes = reg.get_registered_nodes()
    assert len(nodes) == 3, f'expected 3 entries, got {nodes}'
    assert {n for n, _ in nodes} == {'/talker', '/listener', '/my_container'}
    assert {p for _, p in nodes} == {os.getpid()}, 'all three share this PID'
    print('OK: 3 nodes on 1 PID all kept')


def test_cpp_written_entries_are_visible():
    """C++ keys by node name; entries must still be read, via their pid field."""
    _isolate()
    pid = os.getpid()
    json.dump({
        f'{pid}:/new_style': {'node_name': '/new_style', 'pid': pid},
        '/cpp_legacy_key': {'node_name': '/cpp_legacy_key', 'pid': pid},
        str(pid): {'node_name': '/py_legacy_key', 'pid': pid},
    }, open(reg.REGISTRATION_FILE, 'w'))

    names = {n for n, _ in reg.get_registered_nodes()}
    assert names == {'/new_style', '/cpp_legacy_key', '/py_legacy_key'}, names
    print('OK: new, C++-legacy and Python-legacy keys all visible')


def test_dead_pid_removes_all_its_nodes():
    _isolate()
    dead = 999999  # not a running pid
    json.dump({
        f'{dead}:/a': {'node_name': '/a', 'pid': dead},
        f'{dead}:/b': {'node_name': '/b', 'pid': dead},
        f'{os.getpid()}:/alive': {'node_name': '/alive', 'pid': os.getpid()},
    }, open(reg.REGISTRATION_FILE, 'w'))

    assert not any(p == dead for _, p in reg.get_registered_nodes())
    reg.cleanup_stale_registrations()
    left = json.load(open(reg.REGISTRATION_FILE))
    assert set(left) == {f'{os.getpid()}:/alive'}, left
    print('OK: dead PID took both of its nodes with it')


def test_unregister_and_heartbeat_by_name():
    _isolate()
    for name in ('/talker', '/listener'):
        reg.register_node(name)

    assert reg.heartbeat('/talker')
    reg._remove_node_registration('/talker')
    names = {n for n, _ in reg.get_registered_nodes()}
    assert names == {'/listener'}, f'unregister hit the wrong nodes: {names}'
    print('OK: unregister/heartbeat target one node, not the whole process')


if __name__ == '__main__':
    test_composable_nodes_all_survive()
    test_cpp_written_entries_are_visible()
    test_dead_pid_removes_all_its_nodes()
    test_unregister_and_heartbeat_by_name()
