#!/usr/bin/env python3
"""
``ros2top --doctor``: explain why nodes are or are not showing up.

Every "ros2top lists nothing" report so far has come down to one of a handful of
environment problems -- a middleware that carries no PID, nodes in a different
PID namespace, a registry written under a different HOME, a stale lock file, or
ROS 2 not sourced -- and none of them are visible from the table. Rather than
ask each reporter for another round of `cat`, ros2top explains itself.

The report is plain text on stdout, so it can be pasted straight into an issue.
"""

import os
import platform
import sys
import time
from pathlib import Path
from typing import List, Optional

from . import __version__
from . import node_registry
from . import paths


def _ok(label: str, value: str = '') -> str:
    return f'  [ok]   {label}{": " + value if value else ""}'


def _warn(label: str, value: str = '') -> str:
    return f'  [warn] {label}{": " + value if value else ""}'


def _bad(label: str, value: str = '') -> str:
    return f'  [FAIL] {label}{": " + value if value else ""}'


def _info(label: str, value: str = '') -> str:
    return f'         {label}{": " + value if value else ""}'


def _in_container() -> bool:
    """Whether we are most likely running inside a container"""
    if Path('/.dockerenv').exists():
        return True
    try:
        with open('/proc/1/cgroup') as f:
            blob = f.read()
        return any(tag in blob for tag in ('docker', 'lxc', 'kubepods', 'containerd'))
    except OSError:
        return False


def _pid_alive(pid: int) -> bool:
    try:
        import psutil
        return psutil.pid_exists(pid)
    except Exception:
        try:
            os.kill(pid, 0)
            return True
        except OSError:
            return False


def _section(title: str) -> List[str]:
    return ['', title, '-' * len(title)]


def _environment() -> List[str]:
    out = _section('Environment')
    out.append(_info('ros2top', __version__))
    out.append(_info('python', f'{platform.python_version()} ({sys.executable})'))
    out.append(_info('platform', f'{platform.system()} {platform.release()} {platform.machine()}'))
    out.append(_info('installed at', str(Path(__file__).resolve().parent)))

    if platform.system() != 'Linux':
        out.append(_bad('ros2top needs Linux',
                        'CPU/RAM/PID data comes from /proc, which this system does not have'))

    if _in_container():
        out.append(_warn('running inside a container',
                         'nodes on the host are invisible unless the container '
                         'shares the host PID namespace (--pid=host) and the '
                         'registry directory is mounted'))

    for var in ('ROS_DISTRO', 'ROS_DOMAIN_ID', 'RMW_IMPLEMENTATION', 'ROS_LOCALHOST_ONLY'):
        out.append(_info(var, os.environ.get(var, '<unset>')))
    out.append(_info('HOME', os.environ.get('HOME', '<unset>')))
    return out


def _ros() -> List[str]:
    out = _section('ROS 2')
    try:
        import rclpy  # noqa: F401
    except ImportError as e:
        out.append(_bad('rclpy is not importable', str(e)))
        out.append(_info('consequence',
                         'the ROS graph cannot be read, so auto-discovery is '
                         'off. Source your ROS 2 installation, or have nodes '
                         'register themselves.'))
        return out

    out.append(_ok('rclpy imports'))
    try:
        from rclpy.utilities import get_rmw_implementation_identifier
        rmw = get_rmw_implementation_identifier() or 'unknown'
    except Exception as e:
        out.append(_warn('could not identify the middleware', str(e)))
        return out

    out.append(_info('middleware', rmw))
    if 'fastrtps' in rmw or 'fastdds' in rmw:
        out.append(_ok('middleware exposes node PIDs', 'auto-discovery available'))
    else:
        out.append(_warn(f'{rmw} does not expose node PIDs',
                         'auto-discovery is unavailable; nodes must call '
                         'register_node() to appear'))
    return out


def _discovery(timeout: float) -> List[str]:
    out = _section('Auto-discovery self-test')
    try:
        from .graph_discovery import GraphDiscovery
    except Exception as e:
        out.append(_bad('discovery module failed to load', str(e)))
        return out

    discovery = GraphDiscovery()
    try:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and not discovery.status.is_settled:
            time.sleep(0.1)

        status = discovery.status
        if not status.is_settled:
            out.append(_warn('discovery did not settle',
                             f'still starting after {timeout:.0f}s'))
        elif status.is_auto:
            out.append(_ok('auto-discovery works', status.reason))
        else:
            out.append(_warn('auto-discovery unavailable', status.reason))

        nodes = discovery.get_nodes_with_pids()
        out.append(_info('nodes found on the graph', str(len(nodes))))
        for name, pid in sorted(nodes):
            out.append(_info(f'  {name}', f'pid {pid}'))
    finally:
        discovery.shutdown()
    return out


def _registry() -> List[str]:
    out = _section('Registry')
    registry_dir = Path(node_registry.REGISTRY_DIR)
    nodes_file = Path(node_registry.REGISTRATION_FILE)
    lock_file = Path(node_registry.LOCK_FILE)

    out.append(_info('directory', str(registry_dir)))
    if not registry_dir.exists():
        out.append(_info('directory does not exist',
                         'nothing has registered yet; this is normal when every '
                         'node is found automatically'))
        return out

    out.append(_ok('directory exists') if os.access(registry_dir, os.W_OK)
               else _bad('directory is not writable',
                         'nodes cannot register. Check ownership -- a node that '
                         'ran as root will have created it root-owned.'))

    if lock_file.exists():
        owner = None
        try:
            owner = int(lock_file.read_text().split()[0])
        except (OSError, ValueError, IndexError):
            pass
        if owner is None:
            out.append(_warn('lock file present with unreadable owner', str(lock_file)))
        elif _pid_alive(owner):
            out.append(_info('lock held', f'pid {owner} (normal, transient)'))
        else:
            out.append(_warn(f'stale lock file owned by dead pid {owner}',
                             'registrations are blocked until it is removed. '
                             f'ros2top clears it automatically; to do it by '
                             f'hand: rm {lock_file}'))

    if not nodes_file.exists():
        out.append(_info('no nodes.json', 'nothing has registered yet'))
        return out

    entries = node_registry._read_node_registrations()
    if not entries:
        out.append(_warn('nodes.json is empty or unreadable', str(nodes_file)))
        return out

    out.append(_info('registered entries', str(len(entries))))
    shown = 0
    for key, data in sorted(entries.items()):
        name = data.get('node_name', '<no name>')
        pid = node_registry._entry_pid(key, data)
        if pid is None:
            out.append(_bad(f'{name}', f'entry has no usable pid (key {key!r})'))
            continue
        if _pid_alive(pid):
            shown += 1
            out.append(_ok(f'{name}', f'pid {pid} alive -> will be listed'))
        elif _in_container():
            out.append(_warn(f'{name}',
                             f'pid {pid} does not exist in this PID namespace. '
                             'The node is probably running outside this '
                             'container -- start it with --pid=host, or run '
                             'ros2top where the nodes are.'))
        else:
            out.append(_info(f'{name}',
                             f'pid {pid} is gone -> stale entry, will be cleaned up'))

    out.append(_info('entries that will appear in the table', str(shown)))
    return out


def _cpp() -> List[str]:
    out = _section('C++ integration')
    header = paths.include_dir()
    cmake = paths.cmake_dir()

    if header is None:
        out.append(_bad('ros2top.hpp not found',
                        'the C++ API is unavailable. Install the wheel '
                        '(pip install ros2top) rather than a bare source tree.'))
    else:
        out.append(_ok('header', str(header / 'ros2top' / 'ros2top.hpp')))

    if cmake is None:
        out.append(_bad('ros2topConfig.cmake not found',
                        'find_package(ros2top) cannot succeed'))
    else:
        out.append(_ok('cmake config', str(cmake)))
        in_source = cmake == paths.source_tree_cmake_dir()
        if in_source:
            out.append(_info('note',
                             'this is the source checkout, not an installed '
                             'copy -- an editable/source install does not '
                             'install data files. It works, but the path is '
                             'tied to this clone.'))
        out.append(_info('to build a package that uses it',
                         f'colcon build --cmake-args -Dros2top_DIR={cmake}'))
    return out


def _gpu() -> List[str]:
    out = _section('GPU')
    try:
        from .gpu_monitor import GPUMonitor, NVML_AVAILABLE
    except Exception as e:
        out.append(_warn('GPU module failed to load', str(e)))
        return out

    if not NVML_AVAILABLE:
        out.append(_info('pynvml not installed', 'GPU columns will read --'))
        return out

    monitor = GPUMonitor()
    try:
        if monitor.is_available():
            out.append(_ok('NVML', f'{monitor.get_gpu_count()} device(s)'))
        else:
            out.append(_info('NVML present but no usable device',
                             'normal on machines without an NVIDIA GPU, and on '
                             'Jetson, where NVML does not support the '
                             'integrated GPU. GPU columns will read --'))
    finally:
        monitor.shutdown()
    return out


def run(discovery_timeout: float = 8.0, skip_discovery: bool = False) -> int:
    """Print the diagnostic report. Returns a process exit code."""
    lines = ['ros2top doctor', '==============']
    lines += _environment()
    lines += _ros()
    if not skip_discovery:
        lines += _discovery(discovery_timeout)
    lines += _registry()
    lines += _cpp()
    lines += _gpu()
    lines += ['',
              'Paste this report into a bug report at',
              '  https://github.com/AhmedARadwan/ros2top/issues',
              '']

    print('\n'.join(lines))
    return 0
