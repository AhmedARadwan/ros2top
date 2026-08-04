#!/usr/bin/env python3
"""
Registry-free node discovery from the ROS 2 graph.

Nodes that never call ros2top.register_node() are invisible to the registry.
On some middleware they can still be found: the DDS GUID that every endpoint
carries encodes the PID of the process that owns it, so the graph alone is
enough to map a node name to a process.

This is a property of the *middleware*, not of ROS 2. Fast DDS lays its GUID
prefix out as:

    [0:2]  vendor id (01 0f = eProsima)
    [2:4]  host id
    [4:6]  low 16 bits of the PID, little-endian
    [6:8]  random
    [8:12] participant counter

Zenoh and Cyclone use different schemes and carry no PID, so discovery is
unavailable there and nodes must register themselves. Rather than keep a list
of which vendors work -- which would rot the moment any of them changes its
layout -- we simply ask the graph where *our own* node lives and check whether
the answer is our PID. That proves the capability instead of assuming it.
"""

import atexit
import os
import threading
import time
from collections import defaultdict
from enum import Enum
from typing import Dict, Iterable, List, NamedTuple, Optional, Sequence, Set, Tuple

# Our own graph node is named with this prefix, so instances of ros2top can
# recognise and skip each other rather than reporting themselves as user nodes.
_NODE_PREFIX = 'ros2top_discovery_'

# rmw_dds_common reports these placeholders for an endpoint whose owning node it
# has not matched yet. They are not node names: taken literally they show up as
# a phantom "_NODE_NAME_UNKNOWN_" row for a second until discovery settles.
_UNKNOWN_NAME = '_NODE_NAME_UNKNOWN_'
_UNKNOWN_NAMESPACE = '_NODE_NAMESPACE_UNKNOWN_'

# Byte ranges within the 12-byte GUID prefix
_HOST = slice(2, 4)
_PID = slice(4, 6)


class DiscoveryMode(Enum):
    STARTING = 'starting'  # still joining the graph; no verdict yet
    AUTO = 'auto'       # the graph gives us node -> PID, no registration needed
    MANUAL = 'manual'   # nodes must register themselves to be seen


class DiscoveryStatus(NamedTuple):
    mode: DiscoveryMode
    rmw: str        # middleware identifier, e.g. 'rmw_fastrtps_cpp'
    reason: str     # human-readable, shown in the UI when mode is MANUAL

    @property
    def is_auto(self) -> bool:
        return self.mode is DiscoveryMode.AUTO

    @property
    def is_settled(self) -> bool:
        """Whether discovery has decided. False only during the first moments."""
        return self.mode is not DiscoveryMode.STARTING

    @property
    def short_rmw(self) -> str:
        """'rmw_fastrtps_cpp' -> 'fastrtps', for the status bar"""
        name = self.rmw
        if name.startswith('rmw_'):
            name = name[4:]
        if name.endswith('_cpp'):
            name = name[:-4]
        return name or 'unknown'


# --------------------------------------------------------------------------
# Pure helpers. No ROS, no /proc, so they can be unit tested anywhere.
# --------------------------------------------------------------------------

def pid_low16(prefix: Sequence[int]) -> int:
    """The 16 PID bits a Fast DDS GUID prefix carries"""
    return int.from_bytes(bytes(prefix)[_PID], 'little')


def host_id(prefix: Sequence[int]) -> bytes:
    """The host portion of a GUID prefix"""
    return bytes(prefix)[_HOST]


def carries_own_pid(prefix: Sequence[int], pid: Optional[int] = None) -> bool:
    """Self-test: does this prefix encode the PID we are actually running as?"""
    if pid is None:
        pid = os.getpid()
    return pid_low16(prefix) == pid & 0xFFFF


def resolve(nodes_by_prefix: Dict[bytes, Set[str]],
            local_pids: Iterable[int],
            own_host: bytes) -> List[Tuple[str, int]]:
    """
    Map node names to PIDs.

    Args:
        nodes_by_prefix: GUID prefix -> node names sharing it. One prefix is one
            participant, i.e. one process, so a component container and every
            node composed into it appear under a single key.
        local_pids: PIDs of ROS processes on this machine.
        own_host: host id of the machine we are running on.

    Returns:
        (node_name, pid) pairs. Nodes that cannot be resolved *unambiguously*
        are omitted rather than guessed at.
    """
    by_low16 = defaultdict(list)
    for pid in local_pids:
        by_low16[pid & 0xFFFF].append(pid)

    resolved = []
    for prefix, names in nodes_by_prefix.items():
        # A node on another machine would have a meaningless PID here: its low
        # 16 bits could collide with a local process and we would happily show
        # somebody else's CPU usage against it.
        if host_id(prefix) != own_host:
            continue

        # Only 16 bits of PID are available, so collisions are real. Narrowing
        # to ROS processes makes them rare, but when one does happen there is
        # no way to tell the candidates apart -- so report nothing.
        candidates = by_low16.get(pid_low16(prefix), ())
        if len(candidates) != 1:
            continue

        for name in names:
            resolved.append((name, candidates[0]))

    return resolved


# --------------------------------------------------------------------------
# Local process enumeration
# --------------------------------------------------------------------------

def is_ros_process(pid: int) -> bool:
    """Whether a local process has the ROS client library mapped in"""
    try:
        with open(f'/proc/{pid}/maps') as f:
            return 'librcl.so' in f.read()
    except PermissionError:
        # /proc/<pid>/maps is ptrace-protected, so a node running as another
        # user (root is common on robots) is unreadable. cmdline is world
        # readable, so fall back to it rather than dropping the process.
        try:
            with open(f'/proc/{pid}/cmdline', 'rb') as f:
                return b'/opt/ros/' in f.read()
        except OSError:
            return False
    except OSError:
        return False


def _is_reportable(node_name: str, node_namespace: str) -> bool:
    """Whether a graph endpoint belongs to a node worth showing the user"""
    if node_name == _UNKNOWN_NAME or node_namespace == _UNKNOWN_NAMESPACE:
        return False        # discovery has not matched this endpoint yet
    if node_name.startswith('_ros2cli_daemon_'):
        return False        # ros2cli's own helper, not a user's node
    if node_name.startswith(_NODE_PREFIX):
        return False        # another ros2top's discovery node
    return True


def local_ros_pids() -> Set[int]:
    """PIDs of every ROS process on this machine"""
    pids = set()
    for entry in os.listdir('/proc'):
        if entry.isdigit() and is_ros_process(int(entry)):
            pids.add(int(entry))
    return pids


# --------------------------------------------------------------------------
# The graph client
# --------------------------------------------------------------------------

class GraphDiscovery:
    """
    Watches the ROS graph on a background thread and caches node -> PID.

    A single node is created for the lifetime of the process. NodeMonitor
    refreshes as often as every 0.1s, which is far too fast to stand up an
    rclpy node each time, so callers read a cached snapshot instead.
    """

    def __init__(self, period: float = 1.0, autostart: bool = True):
        self._period = period
        self._lock = threading.Lock()
        self._nodes: List[Tuple[str, int]] = []
        # Shown for the second or so before the graph is reachable, so it has
        # to read as progress rather than as a verdict.
        self._status = DiscoveryStatus(
            DiscoveryMode.STARTING, 'unknown', 'Looking for nodes...')
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._context = None
        self._node = None
        self._own_host: Optional[bytes] = None

        if autostart:
            self.start()

    # -- public API --------------------------------------------------------

    @property
    def status(self) -> DiscoveryStatus:
        with self._lock:
            return self._status

    def get_nodes_with_pids(self) -> List[Tuple[str, int]]:
        """Latest snapshot. Empty unless discovery is working."""
        with self._lock:
            return list(self._nodes)

    def start(self):
        # Importing rclpy costs over a second, so it happens on the worker.
        # Doing it here would add that to every ros2top startup, including
        # the runs where ROS is not even installed.
        if self._thread is not None:
            return
        self._thread = threading.Thread(target=self._run, daemon=True,
                                        name='ros2top-discovery')
        self._thread.start()
        # A daemon thread killed mid-spin leaves rclpy's C++ side to abort with
        # "terminate called without an active exception" on the way out. Stop
        # it properly even when the caller forgets to.
        atexit.register(self.shutdown)

    def shutdown(self):
        """Stop the discovery thread. Safe to call more than once."""
        self._stop.set()
        thread, self._thread = self._thread, None
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=2.0)

    # -- internals ---------------------------------------------------------

    def _set_status(self, mode: DiscoveryMode, rmw: str, reason: str):
        with self._lock:
            self._status = DiscoveryStatus(mode, rmw, reason)

    def _run(self):
        try:
            import rclpy
            from rclpy.executors import SingleThreadedExecutor
        except ImportError:
            self._set_status(DiscoveryMode.MANUAL, 'unknown',
                             'rclpy is not importable, so the ROS graph cannot '
                             'be read. Source your ROS 2 installation, or '
                             'register nodes with ros2top explicitly.')
            return

        rmw = 'unknown'
        try:
            from rclpy.utilities import get_rmw_implementation_identifier
            # Own context, so ros2top never disturbs a global rclpy.init() if
            # it is ever imported from inside a running node.
            self._context = rclpy.Context()
            rclpy.init(context=self._context)
            rmw = get_rmw_implementation_identifier() or 'unknown'
            self._node = rclpy.create_node(
                f'{_NODE_PREFIX}{os.getpid()}', context=self._context)
            executor = SingleThreadedExecutor(context=self._context)
            executor.add_node(self._node)
        except Exception as e:
            self._set_status(DiscoveryMode.MANUAL, rmw,
                             f'could not join the ROS graph: {e}')
            self._cleanup_ros()
            return

        try:
            next_snapshot = 0.0
            while not self._stop.is_set():
                executor.spin_once(timeout_sec=0.1)
                now = time.monotonic()
                if now >= next_snapshot:
                    try:
                        self._snapshot(rmw)
                    except Exception:
                        # A transient graph error must not kill the thread
                        pass
                    next_snapshot = now + self._period
        finally:
            self._cleanup_ros()

    def _cleanup_ros(self):
        try:
            if self._node is not None:
                self._node.destroy_node()
        except Exception:
            pass
        try:
            if self._context is not None:
                import rclpy
                rclpy.shutdown(context=self._context)
        except Exception:
            pass
        self._node = None
        self._context = None

    def _collect_endpoints(self):
        """-> (nodes_by_prefix, our own prefix or None)"""
        node = self._node
        by_prefix = defaultdict(set)
        own_prefix = None
        own_name = node.get_name()

        for topic, _ in node.get_topic_names_and_types():
            for getter in (node.get_publishers_info_by_topic,
                           node.get_subscriptions_info_by_topic):
                try:
                    endpoints = getter(topic)
                except Exception:
                    continue
                for info in endpoints:
                    prefix = bytes(info.endpoint_gid[:12])
                    if info.node_name == own_name:
                        own_prefix = prefix
                        continue  # never report ourselves
                    if _is_reportable(info.node_name, info.node_namespace):
                        namespace = info.node_namespace.rstrip('/')
                        by_prefix[prefix].add(f'{namespace}/{info.node_name}')

        return by_prefix, own_prefix

    def _snapshot(self, rmw: str):
        by_prefix, own_prefix = self._collect_endpoints()

        if own_prefix is None:
            # Discovery has not settled yet; say nothing rather than claim the
            # middleware is unsupported.
            return

        if not carries_own_pid(own_prefix):
            self._set_status(
                DiscoveryMode.MANUAL, rmw,
                f'{rmw} does not expose node PIDs, so nodes cannot be found '
                f'automatically. They must register with ros2top to appear.')
            with self._lock:
                self._nodes = []
            return

        self._own_host = host_id(own_prefix)
        resolved = resolve(by_prefix, local_ros_pids(), self._own_host)

        self._set_status(DiscoveryMode.AUTO, rmw,
                         'nodes are discovered from the ROS graph')
        with self._lock:
            self._nodes = resolved
