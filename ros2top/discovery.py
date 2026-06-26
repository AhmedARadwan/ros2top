#!/usr/bin/env python3
"""
Automatic ROS2 node discovery via psutil process inspection.

Identifies ROS2 nodes by scanning running processes for the colcon install/
path signature — no code changes required in monitored nodes.

Ported from rover/ros2/src/system_status/system_status/node_resources_monitor.py
(ResourcesMonitor.get_nodes_process_list) and extended with generic ROS2 support.
"""

import subprocess
from typing import List, Tuple

import psutil

# Colcon-generated setup scripts that are not actual ROS2 nodes
_EXCLUDED_SUFFIXES = ["_local_setup_util_", "local_setup.", "setup."]


def discover_ros2_nodes() -> List[Tuple[str, int]]:
    """
    Discover running ROS2 nodes by inspecting process command lines.

    Looks for processes whose executable path contains an install/ directory
    (the colcon workspace install tree) and belongs to a ros2 or rover package.
    Also handles C++ nodes launched from lib/ and component containers.

    Returns:
        List of (node_name, pid) tuples for all discovered nodes.
    """
    nodes: List[Tuple[str, int]] = []

    for process in psutil.process_iter(["pid", "cmdline"]):
        try:
            cmd = process.info["cmdline"]
            if not cmd:
                continue

            # Python nodes launched via `python3 <script>` have the interpreter at index 0
            exec_idx = 1 if cmd[0] == "/usr/bin/python3" else 0
            if exec_idx >= len(cmd):
                continue

            exec_parts = cmd[exec_idx].split("/")
            exec_tag = exec_parts[-1]

            # Only care about processes inside a colcon install tree or component containers
            in_install = "install" in exec_parts
            is_container = "component_container_isolated" in exec_parts

            if not in_install and not is_container:
                continue

            # Skip colcon-generated setup/utility scripts
            if any(excluded in exec_tag for excluded in _EXCLUDED_SUFFIXES):
                continue

            # Python ROS2 nodes: executable path contains ros2/ or rover/
            if "ros2" in exec_parts or "rover" in exec_parts:
                nodes.append((exec_tag, process.pid))

            # C++ nodes installed under lib/
            elif "lib" in exec_parts:
                if "container" in exec_tag:
                    exec_tag = _resolve_container_node_name(process.pid, exec_tag)
                nodes.append((exec_tag, process.pid))

        except (IndexError, psutil.NoSuchProcess, psutil.AccessDenied):
            continue

    return nodes


def _resolve_container_node_name(pid: int, fallback: str) -> str:
    """
    Extract the actual node name from a component_container_isolated process.

    Component containers pass the node name as __node:=<name> in their args.
    Falls back to the executable tag if parsing fails.
    """
    try:
        cmd_output = subprocess.check_output(
            ["ps", "-p", str(pid), "-o", "cmd="],
            timeout=2,
        ).decode("utf-8")

        for segment in cmd_output.split("/"):
            if "container" in segment:
                for token in segment.split(" "):
                    if token.startswith("__node:="):
                        return token.split("=", 1)[-1].rstrip()
    except Exception:
        pass

    return fallback
