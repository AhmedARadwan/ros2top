"""
ROS2Top - A real-time monitor for ROS2 nodes showing CPU, RAM, and GPU usage
"""

__version__ = "0.1.3"  # This will be automatically updated by GitHub Actions when publishing
__author__ = "Ahmed Radwan"
__email__ = "ahmed.ali.radwan94@gmail.com"

from .node_monitor import NodeMonitor, NodeInfo
from .gpu_monitor import GPUMonitor
from .discovery import discover_ros2_nodes
from .ui.terminal_ui import run_ui

__all__ = [
    'NodeMonitor',
    'NodeInfo',
    'GPUMonitor',
    'discover_ros2_nodes',
    'run_ui',
]
