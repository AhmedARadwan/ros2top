"""
ROS2Top - A real-time monitor for ROS2 nodes showing CPU, RAM, and GPU usage
"""

__version__ = "0.1.0"
__author__ = "Ahmed Radwan"
__email__ = "ahmed.ali.radwan94@gmail.com"

# Import main monitoring functionality
from .node_monitor import NodeMonitor
from .gpu_monitor import GPUMonitor
from .ui.terminal_ui import run_ui

# Import node registration API for external use
from .node_registry import (
    register_node, 
    unregister_node, 
    heartbeat,
    get_registered_nodes,
    get_registered_node_info,
    get_registry_location,
    get_registry_info,
    cleanup_stale_registrations
)

# Make key functions available at package level
__all__ = [
    'NodeMonitor',
    'GPUMonitor', 
    'run_ui',
    'register_node',
    'unregister_node',
    'heartbeat',
    'get_registered_nodes',
    'get_registered_node_info',
    'get_registry_location',
    'get_registry_info',
    'cleanup_stale_registrations',
]
