"""
ROS2Top - A real-time monitor for ROS2 nodes showing CPU, RAM, and GPU usage
"""

__version__ = "0.1.3"
__author__ = "Ahmed Radwan"
__email__ = "ahmed.ali.radwan94@gmail.com"

# Import main monitoring functionality
from .node_monitor import NodeMonitor
from .gpu_monitor import GPUMonitor
from .ui.terminal_ui import run_ui

# Import node registration API for external use
from .node_registry import (
    register_node as _register_node, 
    unregister_node as _unregister_node, 
    heartbeat as _heartbeat,
    get_registered_nodes,
    get_registered_node_info,
    get_registry_location,
    get_registry_info,
    cleanup_stale_registrations
)

def register_node(node_name, additional_info=None):
    """
    Register a Python ROS2 node with ros2top monitoring
    
    Args:
        node_name (str): Name of the ROS2 node
        additional_info (dict, optional): Additional information about the node
    
    Returns:
        bool: True if registration was successful, False otherwise
    """
    if additional_info is None:
        additional_info = {}
    additional_info['language'] = 'python'
    return _register_node(node_name, additional_info)

def unregister_node(node_name):
    """
    Unregister a Python ROS2 node from ros2top monitoring
    
    Args:
        node_name (str): Name of the ROS2 node to unregister
    
    Returns:
        bool: True if unregistration was successful, False otherwise
    """
    return _unregister_node(node_name)

def heartbeat(node_name):
    """
    Send heartbeat to indicate the node is still alive
    
    Args:
        node_name (str): Name of the ROS2 node
    
    Returns:
        bool: True if heartbeat was successful, False otherwise
    """
    return _heartbeat(node_name)

# Make key functions available at package level
__all__ = [
    # Main classes
    'NodeMonitor',
    'GPUMonitor', 
    'run_ui',
    
    # Registration API
    'register_node',
    'unregister_node',
    'heartbeat',
    
    # Registry utilities
    'get_registered_nodes',
    'get_registered_node_info',
    'get_registry_location',
    'get_registry_info',
    'cleanup_stale_registrations',
]
