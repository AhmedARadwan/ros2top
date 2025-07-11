#!/usr/bin/env python3
"""
Simplified ROS2 utilities using node registry as primary source
"""

import subprocess
from typing import List, Tuple, Optional, Dict
from .node_registry import get_registered_nodes, cleanup_stale_registrations


def is_ros2_available() -> bool:
    """Check if ROS2 is available in the environment"""
    try:
        result = subprocess.run(['ros2', '--help'], 
                              capture_output=True, 
                              text=True, 
                              timeout=5)
        return result.returncode == 0
    except (subprocess.TimeoutExpired, FileNotFoundError):
        return False


def get_ros2_nodes() -> List[str]:
    """Get list of ROS2 node names from the registry"""
    try:
        cleanup_stale_registrations()  # Clean up dead registrations
        registered_nodes = get_registered_nodes()
        return [node_name for node_name, _ in registered_nodes]
    except Exception:
        return []


def get_node_pid(node_name: str) -> Optional[int]:
    """Get PID for a specific ROS2 node from the registry"""
    try:
        cleanup_stale_registrations()
        registered_nodes = get_registered_nodes()
        for name, pid in registered_nodes:
            if name == node_name:
                return pid
        return None
    except Exception:
        return None


def get_ros2_nodes_with_pids() -> List[Tuple[str, int]]:
    """
    Get list of ROS2 nodes with their PIDs from the registry
    
    Returns:
        List of tuples: (node_name, pid)
    """
    try:
        cleanup_stale_registrations()  # Clean up dead registrations
        return get_registered_nodes()
    except Exception:
        return []


def check_ros2_environment() -> Dict[str, str]:
    """Check ROS2 environment variables and status"""
    import os
    
    env_info = {}
    
    # Check key ROS2 environment variables
    ros_vars = [
        'ROS_DOMAIN_ID',
        'ROS_LOCALHOST_ONLY', 
        'RMW_IMPLEMENTATION',
        'ROS_DISTRO'
    ]
    
    for var in ros_vars:
        env_info[var] = os.environ.get(var, 'Not set')
    
    # Check if ROS2 is available
    env_info['ros2_available'] = str(is_ros2_available())
    
    return env_info


# Compatibility functions for existing code
def start_background_tracing():
    """Compatibility function - no longer needed with registry approach"""
    pass


def stop_background_tracing():
    """Compatibility function - no longer needed with registry approach"""
    pass
