#!/usr/bin/env python3
"""
ROS2 utilities for node discovery and process mapping
"""

import subprocess
import re
from typing import List, Optional, Dict

# Constants
NOT_SET = 'Not set'

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
    """Get list of ROS2 nodes using 'ros2 node list'"""
    try:
        result = subprocess.run(['ros2', 'node', 'list'], 
                              capture_output=True, 
                              text=True, 
                              timeout=10)
        if result.returncode == 0:
            nodes = result.stdout.strip().split('\n')
            # Filter out empty lines
            return [node.strip() for node in nodes if node.strip()]
        return []
    except (subprocess.TimeoutExpired, FileNotFoundError):
        return []

def get_node_pid(node_name: str) -> Optional[int]:
    """
    Get PID for a ROS2 node using 'ros2 node info'
    
    Note: ROS2 doesn't directly provide PID info like ROS1 did,
    so we need to parse process information or use alternative methods.
    """
    try:
        # Try ros2 node info first
        result = subprocess.run(['ros2', 'node', 'info', node_name], 
                              capture_output=True, 
                              text=True, 
                              timeout=10)
        
        if result.returncode == 0:
            # Look for any PID information in the output
            for line in result.stdout.splitlines():
                if 'pid' in line.lower() or 'process' in line.lower():
                    # Try to extract numbers from the line
                    numbers = re.findall(r'\d+', line)
                    if numbers:
                        return int(numbers[0])
        
        # Fallback: try to match node name to running processes
        return _find_pid_by_process_name(node_name)
        
    except (subprocess.TimeoutExpired, FileNotFoundError):
        return None

def _find_pid_by_process_name(node_name: str) -> Optional[int]:
    """
    Fallback method to find PID by matching process names/command lines
    """
    import psutil
    
    # Clean up node name (remove leading slash if present)
    clean_node_name = node_name.lstrip('/')
    
    try:
        for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
            try:
                # Check if node name appears in command line
                if proc.info['cmdline']:
                    cmdline = ' '.join(proc.info['cmdline'])
                    
                    # Look for various patterns that might indicate this process
                    # is associated with the ROS2 node
                    patterns = [
                        clean_node_name,
                        f'__node:={clean_node_name}',
                        f'--ros-args.*{clean_node_name}',
                    ]
                    
                    for pattern in patterns:
                        if re.search(pattern, cmdline, re.IGNORECASE):
                            return proc.info['pid']
                            
            except psutil.NoSuchProcess:
                continue
                
    except Exception:
        pass
    
    return None

def get_node_info_dict(node_name: str) -> Dict[str, str]:
    """Get detailed node information as a dictionary"""
    try:
        result = subprocess.run(['ros2', 'node', 'info', node_name], 
                              capture_output=True, 
                              text=True, 
                              timeout=10)
        
        info = {}
        if result.returncode == 0:
            current_section = None
            for line in result.stdout.splitlines():
                line = line.strip()
                if line.endswith(':'):
                    current_section = line[:-1]
                    info[current_section] = []
                elif line and current_section:
                    info[current_section].append(line)
                    
        return info
        
    except (subprocess.TimeoutExpired, FileNotFoundError):
        return {}

def check_ros2_environment() -> Dict[str, str]:
    """Check ROS2 environment variables and status"""
    import os
    
    env_info = {
        'ROS_DOMAIN_ID': os.environ.get('ROS_DOMAIN_ID', NOT_SET),
        'RMW_IMPLEMENTATION': os.environ.get('RMW_IMPLEMENTATION', NOT_SET),
        'ROS_DISTRO': os.environ.get('ROS_DISTRO', NOT_SET),
    }
    
    # Check if ros2 command is available
    env_info['ROS2_AVAILABLE'] = 'Yes' if is_ros2_available() else 'No'
    
    return env_info
