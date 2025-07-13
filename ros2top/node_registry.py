#!/usr/bin/env python3
"""
Node registration system for ros2top

This module allows ROS2 nodes to register themselves with ros2top
for monitoring purposes.
"""

import os
import json
import time
import atexit
import psutil
from typing import Dict, List, Tuple, Optional
from pathlib import Path


# Registry location - using user's home directory
REGISTRY_DIR = os.path.expanduser("~/.ros2top/registry")
REGISTRATION_FILE = os.path.join(REGISTRY_DIR, 'nodes.json')
LOCK_FILE = os.path.join(REGISTRY_DIR, 'nodes.lock')


def _ensure_registry_dir():
    """Ensure the registry directory exists"""
    Path(REGISTRY_DIR).mkdir(parents=True, exist_ok=True)

# Keep track of registered nodes for cleanup
_registered_nodes = set()


def register_node(node_name: str, additional_info: Optional[Dict] = None) -> bool:
    """
    Register a ROS2 node with ros2top monitoring
    
    Args:
        node_name: Name of the ROS2 node (with or without leading /)
        additional_info: Optional dictionary with extra node information
        
    Returns:
        True if registration was successful, False otherwise
        
    Example:
        import ros2top
        ros2top.register_node('/my_node', {'description': 'My awesome node'})
    """
    try:
        # Normalize node name
        if not node_name.startswith('/'):
            node_name = f'/{node_name}'
        
        # Get current process info
        current_process = psutil.Process()
        
        # Prepare node data
        node_data = {
            'node_name': node_name,
            'pid': current_process.pid,
            'ppid': current_process.ppid(),
            'process_name': current_process.name(),
            'cmdline': current_process.cmdline(),
            'registration_time': time.time(),
            'last_seen': time.time(),
        }
        
        # Add additional info if provided
        if additional_info:
            node_data['additional_info'] = additional_info
        
        # Register the node
        success = _write_node_registration(node_data)
        
        if success:
            # Track for cleanup
            _registered_nodes.add(node_name)
            
            # Register cleanup on exit
            atexit.register(_cleanup_node_on_exit, node_name)
            
        return success
        
    except Exception:
        return False


def unregister_node(node_name: str) -> bool:
    """
    Unregister a ROS2 node from ros2top monitoring
    
    Args:
        node_name: Name of the ROS2 node to unregister
        
    Returns:
        True if unregistration was successful, False otherwise
    """
    try:
        # Normalize node name
        if not node_name.startswith('/'):
            node_name = f'/{node_name}'
            
        success = _remove_node_registration(node_name)
        
        if success and node_name in _registered_nodes:
            _registered_nodes.remove(node_name)
            
        return success
        
    except Exception:
        return False


def heartbeat(node_name: str) -> bool:
    """
    Send a heartbeat for a registered node to indicate it's still alive
    
    Args:
        node_name: Name of the ROS2 node
        
    Returns:
        True if heartbeat was successful, False otherwise
    """
    try:
        # Normalize node name
        if not node_name.startswith('/'):
            node_name = f'/{node_name}'
            
        return _update_node_heartbeat(node_name)
        
    except Exception:
        return False


def get_registered_nodes() -> List[Tuple[str, int]]:
    """
    Get all currently registered nodes
    
    Returns:
        List of tuples: (node_name, pid)
    """
    try:
        nodes_data = _read_node_registrations()
        
        # Filter out stale registrations (only check if process is still running)
        active_nodes = []
        
        for node_name, data in nodes_data.items():
            # Check if process is still running
            try:
                proc = psutil.Process(data['pid'])
                if proc.is_running():
                    # Process is running, include it regardless of heartbeat timing
                    active_nodes.append((node_name, data['pid']))
                # Note: If heartbeats are being sent, we could add additional logic here
                # to detect unresponsive but running nodes
            except psutil.NoSuchProcess:
                # Process no longer exists, will be cleaned up later
                pass
                
        return active_nodes
        
    except Exception:
        return []


def get_registered_node_info(node_name: str) -> Optional[Dict]:
    """
    Get detailed information about a registered node
    
    Args:
        node_name: Name of the ROS2 node
        
    Returns:
        Dictionary with node information or None if not found
    """
    try:
        # Normalize node name
        if not node_name.startswith('/'):
            node_name = f'/{node_name}'
            
        nodes_data = _read_node_registrations()
        return nodes_data.get(node_name)
        
    except Exception:
        return None


def cleanup_stale_registrations() -> int:
    """
    Clean up registrations for nodes that are no longer running
    
    Returns:
        Number of stale registrations cleaned up
    """
    try:
        nodes_data = _read_node_registrations()
        stale_nodes = []
        
        for node_name, data in nodes_data.items():
            try:
                proc = psutil.Process(data['pid'])
                if not proc.is_running():
                    stale_nodes.append(node_name)
            except psutil.NoSuchProcess:
                stale_nodes.append(node_name)
                
        # Remove stale nodes
        for node_name in stale_nodes:
            _remove_node_registration(node_name)
            
        return len(stale_nodes)
        
    except Exception:
        return 0


def get_registry_location() -> str:
    """
    Get the current registry directory location
    
    Returns:
        Path to the registry directory being used
    """
    return REGISTRY_DIR


def get_registry_info() -> Dict[str, str]:
    """
    Get information about the registry system
    
    Returns:
        Dictionary with registry system information
    """
    return {
        'registry_dir': REGISTRY_DIR,
        'nodes_file': REGISTRATION_FILE,
        'lock_file': LOCK_FILE,
        'directory_exists': str(os.path.exists(REGISTRY_DIR)),
        'nodes_file_exists': str(os.path.exists(REGISTRATION_FILE)),
        'is_writable': str(os.access(REGISTRY_DIR, os.W_OK)) if os.path.exists(REGISTRY_DIR) else 'unknown'
    }


# Internal helper functions

def _write_node_registration(node_data: Dict) -> bool:
    """Write node registration to file with file locking"""
    import tempfile
    import shutil
    
    try:
        _ensure_registry_dir()
        
        # Create a lock file approach compatible with C++
        lock_acquired = False
        max_attempts = 100  # 1 second timeout
        
        for _ in range(max_attempts):
            try:
                # Try to create lock file exclusively
                with open(LOCK_FILE, 'x') as lock_file:
                    lock_file.write(str(os.getpid()))
                    lock_acquired = True
                    break
            except FileExistsError:
                # Lock file exists, wait and retry
                time.sleep(0.01)
                continue
        
        if not lock_acquired:
            return False
        
        try:
            # Read existing data
            existing_data = {}
            if os.path.exists(REGISTRATION_FILE):
                with open(REGISTRATION_FILE, 'r') as f:
                    try:
                        existing_data = json.load(f)
                    except json.JSONDecodeError:
                        # If JSON is corrupted, start fresh
                        existing_data = {}
            
            # Update with new node data
            existing_data[node_data['node_name']] = node_data
            
            # Write to temporary file first, then rename (atomic operation)
            temp_file = REGISTRATION_FILE + '.tmp'
            with open(temp_file, 'w') as f:
                json.dump(existing_data, f, indent=2)
            
            # Atomic rename
            shutil.move(temp_file, REGISTRATION_FILE)
            
            return True
            
        finally:
            # Always clean up lock file
            try:
                os.remove(LOCK_FILE)
            except FileNotFoundError:
                pass
        
    except Exception:
        # Clean up lock file on error
        try:
            os.remove(LOCK_FILE)
        except FileNotFoundError:
            pass
        return False



def _remove_node_registration(node_name: str) -> bool:
    """Remove node registration from file with file locking"""
    import shutil
    
    try:
        if not os.path.exists(REGISTRATION_FILE):
            return True
        
        # Create a lock file approach compatible with C++
        lock_acquired = False
        max_attempts = 100  # 1 second timeout
        
        for _ in range(max_attempts):
            try:
                # Try to create lock file exclusively
                with open(LOCK_FILE, 'x') as lock_file:
                    lock_file.write(str(os.getpid()))
                    lock_acquired = True
                    break
            except FileExistsError:
                # Lock file exists, wait and retry
                time.sleep(0.01)
                continue
        
        if not lock_acquired:
            return False
        
        try:
            # Read existing data
            with open(REGISTRATION_FILE, 'r') as f:
                try:
                    existing_data = json.load(f)
                except json.JSONDecodeError:
                    # If JSON is corrupted, nothing to remove
                    return True
            
            # Remove the node if it exists
            if node_name in existing_data:
                del existing_data[node_name]
                
                # Write to temporary file first, then rename (atomic operation)
                temp_file = REGISTRATION_FILE + '.tmp'
                with open(temp_file, 'w') as f:
                    json.dump(existing_data, f, indent=2)
                
                # Atomic rename
                shutil.move(temp_file, REGISTRATION_FILE)
                
            return True
            
        finally:
            # Always clean up lock file
            try:
                os.remove(LOCK_FILE)
            except FileNotFoundError:
                pass
        
    except Exception:
        # Clean up lock file on error
        try:
            os.remove(LOCK_FILE)
        except FileNotFoundError:
            pass
        return False


def _update_node_heartbeat(node_name: str) -> bool:
    """Update the last_seen timestamp for a node with file locking"""
    import shutil
    
    try:
        if not os.path.exists(REGISTRATION_FILE):
            return False
        
        # Create a lock file approach compatible with C++
        lock_acquired = False
        max_attempts = 100  # 1 second timeout
        
        for _ in range(max_attempts):
            try:
                # Try to create lock file exclusively
                with open(LOCK_FILE, 'x') as lock_file:
                    lock_file.write(str(os.getpid()))
                    lock_acquired = True
                    break
            except FileExistsError:
                # Lock file exists, wait and retry
                time.sleep(0.01)
                continue
        
        if not lock_acquired:
            return False
        
        try:
            # Read existing data
            with open(REGISTRATION_FILE, 'r') as f:
                try:
                    existing_data = json.load(f)
                except json.JSONDecodeError:
                    # If JSON is corrupted, can't update
                    return False
            
            # Update heartbeat if node exists
            if node_name in existing_data:
                existing_data[node_name]['last_seen'] = time.time()
                
                # Write to temporary file first, then rename (atomic operation)
                temp_file = REGISTRATION_FILE + '.tmp'
                with open(temp_file, 'w') as f:
                    json.dump(existing_data, f, indent=2)
                
                # Atomic rename
                shutil.move(temp_file, REGISTRATION_FILE)
                
                return True
                
            return False
            
        finally:
            # Always clean up lock file
            try:
                os.remove(LOCK_FILE)
            except FileNotFoundError:
                pass
        
    except Exception:
        # Clean up lock file on error
        try:
            os.remove(LOCK_FILE)
        except FileNotFoundError:
            pass
        return False


def _read_node_registrations() -> Dict:
    """Read all node registrations from file"""
    try:
        if not os.path.exists(REGISTRATION_FILE):
            return {}
            
        with open(REGISTRATION_FILE, 'r') as f:
            return json.load(f)
            
    except Exception:
        return {}


def _cleanup_node_on_exit(node_name: str):
    """Cleanup function called on process exit"""
    try:
        unregister_node(node_name)
    except Exception:
        pass
