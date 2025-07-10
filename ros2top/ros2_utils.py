#!/usr/bin/env python3
"""
ROS2 utilities for node discovery and process mapping
"""

import subprocess
import re
import time
import tempfile
import os
import threading
from typing import List, Optional, Dict, Tuple

# Constants
NOT_SET = 'Not set'

# Global node tracking
_node_tid_cache = {}  # {node_name: tid}
_node_pid_cache = {}  # {node_name: pid}
_tracing_thread = None
_tracing_active = False
_cache_lock = threading.Lock()

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
    Get PID for a ROS2 node using 'ros2 node info' (fallback method)
    
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

def get_ros2_nodes_with_pids() -> List[Tuple[str, int]]:
    """
    Get list of ROS2 nodes with their PIDs using cached data from background thread
    
    Returns:
        List of tuples: (node_name, pid)
    """
    # Start background tracing if not already running
    start_background_tracing()
    
    # Get cached data
    cached_pids = get_cached_node_pids()
    
    if cached_pids:
        return list(cached_pids.items())
    
    # If no cached data available, fall back to direct method
    # This happens on first call before background thread has populated cache
    nodes = get_ros2_nodes()
    if not nodes:
        return []
    
    if is_tracing_available():
        return _get_nodes_with_tracing(nodes)
    else:
        return _get_nodes_with_fallback(nodes)

def _get_nodes_with_tracing(nodes: List[str]) -> List[Tuple[str, int]]:
    """Get nodes with PIDs using tracing"""
    node_pids = []
    try:
        tids = get_ros2_tids_from_trace()
        for node in nodes:
            pid = find_pid_for_node(node, tids)
            if pid:
                node_pids.append((node, pid))
    except Exception:
        # If tracing fails, use fallback
        return _get_nodes_with_fallback(nodes)
    return node_pids

def _get_nodes_with_fallback(nodes: List[str]) -> List[Tuple[str, int]]:
    """Get nodes with PIDs using fallback method"""
    node_pids = []
    for node in nodes:
        pid = get_node_pid(node)
        if pid:
            node_pids.append((node, pid))
    return node_pids

def is_tracing_available() -> bool:
    """Check if ROS2 tracing is available"""
    try:
        result = subprocess.run(['ros2', 'trace', '--help'], 
                              capture_output=True, 
                              text=True, 
                              timeout=5)
        return result.returncode == 0
    except (subprocess.TimeoutExpired, FileNotFoundError):
        return False

def get_ros2_tids_from_trace() -> List[int]:
    """
    Get ROS2 node TIDs using tracing
    
    Returns:
        List of thread IDs
    """
    session_name = f'ros2top_trace_{int(time.time())}'
    
    try:
        # Setup tracing session with proper listing
        setup_result = subprocess.run(['ros2', 'trace', '--session-name', session_name, '--list'], 
                                    capture_output=True, 
                                    text=True,
                                    timeout=10)
        
        if setup_result.returncode != 0:
            return []
        
        # Let it collect trace data for a bit
        time.sleep(1.0)
        
        # Analyze trace
        trace_dir = os.path.expanduser(f'~/.ros/tracing/{session_name}')
        tids = analyze_trace_for_tids(trace_dir)
        
        # Cleanup trace directory
        try:
            subprocess.run(['rm', '-rf', trace_dir], 
                          stdout=subprocess.DEVNULL,
                          stderr=subprocess.DEVNULL)
        except subprocess.SubprocessError:
            pass
            
        return tids
        
    except Exception:
        return []

def analyze_trace_for_tids(trace_path: str) -> List[int]:
    """
    Analyze trace to extract TIDs
    
    Args:
        trace_path: Path to trace directory
        
    Returns:
        List of thread IDs
    """
    try:
        # Try to import tracing analysis tools
        from tracetools_analysis.loading import load_file
        from tracetools_analysis.processor import Processor
        from tracetools_analysis.processor.ros2 import Ros2Handler
        from tracetools_analysis.utils.ros2 import Ros2DataModelUtil
        
        events = load_file(trace_path)
        ros2h = Ros2Handler()
        Processor(ros2h).process(events)
        util = Ros2DataModelUtil(ros2h.data)
        tids = util.get_tids()
        
        return list(tids) if tids else []
        
    except ImportError:
        # Tracing analysis tools not available
        return []
    except Exception:
        # Analysis failed
        return []

def find_pid_for_node(node_name: str, tids: List[int]) -> Optional[int]:
    """
    Find PID for a node given its name and available TIDs
    """
    import psutil
    
    # Clean up node name
    clean_node_name = node_name.lstrip('/')
    
    try:
        # Try to match TIDs to processes
        for tid in tids:
            pid = _find_pid_by_tid(tid, clean_node_name)
            if pid:
                return pid
                
        # If no direct match, try fallback matching
        return _find_pid_by_process_name(node_name)
        
    except Exception:
        return _find_pid_by_process_name(node_name)

def _find_pid_by_tid(tid: int, node_name: str) -> Optional[int]:
    """Find PID by matching TID to process threads"""
    import psutil
    
    try:
        for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
            if _process_has_tid_and_node(proc, tid, node_name):
                return proc.info['pid']
    except Exception:
        pass
    
    return None

def _process_has_tid_and_node(proc, tid: int, node_name: str) -> bool:
    """Check if process has the TID and is related to the node"""
    import psutil
    
    try:
        # Check if this process has the TID as a thread
        if hasattr(proc, 'threads'):
            thread_ids = [t.id for t in proc.threads()]
            if tid in thread_ids:
                # Check if this process is related to our node
                if proc.info['cmdline']:
                    cmdline = ' '.join(proc.info['cmdline'])
                    return node_name in cmdline
    except psutil.NoSuchProcess:
        pass
    return False
    
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

def start_background_tracing():
    """Start background thread for continuous node tracking"""
    global _tracing_thread, _tracing_active
    
    if _tracing_thread and _tracing_thread.is_alive():
        return  # Already running
    
    _tracing_active = True
    _tracing_thread = threading.Thread(target=_background_trace_worker, daemon=True)
    _tracing_thread.start()

def stop_background_tracing():
    """Stop background tracing thread"""
    global _tracing_active
    _tracing_active = False
    
    if _tracing_thread and _tracing_thread.is_alive():
        _tracing_thread.join(timeout=2.0)

def _background_trace_worker():
    """Background worker that continuously updates node TID/PID mappings"""
    while _tracing_active:
        try:
            _update_node_cache()
            time.sleep(2.0)  # Update every 2 seconds
        except Exception:
            time.sleep(1.0)  # On error, sleep and continue

def _update_node_cache():
    """Update the node cache with current tracing data"""
    nodes = get_ros2_nodes()
    
    if not nodes or not is_tracing_available():
        return
    
    # Run trace and analysis
    tids = get_ros2_tids_from_trace_fast()
    
    # Update cache with new mappings
    with _cache_lock:
        _node_tid_cache.clear()
        _node_pid_cache.clear()
        
        for node in nodes:
            _process_node_for_cache(node, tids)

def _process_node_for_cache(node: str, tids: List[int]):
    """Process a single node for cache update"""
    # Find TID for this node
    tid = _find_best_tid_for_node(node, tids)
    if tid:
        _node_tid_cache[node] = tid
        # Convert TID to PID
        pid = _tid_to_pid(tid)
        if pid:
            _node_pid_cache[node] = pid
    else:
        # Fallback to process name matching
        pid = _find_pid_by_process_name(node)
        if pid:
            _node_pid_cache[node] = pid

def get_cached_node_pids() -> Dict[str, int]:
    """Get the current cache of node name to PID mappings"""
    with _cache_lock:
        return _node_pid_cache.copy()

def get_cached_node_tids() -> Dict[str, int]:
    """Get the current cache of node name to TID mappings"""
    with _cache_lock:
        return _node_tid_cache.copy()

def _find_best_tid_for_node(node_name: str, tids: List[int]) -> Optional[int]:
    """Find the best TID match for a given node name"""
    import psutil
    
    clean_node_name = node_name.lstrip('/')
    
    # Try to find exact matches first
    for tid in tids:
        if _tid_matches_node(tid, clean_node_name, exact=True):
            return tid
    
    # Try partial matches
    for tid in tids:
        if _tid_matches_node(tid, clean_node_name, exact=False):
            return tid
    
    return None

def _tid_matches_node(tid: int, node_name: str, exact: bool = True) -> bool:
    """Check if a TID belongs to a process related to the node"""
    import psutil
    
    try:
        for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
            if _check_process_for_tid_match(proc, tid, node_name, exact):
                return True
    except Exception:
        pass
    
    return False

def _check_process_for_tid_match(proc, tid: int, node_name: str, exact: bool) -> bool:
    """Check if a specific process matches the TID and node"""
    import psutil
    
    try:
        if not hasattr(proc, 'threads') or not proc.info['cmdline']:
            return False
            
        thread_ids = [t.id for t in proc.threads()]
        if tid not in thread_ids:
            return False
            
        cmdline = ' '.join(proc.info['cmdline'])
        if exact:
            return node_name in cmdline
        else:
            # Partial matching for broader detection
            return (node_name.lower() in cmdline.lower() or 
                   any(part in cmdline.lower() for part in node_name.split('_')))
    except psutil.NoSuchProcess:
        return False

def _tid_to_pid(tid: int) -> Optional[int]:
    """Convert a TID to its parent PID"""
    import psutil
    
    try:
        for proc in psutil.process_iter(['pid']):
            try:
                if hasattr(proc, 'threads'):
                    thread_ids = [t.id for t in proc.threads()]
                    if tid in thread_ids:
                        return proc.info['pid']
            except psutil.NoSuchProcess:
                continue
    except Exception:
        pass
    
    return None

def get_ros2_tids_from_trace_fast() -> List[int]:
    """
    Fast version of tracing that doesn't clean up immediately
    for use in background thread
    """
    session_name = f'ros2top_bg_{int(time.time())}'
    
    try:
        # Start tracing
        start_result = subprocess.run(['ros2', 'trace', 'start', session_name], 
                                    capture_output=True, 
                                    text=True,
                                    timeout=5)
        
        if start_result.returncode != 0:
            return []
        
        # Let it collect trace data briefly
        time.sleep(0.8)
        
        # Stop tracing
        subprocess.run(['ros2', 'trace', 'stop', session_name], 
                      capture_output=True, 
                      text=True,
                      timeout=5)
        
        # Analyze trace
        trace_dir = os.path.expanduser(f'~/.ros/tracing/{session_name}')
        tids = analyze_trace_for_tids(trace_dir)
        
        # Cleanup in background (non-blocking)
        threading.Thread(target=_cleanup_trace_dir, args=(trace_dir,), daemon=True).start()
            
        return tids
        
    except Exception:
        return []

def _cleanup_trace_dir(trace_dir: str):
    """Clean up trace directory in background"""
    try:
        time.sleep(1.0)  # Give some time for analysis to complete
        subprocess.run(['rm', '-rf', trace_dir], 
                      stdout=subprocess.DEVNULL,
                      stderr=subprocess.DEVNULL)
    except Exception:
        pass
