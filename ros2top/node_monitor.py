#!/usr/bin/env python3
"""
Core node monitoring functionality
"""

import time
import psutil
from typing import Dict, List, Optional, NamedTuple, Tuple
from .discovery import discover_ros2_nodes
from .gpu_monitor import GPUMonitor


class NodeInfo(NamedTuple):
    """Information about a monitored process/node"""
    name: str
    pid: int
    cpu_percent: float
    ram_mb: float
    gpu_memory_mb: int
    gpu_utilization: float
    gpu_device_id: int
    start_time: float  # Unix timestamp


class NodeMonitor:
    """Monitor registered processes and their resource usage (supports both ROS2 nodes and generic processes)"""
    
    def __init__(self, refresh_interval: float = 5.0):
        self.refresh_interval = refresh_interval
        self.last_refresh = 0.0
        self.processes: Dict[str, psutil.Process] = {}
        self.cores = psutil.cpu_count() or 1
        self.gpu_monitor = GPUMonitor()
        self._ros2_available: Optional[bool] = None

    def cleanup(self):
        """Cleanup resources"""
        pass

    def is_ros2_available(self) -> bool:
        """Check if ROS2 CLI is reachable (lazy, cached)."""
        if self._ros2_available is None:
            import subprocess
            try:
                result = subprocess.run(['ros2', '--help'], capture_output=True, timeout=5)
                self._ros2_available = result.returncode == 0
            except Exception:
                self._ros2_available = False
        return self._ros2_available
    
    def is_gpu_available(self) -> bool:
        """Check if GPU monitoring is available"""
        return self.gpu_monitor.is_available()
    
    def get_gpu_count(self) -> int:
        """Get number of available GPUs"""
        return self.gpu_monitor.get_gpu_count()
    
    def update_nodes(self) -> bool:
        """
        Update the list of monitored nodes and processes
        
        Returns:
            True if nodes were updated, False otherwise
        """
        current_time = time.time()
        
        if current_time - self.last_refresh < self.refresh_interval:
            return False
            
        try:
            all_processes = self._get_all_processes_to_monitor()
            self._remove_dead_nodes(all_processes)
            self._add_new_nodes_with_pids(all_processes)
            
            self.last_refresh = current_time
            return True
            
        except Exception:
            return False
    
    def _get_all_processes_to_monitor(self) -> List[Tuple[str, int]]:
        """Discover running ROS2 nodes via psutil process inspection."""
        return discover_ros2_nodes()

    def _remove_dead_nodes(self, current_nodes: List[Tuple[str, int]]):
        """Remove processes for nodes that are no longer discovered."""
        current_unique_keys = {f"{name}:{pid}" for name, pid in current_nodes}
        nodes_to_remove = [key for key in self.processes if key not in current_unique_keys]
        for key in nodes_to_remove:
            del self.processes[key]
    
    def _add_new_nodes_with_pids(self, nodes_with_pids: List[Tuple[str, int]]):
        """Add new nodes to monitoring using pre-discovered PIDs"""
        for node, pid in nodes_with_pids:
            # Use a unique key combining node name and PID to allow multiple nodes with same name
            unique_key = f"{node}:{pid}"
            if unique_key not in self.processes:
                try:
                    proc = psutil.Process(pid)
                    # Initialize CPU measurement
                    proc.cpu_percent()
                    self.processes[unique_key] = proc
                except psutil.NoSuchProcess:
                    pass
    
    def cleanup_dead_processes(self):
        """Remove processes that are no longer running"""
        nodes_to_remove = []
        for node, process in self.processes.items():
            try:
                if not process.is_running():
                    nodes_to_remove.append(node)
            except psutil.NoSuchProcess:
                nodes_to_remove.append(node)
        
        for node in nodes_to_remove:
            del self.processes[node]
    
    def get_node_info_list(self) -> List[NodeInfo]:
        """
        Get information for all monitored nodes
        
        Returns:
            List of NodeInfo objects
        """
        node_infos = []
        
        for unique_key, process in self.processes.items():
            try:
                # Extract original node name from unique key (format: "node_name:pid")
                node_name = unique_key.rsplit(':', 1)[0]
                
                # Get CPU usage (normalized by number of cores)
                raw_cpu = process.cpu_percent()
                cpu_pct = raw_cpu / self.cores if self.cores > 0 else raw_cpu
                
                # Get RAM memory usage in MB
                memory_info = process.memory_info()
                ram_mb = memory_info.rss / (1024 * 1024)  # Resident Set Size in MB
                
                start_time = self._get_process_start_time(process)
                
                # Get GPU usage
                gpu_mem, gpu_util, gpu_id = self.gpu_monitor.get_gpu_usage(process.pid)
                
                node_info = NodeInfo(
                    name=node_name,
                    pid=process.pid,
                    cpu_percent=cpu_pct,
                    ram_mb=ram_mb,
                    gpu_memory_mb=gpu_mem,
                    gpu_utilization=gpu_util,
                    gpu_device_id=gpu_id,
                    start_time=start_time
                )
                
                node_infos.append(node_info)
                
            except psutil.NoSuchProcess:
                # Process died, will be cleaned up in next update
                continue
            except Exception:
                # Skip this process if we can't get info
                continue
        
        return node_infos
    
    def _get_process_start_time(self, process: psutil.Process) -> float:
        """Get process start time from psutil."""
        try:
            return process.create_time()
        except Exception:
            # If all else fails, use current time
            return time.time()
    
    def get_nodes_count(self) -> int:
        """Get number of monitored nodes"""
        return len(self.processes)
    
    def force_refresh(self):
        """Force refresh of node list on next update"""
        self.last_refresh = 0.0
    
    def kill_process(self, node_name: str, pid: int = None, force: bool = False) -> bool:
        """
        Kill a monitored process
        
        Args:
            node_name: Name of the node/process to kill
            pid: Specific PID to kill (optional, if not provided kills first match)
            force: If True, use SIGKILL instead of SIGTERM
            
        Returns:
            True if kill was successful, False otherwise
        """
        # Find the process to kill
        process_to_kill = None
        key_to_remove = None
        
        if pid is not None:
            # Look for specific node name and PID combination
            unique_key = f"{node_name}:{pid}"
            if unique_key in self.processes:
                process_to_kill = self.processes[unique_key]
                key_to_remove = unique_key
        else:
            # Find first process matching the node name
            for key, process in self.processes.items():
                if key.startswith(f"{node_name}:"):
                    process_to_kill = process
                    key_to_remove = key
                    break
        
        if process_to_kill is None:
            return False
            
        try:
            if force:
                # Force kill with SIGKILL
                process_to_kill.kill()
            else:
                # Graceful termination with SIGTERM
                process_to_kill.terminate()
            
            # Wait briefly to see if process terminates
            try:
                process_to_kill.wait(timeout=1.0)
            except psutil.TimeoutExpired:
                # Process didn't terminate within timeout
                pass
                
            return True
            
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            return False
    
    def shutdown(self):
        """Clean shutdown of monitoring"""
        self.processes.clear()
        self.gpu_monitor.shutdown()
    
    def get_system_info(self) -> Dict[str, str]:
        """Get system information"""
        info = {
            'CPU Cores': str(self.cores),
            'GPU Count': str(self.get_gpu_count()),
            'Monitored Nodes': str(self.get_nodes_count()),
        }

        info['ROS2 Available'] = str(self.is_ros2_available())

        return info
