#!/usr/bin/env python3
"""
Core node monitoring functionality
"""

import time
import psutil
from collections import defaultdict
from typing import Dict, List, Optional, NamedTuple, Tuple
from .ros2_utils import is_ros2_available, get_ros2_nodes_with_pids, check_ros2_environment
from .gpu_monitor import GPUMonitor
from .node_registry import get_registered_nodes, get_registered_node_info


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
    # How many monitored nodes live in this PID. >1 means composable nodes in a
    # shared container: the cpu/ram/gpu figures above are whole-process values
    # that cannot be split per node, and are identical for each of them.
    shared_count: int = 1


class NodeMonitor:
    """Monitor registered processes and their resource usage (supports both ROS2 nodes and generic processes)"""
    
    def __init__(self, refresh_interval: float = 5.0):
        self.refresh_interval = refresh_interval
        self.last_refresh = 0.0
        self.processes: Dict[str, psutil.Process] = {}
        # One sampler per PID, so a process hosting N nodes is measured once.
        # Keeping the same object across refreshes keeps cpu_percent()'s
        # interval baseline fresh.
        self._samplers: Dict[int, psutil.Process] = {}
        self.cores = psutil.cpu_count()
        self.gpu_monitor = GPUMonitor()
        
        # Check ROS2 availability
        self.ros2_available = is_ros2_available()
    
    def cleanup(self):
        """Cleanup resources"""
        pass
        
    def is_ros2_available(self) -> bool:
        """Check if ROS2 is available"""
        return self.ros2_available
    
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
            # Get all processes to monitor (ROS2 nodes + registered processes)
            all_processes = self._get_all_processes_to_monitor()
            
            # Update processes based on discovered nodes
            current_names = [name for name, pid in all_processes]
            self._remove_dead_nodes(current_names)
            self._add_new_nodes_with_pids(all_processes)
            
            self.last_refresh = current_time
            return True
            
        except Exception:
            return False
    
    def _get_all_processes_to_monitor(self) -> List[Tuple[str, int]]:
        """Get all processes to monitor (primarily from registry, optionally including ROS2 nodes)"""
        all_processes = []
        
        # Primary source: registered processes
        try:
            registered_nodes = get_registered_nodes()
            all_processes.extend(registered_nodes)
        except Exception:
            pass
        
        # Secondary source: ROS2 nodes (if available and not already in registry)
        if self.ros2_available:
            try:
                ros2_nodes = get_ros2_nodes_with_pids()
                # Only add ROS2 nodes that aren't already registered
                registered_names = {name for name, pid in all_processes}
                for name, pid in ros2_nodes:
                    if name not in registered_names:
                        all_processes.append((name, pid))
            except Exception:
                pass
            
        return all_processes
    
    def _remove_dead_nodes(self, current_nodes: List[str]):
        """Remove processes for nodes that no longer exist"""
        # Convert current_nodes to set of unique keys for comparison
        current_unique_keys = set()
        for node_name, pid in self._get_all_processes_to_monitor():
            current_unique_keys.add(f"{node_name}:{pid}")
        
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
                    # Start the CPU measurement interval now, not on first read,
                    # otherwise the node's first displayed sample is always 0.0
                    self._sampler(pid)
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
        # Composable nodes share one process, so group by PID and sample each
        # process exactly once. Sampling per node would query psutil/NVML N
        # times for the same PID and report that process's CPU, RAM and GPU
        # memory N times over in any total.
        nodes_by_pid: Dict[int, List[str]] = defaultdict(list)
        for unique_key in self.processes:
            node_name, _, pid = unique_key.rpartition(':')
            nodes_by_pid[int(pid)].append(node_name)

        self._prune_samplers(set(nodes_by_pid))

        node_infos = []

        for pid, node_names in nodes_by_pid.items():
            try:
                process = self._sampler(pid)

                # Get CPU usage (normalized by number of cores)
                raw_cpu = process.cpu_percent()
                cpu_pct = raw_cpu / self.cores if self.cores > 0 else raw_cpu

                # Get RAM memory usage in MB
                memory_info = process.memory_info()
                ram_mb = memory_info.rss / (1024 * 1024)  # Resident Set Size in MB

                # Get GPU usage
                gpu_mem, gpu_util, gpu_id = self.gpu_monitor.get_gpu_usage(pid)

            except psutil.NoSuchProcess:
                # Process died, will be cleaned up in next update
                continue
            except Exception:
                # Skip this process if we can't get info
                continue

            # Every node in this process reports the same whole-process figures;
            # shared_count tells the UI they must not be read as per-node.
            for node_name in node_names:
                node_infos.append(NodeInfo(
                    name=node_name,
                    pid=pid,
                    cpu_percent=cpu_pct,
                    ram_mb=ram_mb,
                    gpu_memory_mb=gpu_mem,
                    gpu_utilization=gpu_util,
                    gpu_device_id=gpu_id,
                    # Start time is per node: a composed node is loaded into an
                    # already-running container, so it is younger than the process.
                    start_time=self._get_process_start_time(node_name, process),
                    shared_count=len(node_names),
                ))

        # Stable, process-grouped order. The UI selects and kills by row index
        # into this list, so it must not sort independently or the row on screen
        # and the node acted on would drift apart.
        #
        # Within a process the oldest node comes first: composable nodes are
        # loaded into an already-running container, so the container's own node
        # is always the eldest and belongs at the head of its group.
        node_infos.sort(key=lambda n: (n.pid, n.start_time, n.name or ""))
        return node_infos

    def _sampler(self, pid: int) -> psutil.Process:
        """Get the cached psutil.Process used to measure this PID"""
        process = self._samplers.get(pid)
        if process is None:
            process = psutil.Process(pid)
            process.cpu_percent()  # prime the interval baseline
            self._samplers[pid] = process
        return process

    def _prune_samplers(self, live_pids: set):
        """Drop samplers for PIDs we no longer monitor"""
        for pid in [p for p in self._samplers if p not in live_pids]:
            del self._samplers[pid]

    def get_process_totals(self, node_infos: Optional[List[NodeInfo]] = None
                           ) -> Tuple[float, float, int]:
        """
        Aggregate CPU%, RAM MB and GPU MB across monitored processes.

        Counts each PID once, so a container hosting N nodes contributes its
        usage a single time rather than N times.

        Pass the list from get_node_info_list() to total the numbers already on
        screen; omitting it takes a fresh sample over a new, very short CPU
        interval, which will not match the displayed rows.
        """
        if node_infos is None:
            node_infos = self.get_node_info_list()

        seen = set()
        cpu = ram = 0.0
        gpu = 0
        for info in node_infos:
            if info.pid in seen:
                continue
            seen.add(info.pid)
            cpu += info.cpu_percent
            ram += info.ram_mb
            gpu += info.gpu_memory_mb
        return cpu, ram, gpu
    
    def _get_process_start_time(self, node_name: str, process: psutil.Process) -> float:
        """Get node start time, preferring registry registration time"""
        try:
            # First try to get registration time from registry. Pass the PID:
            # two processes can host nodes of the same name, and a container's
            # composed nodes each register at their own time.
            registry_info = get_registered_node_info(node_name, process.pid)
            if registry_info and 'registration_time' in registry_info:
                return registry_info['registration_time']
        except Exception:
            pass
        
        # Fall back to psutil create_time
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
    
    def _node_name_of(self, key: str) -> str:
        """Node name held in a self.processes key, tolerating a missing ':pid'"""
        name, sep, tail = key.rpartition(':')
        return name if sep and tail.isdigit() else key

    def kill_process(self, node_name: str, pid: int = None, force: bool = False) -> bool:
        """
        Kill a monitored process

        Killing is per process, not per node: if the target shares its process
        with composable nodes, they all die with it.

        Args:
            node_name: Name of the node/process to kill
            pid: PID to kill. Node names are not unique, so without this the
                 first match wins, which may not be the node you meant.
            force: If True, use SIGKILL instead of SIGTERM

        Returns:
            True if kill was successful, False otherwise
        """
        # Find the process to kill. Identity comes from the stored Process
        # object, not from parsing the key, so it holds however the key was
        # built.
        process_to_kill = None

        for key, process in self.processes.items():
            if self._node_name_of(key) != node_name:
                continue
            if pid is not None and process.pid != pid:
                continue
            process_to_kill = process
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

            # The whole process is gone, so drop every node it was hosting,
            # not just the one that was selected
            dead_pid = process_to_kill.pid
            for key in [k for k, p in self.processes.items() if p.pid == dead_pid]:
                del self.processes[key]
            self._samplers.pop(dead_pid, None)

            return True

        except (psutil.NoSuchProcess, psutil.AccessDenied):
            return False
    
    def shutdown(self):
        """Clean shutdown of monitoring"""
        self.processes.clear()
        self._samplers.clear()
        self.gpu_monitor.shutdown()
    
    def get_system_info(self) -> Dict[str, str]:
        """Get system information"""
        info = {
            'CPU Cores': str(self.cores),
            'GPU Count': str(self.get_gpu_count()),
            'Monitored Nodes': str(self.get_nodes_count()),
        }
        
        # Add ROS2 environment info if available
        try:
            ros2_env = check_ros2_environment()
            info.update(ros2_env)
        except Exception:
            # If ROS2 environment check fails, just skip it
            info['ROS2 Available'] = str(self.ros2_available)
        
        return info
