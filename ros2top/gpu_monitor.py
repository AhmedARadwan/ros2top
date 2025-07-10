#!/usr/bin/env python3
"""
GPU monitoring utilities using NVIDIA Management Library (NVML)
"""

import psutil
from typing import Tuple, List, Optional

try:
    import pynvml
    NVML_AVAILABLE = True
except ImportError:
    NVML_AVAILABLE = False
    pynvml = None


class GPUMonitor:
    """Monitor GPU usage for processes"""
    
    def __init__(self):
        self.gpu_available = False
        self.device_count = 0
        
        if NVML_AVAILABLE:
            try:
                pynvml.nvmlInit()
                self.device_count = pynvml.nvmlDeviceGetCount()
                self.gpu_available = True
            except Exception:
                self.gpu_available = False
    
    def is_available(self) -> bool:
        """Check if GPU monitoring is available"""
        return self.gpu_available
    
    def get_gpu_count(self) -> int:
        """Get number of available GPUs"""
        return self.device_count if self.gpu_available else 0
    
    def get_gpu_usage(self, pid: int) -> Tuple[int, float, int]:
        """
        Get GPU usage for a specific process and its children
        
        Args:
            pid: Process ID to check
            
        Returns:
            Tuple of (gpu_memory_mb, gpu_utilization_percent, gpu_device_id)
            Returns (0, 0.0, -1) if no GPU usage found
        """
        if not self.gpu_available:
            return 0, 0.0, -1
            
        try:
            # Get all PIDs to check (main process + children)
            pids_to_check = [pid]
            try:
                process = psutil.Process(pid)
                children = process.children(recursive=True)
                pids_to_check.extend([child.pid for child in children])
            except psutil.NoSuchProcess:
                pass
            
            for i in range(self.device_count):
                handle = pynvml.nvmlDeviceGetHandleByIndex(i)
                try:
                    procs = pynvml.nvmlDeviceGetComputeRunningProcesses(handle)
                    
                    for proc in procs:
                        if proc.pid in pids_to_check:
                            # Get memory usage in MB
                            gpu_mem = proc.usedGpuMemory // (1024 * 1024)
                            
                            # Get GPU utilization (this gives overall GPU util, not per-process)
                            util = pynvml.nvmlDeviceGetUtilizationRates(handle)
                            return gpu_mem, float(util.gpu), i
                except pynvml.NVMLError:
                    # Skip this GPU if we can't access it
                    continue
            
            return 0, 0.0, -1  # No GPU usage found
        except Exception:
            return 0, 0.0, -1
    
    def get_all_gpu_processes(self) -> List[Tuple[int, str, int, int, int]]:
        """
        Debug function to get all GPU processes with process names
        
        Returns:
            List of tuples: (pid, process_name, parent_pid, gpu_memory_mb, gpu_device_id)
        """
        if not self.gpu_available:
            return []
            
        try:
            all_procs = []
            
            for i in range(self.device_count):
                handle = pynvml.nvmlDeviceGetHandleByIndex(i)
                try:
                    procs = pynvml.nvmlDeviceGetComputeRunningProcesses(handle)
                    for proc in procs:
                        try:
                            process_name = psutil.Process(proc.pid).name()
                            parent_pid = psutil.Process(proc.pid).ppid()
                        except psutil.NoSuchProcess:
                            process_name = "unknown"
                            parent_pid = -1
                        
                        gpu_mem_mb = proc.usedGpuMemory // (1024 * 1024)
                        all_procs.append((proc.pid, process_name, parent_pid, gpu_mem_mb, i))
                except pynvml.NVMLError:
                    continue
            
            return all_procs
        except Exception:
            return []
    
    def get_gpu_info(self, device_id: int) -> Optional[dict]:
        """
        Get information about a specific GPU device
        
        Args:
            device_id: GPU device index
            
        Returns:
            Dictionary with GPU information or None if not available
        """
        if not self.gpu_available or device_id >= self.device_count:
            return None
            
        try:
            handle = pynvml.nvmlDeviceGetHandleByIndex(device_id)
            
            name = pynvml.nvmlDeviceGetName(handle).decode('utf-8')
            memory_info = pynvml.nvmlDeviceGetMemoryInfo(handle)
            utilization = pynvml.nvmlDeviceGetUtilizationRates(handle)
            
            return {
                'name': name,
                'memory_total_mb': memory_info.total // (1024 * 1024),
                'memory_used_mb': memory_info.used // (1024 * 1024),
                'memory_free_mb': memory_info.free // (1024 * 1024),
                'utilization_gpu': utilization.gpu,
                'utilization_memory': utilization.memory,
            }
        except Exception:
            return None
    
    def shutdown(self):
        """Clean shutdown of GPU monitoring"""
        if self.gpu_available:
            try:
                pynvml.nvmlShutdown()
            except Exception:
                pass
