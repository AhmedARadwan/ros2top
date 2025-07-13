#!/usr/bin/env python3
"""
Basic tests for ros2top functionality
"""

import unittest
from unittest.mock import patch, MagicMock
import sys
import os

# Add the parent directory to the path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from ros2top.ros2_utils import is_ros2_available, get_ros2_nodes, get_ros2_nodes_with_pids
from ros2top.gpu_monitor import GPUMonitor
from ros2top.node_monitor import NodeMonitor, NodeInfo
from ros2top.node_registry import register_node, unregister_node, get_registered_nodes


class TestROS2Utils(unittest.TestCase):
    """Test ROS2 utility functions"""
    
    @patch('subprocess.run')
    def test_is_ros2_available_success(self, mock_run):
        """Test ROS2 availability check when ROS2 is available"""
        mock_run.return_value.returncode = 0
        self.assertTrue(is_ros2_available())
    
    @patch('subprocess.run')
    def test_is_ros2_available_failure(self, mock_run):
        """Test ROS2 availability check when ROS2 is not available"""
        mock_run.side_effect = FileNotFoundError()
        self.assertFalse(is_ros2_available())
    
    @patch('ros2top.ros2_utils.get_registered_nodes')
    @patch('ros2top.ros2_utils.cleanup_stale_registrations')
    def test_get_ros2_nodes_from_registry(self, mock_cleanup, mock_get_registered):
        """Test getting ROS2 nodes from registry"""
        mock_get_registered.return_value = [("/node1", 1234), ("/node2", 1235), ("/node3", 1236)]
        
        nodes = get_ros2_nodes()
        self.assertEqual(nodes, ["/node1", "/node2", "/node3"])
        mock_cleanup.assert_called_once()
    
    @patch('ros2top.ros2_utils.get_registered_nodes')
    @patch('ros2top.ros2_utils.cleanup_stale_registrations')
    def test_get_ros2_nodes_with_pids_from_registry(self, mock_cleanup, mock_get_registered):
        """Test getting ROS2 nodes with PIDs from registry"""
        expected_result = [("/node1", 1234), ("/node2", 1235)]
        mock_get_registered.return_value = expected_result
        
        nodes_with_pids = get_ros2_nodes_with_pids()
        self.assertEqual(nodes_with_pids, expected_result)
        mock_cleanup.assert_called_once()
    
    @patch('ros2top.ros2_utils.get_registered_nodes')
    def test_get_ros2_nodes_failure(self, mock_get_registered):
        """Test getting ROS2 nodes when registry access fails"""
        mock_get_registered.side_effect = Exception("Registry error")
        nodes = get_ros2_nodes()
        self.assertEqual(nodes, [])


class TestNodeRegistry(unittest.TestCase):
    """Test node registry functionality"""
    
    def test_register_unregister_node(self):
        """Test basic node registration and unregistration"""
        test_node = "/test_registry_node"
        
        # Register a node
        success = register_node(test_node, {"description": "Test node"})
        self.assertTrue(success)
        
        # Check it appears in registered nodes
        registered = get_registered_nodes()
        node_names = [name for name, pid in registered]
        self.assertIn(test_node, node_names)
        
        # Unregister the node
        success = unregister_node(test_node)
        self.assertTrue(success)
        
        # Check it's no longer in registered nodes
        registered = get_registered_nodes()
        node_names = [name for name, pid in registered]
        self.assertNotIn(test_node, node_names)


class TestGPUMonitor(unittest.TestCase):
    """Test GPU monitoring functionality"""
    
    def test_gpu_monitor_init_no_nvml(self):
        """Test GPU monitor initialization when NVML is not available"""
        with patch('ros2top.gpu_monitor.NVML_AVAILABLE', False):
            monitor = GPUMonitor()
            self.assertFalse(monitor.is_available())
            self.assertEqual(monitor.get_gpu_count(), 0)
    
    def test_gpu_usage_no_gpu(self):
        """Test GPU usage when no GPU is available"""
        with patch('ros2top.gpu_monitor.NVML_AVAILABLE', False):
            monitor = GPUMonitor()
            gpu_mem, gpu_util, gpu_id = monitor.get_gpu_usage(1234)
            self.assertEqual((gpu_mem, gpu_util, gpu_id), (0, 0.0, -1))


class TestNodeMonitor(unittest.TestCase):
    """Test node monitoring functionality"""
    
    @patch('ros2top.node_monitor.is_ros2_available')
    def test_node_monitor_init(self, mock_ros2_available):
        """Test node monitor initialization"""
        mock_ros2_available.return_value = True
        monitor = NodeMonitor()
        self.assertTrue(monitor.is_ros2_available())
    
    @patch('ros2top.node_monitor.is_ros2_available')
    def test_node_monitor_no_ros2(self, mock_ros2_available):
        """Test node monitor when ROS2 is not available"""
        mock_ros2_available.return_value = False
        monitor = NodeMonitor()
        self.assertFalse(monitor.is_ros2_available())
        
        # Should still be able to update nodes (for registry-based monitoring)
        result = monitor.update_nodes()
        # Result depends on whether there are registered processes
        self.assertIsInstance(result, bool)
    
    @patch('ros2top.node_monitor.get_registered_nodes')
    @patch('ros2top.node_monitor.is_ros2_available')
    def test_node_monitor_with_registered_nodes(self, mock_ros2_available, mock_get_registered):
        """Test node monitor with registered nodes"""
        mock_ros2_available.return_value = False
        mock_get_registered.return_value = [("/test_node", 1234)]
        
        monitor = NodeMonitor(refresh_interval=0.1)  # Fast refresh for testing
        result = monitor.update_nodes()
        self.assertTrue(result)
    
    def test_node_info_structure(self):
        """Test NodeInfo namedtuple structure"""
        node_info = NodeInfo(
            name="/test_node",
            pid=1234,
            cpu_percent=25.5,
            ram_mb=128.0,
            gpu_memory_mb=512,
            gpu_utilization=75.0,
            gpu_device_id=0,
            start_time=1234567890.0
        )
        
        self.assertEqual(node_info.name, "/test_node")
        self.assertEqual(node_info.pid, 1234)
        self.assertEqual(node_info.cpu_percent, 25.5)
        self.assertEqual(node_info.ram_mb, 128.0)
        self.assertEqual(node_info.gpu_memory_mb, 512)
        self.assertEqual(node_info.gpu_utilization, 75.0)
        self.assertEqual(node_info.gpu_device_id, 0)
        self.assertEqual(node_info.start_time, 1234567890.0)
    
    @patch('ros2top.node_monitor.is_ros2_available')
    def test_node_monitor_system_info(self, mock_ros2_available):
        """Test getting system information"""
        mock_ros2_available.return_value = True
        monitor = NodeMonitor()
        
        system_info = monitor.get_system_info()
        
        # Check that system info contains expected keys
        expected_keys = ['CPU Cores', 'GPU Count', 'Monitored Nodes']
        for key in expected_keys:
            self.assertIn(key, system_info)
            self.assertIsInstance(system_info[key], str)


class TestUIFormatting(unittest.TestCase):
    """Test UI formatting functions"""
    
    def _format_uptime_seconds(self, uptime_seconds: int) -> str:
        """Helper function to format uptime from seconds (for testing)"""
        days = uptime_seconds // 86400
        hours = (uptime_seconds % 86400) // 3600
        minutes = (uptime_seconds % 3600) // 60
        seconds = uptime_seconds % 60
        return f"{days:02d}d:{hours:02d}h:{minutes:02d}m:{seconds:02d}s"
    
    def test_uptime_formatting(self):
        """Test uptime formatting function"""
        # Test various uptime durations
        test_cases = [
            (0, "00d:00h:00m:00s"),
            (30, "00d:00h:00m:30s"),
            (90, "00d:00h:01m:30s"),
            (3661, "00d:01h:01m:01s"),
            (90061, "01d:01h:01m:01s"),
            (273661, "03d:04h:01m:01s"),
        ]
        
        for seconds, expected in test_cases:
            with self.subTest(seconds=seconds):
                result = self._format_uptime_seconds(seconds)
                self.assertEqual(result, expected)
    
    @patch('time.time')
    def test_terminal_ui_uptime_formatting(self, mock_time):
        """Test TerminalUI uptime formatting with mocked time"""
        from ros2top.ui.terminal_ui import TerminalUI
        from ros2top.node_monitor import NodeMonitor
        
        # Mock current time
        mock_time.return_value = 1234567890.0
        
        # Create a UI instance
        monitor = NodeMonitor()
        ui = TerminalUI(monitor)
        
        # Test various cases
        test_cases = [
            (30, "30s"),  # Just seconds
            (90, "01m:30s"),  # Minutes and seconds
            (3661, "01h:01m:01s"),  # Hours, minutes, seconds
            (90061, "01d:01h:01m:01s"),  # Days, hours, minutes, seconds
        ]
        
        for offset, expected in test_cases:
            with self.subTest(offset=offset):
                start_time = 1234567890.0 - offset
                result = ui._format_uptime(start_time)
                self.assertEqual(result, expected)


class TestRegistryStartTime(unittest.TestCase):
    """Test that NodeMonitor uses registry start time"""
    
    @patch('ros2top.node_monitor.get_registered_node_info')
    @patch('psutil.Process')
    def test_registry_start_time_usage(self, mock_process, mock_get_info):
        """Test that NodeMonitor prefers registry start time over psutil"""
        # Mock registry returning start time
        registry_start_time = 1234567890.0
        mock_get_info.return_value = {
            'registration_time': registry_start_time,
            'pid': 1234
        }
        
        # Mock psutil process
        mock_proc = MagicMock()
        mock_proc.create_time.return_value = 1234567800.0  # Different time
        mock_process.return_value = mock_proc
        
        # Create monitor and test start time retrieval
        monitor = NodeMonitor()
        start_time = monitor._get_process_start_time("/test_node", mock_proc)
        
        # Should use registry time, not psutil time
        self.assertEqual(start_time, registry_start_time)
        mock_get_info.assert_called_once_with("/test_node")
    
    @patch('ros2top.node_monitor.get_registered_node_info')
    @patch('psutil.Process')
    def test_fallback_to_psutil_start_time(self, mock_process, mock_get_info):
        """Test fallback to psutil when registry info unavailable"""
        # Mock registry returning None
        mock_get_info.return_value = None
        
        # Mock psutil process
        psutil_start_time = 1234567800.0
        mock_proc = MagicMock()
        mock_proc.create_time.return_value = psutil_start_time
        mock_process.return_value = mock_proc
        
        # Create monitor and test start time retrieval
        monitor = NodeMonitor()
        start_time = monitor._get_process_start_time("/test_node", mock_proc)
        
        # Should use psutil time as fallback
        self.assertEqual(start_time, psutil_start_time)
        mock_proc.create_time.assert_called_once()


if __name__ == '__main__':
    unittest.main()
