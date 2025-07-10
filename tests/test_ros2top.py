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

from ros2top.ros2_utils import is_ros2_available, get_ros2_nodes
from ros2top.gpu_monitor import GPUMonitor
from ros2top.node_monitor import NodeMonitor


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
    
    @patch('subprocess.run')
    def test_get_ros2_nodes_success(self, mock_run):
        """Test getting ROS2 nodes when successful"""
        mock_run.return_value.returncode = 0
        mock_run.return_value.stdout = "/node1\n/node2\n/node3\n"
        
        nodes = get_ros2_nodes()
        self.assertEqual(nodes, ["/node1", "/node2", "/node3"])
    
    @patch('subprocess.run')
    def test_get_ros2_nodes_failure(self, mock_run):
        """Test getting ROS2 nodes when command fails"""
        mock_run.side_effect = FileNotFoundError()
        nodes = get_ros2_nodes()
        self.assertEqual(nodes, [])


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
        
        # Should not update nodes when ROS2 is not available
        result = monitor.update_nodes()
        self.assertFalse(result)


if __name__ == '__main__':
    unittest.main()
