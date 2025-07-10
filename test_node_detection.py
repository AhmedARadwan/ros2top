#!/usr/bin/env python3
"""
Test script to verify that ros2top can detect running ROS2 nodes and their PIDs
"""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'ros2top'))

from ros2top.ros2_utils import get_ros2_nodes_with_pids, get_ros2_nodes, is_tracing_available

def main():
    print("=== ROS2Top Node Detection Test ===")
    
    # Check if tracing is available
    tracing_available = is_tracing_available()
    print(f"Tracing available: {tracing_available}")
    
    # Get list of nodes
    nodes = get_ros2_nodes()
    print(f"Detected {len(nodes)} ROS2 nodes:")
    for node in nodes:
        print(f"  - {node}")
    
    if not nodes:
        print("No nodes detected. Make sure some ROS2 nodes are running.")
        return
    
    # Get nodes with PIDs using our tracing approach
    print("\nGetting nodes with PIDs using tracing approach...")
    nodes_with_pids = get_ros2_nodes_with_pids()
    
    print(f"\nSuccessfully detected {len(nodes_with_pids)} nodes with PIDs:")
    for node_name, pid in nodes_with_pids:
        print(f"  - {node_name}: PID {pid}")
    
    if len(nodes_with_pids) == len(nodes):
        print("\n✅ SUCCESS: All nodes were detected with PIDs!")
    else:
        print(f"\n⚠️  PARTIAL: {len(nodes_with_pids)}/{len(nodes)} nodes detected with PIDs")
        print("Some nodes may not have been traced or PID detection failed")

if __name__ == "__main__":
    main()
