#!/usr/bin/env python3
"""
Example ROS2 node that registers itself with ros2top for monitoring

This example demonstrates:
1. How to register a ROS2 node with ros2top
2. How to send periodic heartbeats
3. How to unregister on shutdown
4. Basic ROS2 node functionality with publishing and subscribing
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
import signal
import sys

# Import ros2top registration functions
try:
    from ros2top.node_registry import register_node, unregister_node, heartbeat
    ROS2TOP_AVAILABLE = True
except ImportError:
    print("ros2top not available - node will run without monitoring")
    ROS2TOP_AVAILABLE = False


class ExampleMonitoredNode(Node):
    """
    Example ROS2 node that registers itself with ros2top for monitoring
    """
    
    def __init__(self):
        super().__init__('example_monitored_node')
        
        # Register with ros2top if available
        self.ros2top_registered = False
        if ROS2TOP_AVAILABLE:
            self._register_with_ros2top()
        
        # Create publisher and subscriber for demonstration
        self.publisher = self.create_publisher(String, 'example_topic', 10)
        self.subscription = self.create_subscription(
            String,
            'example_input',
            self.listener_callback,
            10
        )
        
        # Create timer for periodic publishing and heartbeat
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.heartbeat_timer = self.create_timer(5.0, self.heartbeat_callback)
        
        # Message counter
        self.message_count = 0
        
        self.get_logger().info('Example monitored node started')
        self.get_logger().info(f'ros2top monitoring: {"enabled" if self.ros2top_registered else "disabled"}')
    
    def _register_with_ros2top(self):
        """Register this node with ros2top for monitoring"""
        try:
            node_info = {
                'description': 'Example ROS2 node demonstrating ros2top integration',
                'version': '1.0.0',
                'topics_published': ['example_topic'],
                'topics_subscribed': ['example_input'],
                'node_type': 'example_demo'
            }
            
            success = register_node(self.get_name(), node_info)
            if success:
                self.ros2top_registered = True
                self.get_logger().info('Successfully registered with ros2top')
            else:
                self.get_logger().warn('Failed to register with ros2top')
                
        except Exception as e:
            self.get_logger().error(f'Error registering with ros2top: {e}')
    
    def timer_callback(self):
        """Timer callback for periodic publishing"""
        # Create and publish a message
        msg = String()
        msg.data = f'Hello from monitored node! Message #{self.message_count}'
        self.publisher.publish(msg)
        
        self.get_logger().info(f'Published: {msg.data}')
        self.message_count += 1
        
        # Simulate some CPU work
        start_time = time.time()
        while time.time() - start_time < 0.1:  # 100ms of work
            _ = sum(i * i for i in range(1000))
    
    def heartbeat_callback(self):
        """Send heartbeat to ros2top if registered"""
        if self.ros2top_registered and ROS2TOP_AVAILABLE:
            try:
                success = heartbeat(self.get_name())
                if success:
                    self.get_logger().debug('Sent heartbeat to ros2top')
                else:
                    self.get_logger().warn('Failed to send heartbeat to ros2top')
            except Exception as e:
                self.get_logger().error(f'Error sending heartbeat: {e}')
    
    def listener_callback(self, msg):
        """Callback for subscription"""
        self.get_logger().info(f'Received: {msg.data}')
        
        # Simulate some processing work
        time.sleep(0.05)  # 50ms processing time
    
    def shutdown(self):
        """Clean shutdown - unregister from ros2top"""
        if self.ros2top_registered and ROS2TOP_AVAILABLE:
            try:
                success = unregister_node(self.get_name())
                if success:
                    self.get_logger().info('Successfully unregistered from ros2top')
                else:
                    self.get_logger().warn('Failed to unregister from ros2top')
            except Exception as e:
                self.get_logger().error(f'Error unregistering from ros2top: {e}')
        
        # Destroy the node
        self.destroy_node()


def signal_handler(signum, frame):
    """Handle shutdown signals gracefully"""
    print(f"\nReceived signal {signum}, shutting down...")
    rclpy.shutdown()


def main(args=None):
    """Main function"""
    # Initialize ROS2
    rclpy.init(args=args)
    
    # Set up signal handlers for graceful shutdown
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    # Create the node
    node = ExampleMonitoredNode()
    
    try:
        # Spin the node
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean shutdown
        node.shutdown()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
