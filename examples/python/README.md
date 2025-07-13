# ROS2Top Examples

This directory contains example code demonstrating how to integrate nodes with ros2top for monitoring.

## Python Examples

### example_node.py

A complete example of a ROS2 node that registers itself with ros2top for monitoring.

**Features:**

- Automatic registration with ros2top on startup
- Periodic heartbeat messages to maintain monitoring status
- Graceful unregistration on shutdown
- Example ROS2 publisher/subscriber functionality
- CPU usage simulation for demonstration

**Usage:**

1. **Prerequisites:**
   - ROS2 installed and sourced
   - ros2top installed (`pip install -e .` from the project root)

2. **Run the example:**

   ```bash
   # Navigate to the examples directory
   cd /home/radwan/ros2top/examples/python
   
   # Run the example node
   python3 example_node.py
   ```

3. **Monitor with ros2top:**

   ```bash
   # In another terminal, run ros2top to see the registered node
   ros2top
   ```

**What you'll see:**

- The node will appear in ros2top's monitoring interface
- CPU usage will be displayed (simulated work)
- Memory usage will be tracked
- Node uptime will be shown in DDd:HHh:MMm:SSs format
- Heartbeat status will be maintained

**Node Information:**

- **Name:** `example_monitored_node`
- **Published Topics:** `example_topic`
- **Subscribed Topics:** `example_input`
- **Update Rate:** 1 Hz (publishing), 0.2 Hz (heartbeat)

## Integration in Your Own Nodes

To add ros2top monitoring to your own ROS2 nodes:

1. **Import the registry functions:**

   ```python
   from ros2top.node_registry import register_node, unregister_node, heartbeat
   ```

2. **Register on startup:**

   ```python
   node_info = {
       'description': 'Your node description',
       'version': '1.0.0',
       'topics_published': ['topic1', 'topic2'],
       'topics_subscribed': ['input_topic'],
       'node_type': 'your_node_type'
   }
   register_node(node_name, node_info)
   ```

3. **Unregister on shutdown:**

   ```python
   unregister_node(node_name)
   ```

## Troubleshooting

- **ImportError for ros2top:** The node will run without monitoring if ros2top is not installed
- **Registration fails:** Check that ros2top is properly installed and the registry directory is writable
- **Node not appearing in ros2top:** Ensure the node is running and has successfully registered (check logs)
- **Heartbeat failures:** Verify file permissions and that the registry file is accessible
