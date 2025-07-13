# C++ ROS2 Node Example with ros2top Integration

This directory contains a complete ROS2 C++ package demonstrating how to integrate nodes with ros2top for monitoring.

## Package: example_monitored_node

A complete example of a ROS2 C++ node that registers itself with ros2top for monitoring.

### Features

- Automatic registration with ros2top on startup using C++ API
- Periodic heartbeat messages to maintain monitoring status
- Graceful unregistration on shutdown
- Example ROS2 publisher/subscriber functionality
- CPU usage simulation for demonstration
- Proper ROS2 package structure with CMake and package.xml

### Prerequisites

1. **ROS2 installed and sourced**
2. **ros2top installed** (`pip install -e .` from the project root)
3. **Build tools**: `sudo apt install build-essential cmake`
4. \*\*

### Building the Package

1. **Navigate to your ROS2 workspace** (or create one):

   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
   ```

2. **Copy the package**:

   ```bash
   cp -r /home/radwan/ros2top/examples/cpp/example_monitored_node .
   ```

3. **Build the package**:

   ```bash
   cd ~/ros2_ws
   colcon build --packages-select example_monitored_node
   ```

### Running the Example

1. **Source your ROS2 workspace**:

   ```bash
   source ~/ros2_ws/install/setup.bash
   ```

2. **Run the node directly**:

   ```bash
   ros2 run example_monitored_node example_monitored_node
   ```

   **Or use the launch file**:

   ```bash
   ros2 launch example_monitored_node example_monitored_node.launch.py
   ```

3. **Monitor with ros2top**:

   ```bash
   # In another terminal, run ros2top to see the registered node
   ros2top
   ```

### What You'll See

- The C++ node will appear in ros2top's monitoring interface
- CPU usage will be displayed (simulated work)
- Memory usage will be tracked
- Node uptime will be shown in DDd:HHh:MMm:SSs format
- Heartbeat status will be maintained

### Node Information

- **Package Name:** `example_monitored_node`
- **Node Name:** `example_monitored_node_cpp`
- **Published Topics:** `example_topic_cpp`
- **Subscribed Topics:** `example_input_cpp`

## Integration in Your Own C++ Nodes

To add ros2top monitoring to your own ROS2 C++ nodes:

1. **Include the ros2top header**:

   ```cpp
   #include "ros2top/ros2top.hpp"
   ```

2. **Add to your CMakeLists.txt**:

   ```cmake
   find_package(ros2top REQUIRED)
   include_directories(${ros2top_INCLUDE_DIRS})
   ```

3. **Register on startup**:

   ```cpp
   nlohmann::json node_info;
   node_info["description"] = "Your node description";
   node_info["version"] = "1.0.0";
   node_info["topics_published"] = nlohmann::json::array({"topic1", "topic2"});
   node_info["topics_subscribed"] = nlohmann::json::array({"input_topic"});
   node_info["node_type"] = "your_node_type";

   bool success = ros2top::register_node(node_name, node_info);
   ```

4. **Send periodic heartbeats**:

   ```cpp
   // In a timer callback or periodic function
   bool success = ros2top::heartbeat(node_name);
   ```

5. **Unregister on shutdown**:

   ```cpp
   bool success = ros2top::unregister_node(node_name);
   ```

## Package Structure

```text
example_monitored_node/
├── CMakeLists.txt          # Build configuration
├── package.xml             # Package metadata
├── src/
│   └── example_monitored_node.cpp  # Main node implementation
└── launch/
    └── example_monitored_node.launch.py  # Launch file
```

## Troubleshooting

- **ros2top registration fails**: Check that ros2top is installed and registry directory is writable
- **Node not appearing in ros2top**: Verify the node is running and has successfully registered (check logs)
- **Include errors**: Make sure the ros2top include path is correctly set in CMakeLists.txt
