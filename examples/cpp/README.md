# C++ Examples for ros2top

This directory contains C++ examples demonstrating how to integrate ROS2 nodes with ros2top for monitoring.

## Available Examples

### example_monitored_node

A complete ROS2 C++ package showing how to:

- Register a C++ node with ros2top
- Send periodic heartbeats
- Handle graceful shutdown and unregistration
- Integrate with the ros2top C++ API

See the [package README](example_monitored_node/README.md) for detailed usage instructions.

## Quick Start

1. **Prerequisites**:

   - ROS2 (Humble, Iron, or Rolling)
   - ros2top installed: `pip install -e /home/radwan/ros2top`
   - Build tools: `sudo apt install build-essential cmake`
   - nlohmann_json: `sudo apt install nlohmann-json3-dev`

2. **Build the example**:

   ```bash
   # Create or navigate to your ROS2 workspace
   mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src

   # Copy or link the example package
   ln -s /home/radwan/ros2top/examples/cpp/example_monitored_node .

   # Build
   cd ~/ros2_ws
   colcon build --packages-select example_monitored_node
   source install/setup.bash
   ```

3. **Run the example**:

   ```bash
   ros2 run example_monitored_node example_monitored_node
   ```

4. **Monitor with ros2top**:

   ```bash
   ros2top
   ```

## C++ API Overview

The ros2top C++ API is defined in `/home/radwan/ros2top/include/ros2top/ros2top.hpp` and provides:

### Core Functions

```cpp
namespace ros2top {
    // Register a node with monitoring information
    bool register_node(const std::string& node_name, const nlohmann::json& node_info);

    // Send a heartbeat to maintain registration
    bool heartbeat(const std::string& node_name);

    // Unregister a node from monitoring
    bool unregister_node(const std::string& node_name);

    // Get information about registered nodes
    nlohmann::json get_registered_nodes();

    // Check if a specific node is registered
    bool is_node_registered(const std::string& node_name);
}
```

### Usage Pattern

```cpp
#include "ros2top/ros2top.hpp"

class MyNode : public rclcpp::Node {
private:
    bool ros2top_registered_ = false;

    void register_with_ros2top() {
        nlohmann::json node_info;
        node_info["description"] = "My awesome node";
        node_info["version"] = "1.0.0";
        node_info["topics_published"] = nlohmann::json::array({"output_topic"});
        node_info["topics_subscribed"] = nlohmann::json::array({"input_topic"});
        node_info["node_type"] = "sensor_processor";

        ros2top_registered_ = ros2top::register_node(this->get_name(), node_info);
    }

    void heartbeat_callback() {
        if (ros2top_registered_) {
            ros2top::heartbeat(this->get_name());
        }
    }

    ~MyNode() {
        if (ros2top_registered_) {
            ros2top::unregister_node(this->get_name());
        }
    }
};
```

## Integration Checklist

When adding ros2top to your C++ ROS2 package:

- [ ] Add `nlohmann_json` dependency to `package.xml`
- [ ] Include ros2top headers in `CMakeLists.txt`
- [ ] Link `nlohmann_json` in `CMakeLists.txt`
- [ ] Include `ros2top/ros2top.hpp` in your source
- [ ] Register node in constructor with metadata
- [ ] Set up periodic heartbeat timer
- [ ] Unregister in destructor or shutdown handler
- [ ] Handle registration failures gracefully

## Troubleshooting

**Build Issues:**

- Ensure `nlohmann-json3-dev` is installed
- Check that ros2top include path is correct
- Verify ROS2 environment is sourced

**Runtime Issues:**

- Check file permissions for registry directory
- Ensure ros2top Python package is installed
- Verify node name doesn't conflict with existing registrations

**Integration Issues:**

- Review example code for proper API usage
- Check that heartbeat timer is running
- Ensure graceful shutdown calls unregister

## Related Documentation

- [Python examples](../python/README.md)
- [ros2top main documentation](../../README.md)
- [C++ API header](../../include/ros2top/ros2top.hpp)
