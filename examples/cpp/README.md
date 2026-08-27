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

   - ROS 2 Humble, Jazzy, Kilted or Rolling
   - ros2top installed: `pip install ros2top`
   - Build tools: `sudo apt install build-essential cmake`
   - nlohmann_json: `sudo apt install nlohmann-json3-dev`

2. **Build the example**:

   ```bash
   # Create or navigate to your ROS2 workspace
   mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src

   # Copy or link the example package
   git clone https://github.com/AhmedARadwan/ros2top.git /tmp/ros2top
   ln -s /tmp/ros2top/examples/cpp/example_monitored_node .

   # Build. ros2top ships as a pip package, so its CMake config is not on
   # CMake's default search path unless it was installed system-wide --
   # `ros2top --cmake-dir` reports where it actually is.
   cd ~/ros2_ws
   colcon build --packages-select example_monitored_node \
       --cmake-args -Dros2top_DIR=$(ros2top --cmake-dir)
   source install/setup.bash
   ```

   If `find_package(ros2top)` fails, that flag is the fix. Alternatively put
   the prefix on `CMAKE_PREFIX_PATH` once:

   ```bash
   export CMAKE_PREFIX_PATH="$(dirname $(dirname $(ros2top --cmake-dir))):$CMAKE_PREFIX_PATH"
   ```

3. **Run the example**:

   ```bash
   ros2 run example_monitored_node example_monitored_node
   ```

4. **Monitor with ros2top**:

   ```bash
   ros2top

   # or, equivalently, as a ros2 CLI sub-command
   ros2 top
   ```

## C++ API Overview

The API is header-only. Install ros2top, then include it:

```cpp
#include <ros2top/ros2top.hpp>
```

`ros2top --include-dir` prints the include path if you are driving the compiler
yourself rather than through CMake.

### Core Functions

```cpp
namespace ros2top {
    // Register a node. additional_info is any JSON object; values may be
    // strings, numbers or arrays. Returns false if the registry cannot be
    // written -- it never throws.
    bool register_node(const std::string& node_name,
                       const nlohmann::json& additional_info = {});

    // Refresh the node's last_seen timestamp. False means the node is not
    // registered, or the registry could not be written.
    bool heartbeat(const std::string& node_name);

    // Remove the node from the registry. Unregistering a node that was never
    // registered succeeds: the requested state already holds.
    bool unregister_node(const std::string& node_name);
}
```

All three are safe to call from any thread and never throw: writers in both
languages exclude each other with a lock file, and a lock left behind by a
crashed process is detected and cleared rather than blocking forever.

### RAII registration

`AutoNodeRegistrar` registers on construction and unregisters on destruction,
so a node cannot leak a registration by returning early or throwing:

```cpp
class MyNode : public rclcpp::Node {
public:
    MyNode() : Node("my_node"), registrar_(this->get_name(), {{"version", "1.0.0"}}) {}
private:
    ros2top::AutoNodeRegistrar registrar_;
};
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
- [ ] `find_package(ros2top REQUIRED)` in `CMakeLists.txt`
- [ ] `target_link_libraries(<target> ros2top::ros2top)` - this carries the
      include path, the C++17 requirement and nlohmann_json with it
- [ ] Include `ros2top/ros2top.hpp` in your source
- [ ] Register node in constructor with metadata
- [ ] Set up periodic heartbeat timer
- [ ] Unregister in destructor or shutdown handler
- [ ] Handle registration failures gracefully

## Troubleshooting

**Build Issues:**

- `Could not find a package configuration file provided by "ros2top"` - pass
  `--cmake-args -Dros2top_DIR=$(ros2top --cmake-dir)`, as above.
- `nlohmann/json.hpp: No such file or directory` - `sudo apt install nlohmann-json3-dev`
- Verify the ROS 2 environment is sourced

**Runtime Issues:**

Run `ros2top --doctor`. It reports every registry entry, whether the process
behind it is alive, and why an entry would not be listed - which covers the
usual causes: a registry directory owned by root, nodes in a different PID
namespace (a container without `--pid=host`), or a middleware that does not
expose PIDs.

**Integration Issues:**

- Review example code for proper API usage
- Check that heartbeat timer is running
- Ensure graceful shutdown calls unregister

## Related Documentation

- [Python examples](../python/README.md)
- [ros2top main documentation](../../README.md)
- [C++ API header](../../include/ros2top/ros2top.hpp)
