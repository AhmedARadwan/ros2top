# ROS2Top

A real-time monitor for ROS2 nodes showing CPU, RAM, and GPU usage - like `htop` but for ROS2 nodes.

<!-- ![ROS2Top Demo]() -->

## Features

- 🔍 **Real-time monitoring** of all ROS2 nodes
- 💻 **CPU usage** tracking per node
- 🧠 **RAM usage** monitoring
- 🎮 **GPU usage** tracking (NVIDIA GPUs via NVML)
- 🖥️ **Terminal-based interface** using curses
- 🔄 **Auto-refresh** with configurable intervals
- 🏷️ **Process tree awareness** (includes child processes)
- 🕵️ **Advanced node detection** using ROS2 tracing for reliable C++ node discovery
- 🧵 **Background tracing** on separate thread for responsive UI
- 📝 **Node registration API** for reliable node-to-monitor communication

## Installation

### From PyPI (when published)

```bash
pip install ros2top
```

### From Source

```bash
git clone https://github.com/AhmedARadwan/ros2top.git
cd ros2top
pip install -e .
```

## Requirements

- Python 3.8+
- ROS2 (any distribution)
- NVIDIA drivers (for GPU monitoring)

### Python Dependencies

- `psutil>=5.8.0`
- `pynvml>=11.0.0`

## Usage

### Basic Usage

```bash
# Make sure ROS2 is sourced
source /opt/ros/humble/setup.bash  # or your ROS2 distro

# Run ros2top
ros2top
```

### Command Line Options

```bash
ros2top --help                # Show help
ros2top --refresh 2          # Refresh every 2 seconds (default: 5)
ros2top --no-gpu            # Disable GPU monitoring
ros2top --version           # Show version
```

### Interactive Controls

| Key        | Action                  |
| ---------- | ----------------------- |
| `q` or `Q` | Quit                    |
| `r` or `R` | Force refresh node list |
| `h` or `H` | Show help dialog        |

## Display Columns

| Column      | Description                                     |
| ----------- | ----------------------------------------------- |
| **Node**    | ROS2 node name                                  |
| **PID**     | Process ID                                      |
| **%CPU**    | CPU usage percentage (normalized by core count) |
| **RAM(MB)** | RAM usage in megabytes                          |
| **GPU#**    | GPU device number (if using GPU)                |
| **GPU%**    | GPU utilization percentage                      |
| **GMEM**    | GPU memory usage in MB                          |

## Examples

### Monitor nodes with 2-second refresh

```bash
ros2top --refresh 2
```

### Run without GPU monitoring

```bash
ros2top --no-gpu
```

### Typical workflow

```bash
# Terminal 1: Start your ROS2 nodes
ros2 launch my_package my_launch.py

# Terminal 2: Monitor with ros2top
source /opt/ros/humble/setup.bash
ros2top
```

## How It Works

1. **Node Discovery**: Uses `ros2 node list` to find active nodes
2. **Process Mapping**: Maps node names to system processes using `ros2 node info` and process matching
3. **Resource Monitoring**: Uses `psutil` for CPU/RAM and `pynvml` for GPU metrics
4. **Display**: Curses-based terminal interface for real-time updates

## Troubleshooting

### "ROS2 not available" message

Make sure ROS2 is properly sourced:

```bash
source /opt/ros/<your-distro>/setup.bash
# or for workspace
source ~/ros2_ws/install/setup.bash
```

### No GPU monitoring

- Install NVIDIA drivers
- Install pynvml: `pip install pynvml`
- Use `--no-gpu` flag to disable GPU monitoring

### Nodes not showing up

- Verify nodes are running: `ros2 node list`
- Check node info: `ros2 node info /your_node`
- Some nodes might not have detectable PIDs

### Permission errors

Run with appropriate permissions or adjust system settings for process monitoring.

## Development

### Setup Development Environment

```bash
git clone https://github.com/AhmedARadwan/ros2top.git
cd ros2top
pip install -e .
pip install -r requirements.txt
```

### Running Tests

```bash
python -m pytest tests/
```

### Code Style

```bash
black ros2top/
flake8 ros2top/
mypy ros2top/
```

## Architecture

```
ros2top/
├── ros2top/
│   ├── main.py          # CLI entry point
│   ├── node_monitor.py  # Core monitoring logic
│   ├── gpu_monitor.py   # GPU monitoring
│   ├── terminal_ui.py   # Curses interface
│   ├── node_registry.py # Node registration system
│   └── ros2_utils.py    # Simplified ROS2 utilities
├── setup.py             # Package setup
└── README.md           # This file
```

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Add tests
5. Submit a pull request

## License

MIT License - see [LICENSE](LICENSE) file for details.

## Changelog

### v0.1.0

- Initial release
- Basic node monitoring with CPU, RAM, GPU usage
- Terminal interface with curses
- Command line options
- ROS2 node discovery and process mapping

## Similar Tools

- `htop` - System process monitor
- `nvtop` - GPU process monitor
- `ros2 node list` - Basic ROS2 node listing

## Acknowledgments

- Inspired by `htop` and `nvtop`
- Built for the ROS2 community
- Uses `psutil` for system monitoring and `pynvml` for GPU monitoring

## Node Registration API

For the most reliable monitoring, ROS2 nodes can register themselves with `ros2top`. This is especially useful for:

- Multiple nodes running in the same Python process
- Complex applications where automatic detection might miss some nodes
- Getting additional metadata about nodes

### Basic Registration

```python
import ros2top

# Register your node (call this once when your node starts)
ros2top.register_node('/my_node_name')

# Send periodic heartbeats (optional, but recommended)
ros2top.heartbeat('/my_node_name')

# Unregister when shutting down (optional, automatic cleanup on process exit)
ros2top.unregister_node('/my_node_name')
```

### Advanced Registration with Metadata

```python
import ros2top

# Register with additional information
ros2top.register_node('/camera_processor', {
    'description': 'Processes camera feed for object detection',
    'type': 'vision_processor',
    'input_topics': ['/camera/image_raw'],
    'output_topics': ['/detected_objects'],
    'framerate': 30
})

# In your main loop, send heartbeats every few seconds
ros2top.heartbeat('/camera_processor')
```

### Example: Multiple Nodes in One Process

```python
import ros2top
import time

def main():
    # Register multiple nodes running in this process
    ros2top.register_node('/human_tracker', {'type': 'detector'})
    ros2top.register_node('/video_publisher', {'type': 'publisher'})

    try:
        while True:
            # Your processing logic here
            do_object_detection()

            # Send heartbeats every 5 seconds
            ros2top.heartbeat('/human_tracker')
            ros2top.heartbeat('/video_publisher')

            time.sleep(5)

    finally:
        # Cleanup (optional - happens automatically on exit)
        ros2top.unregister_node('/human_tracker')
        ros2top.unregister_node('/video_publisher')
```

### Benefits of Registration

- **Guaranteed detection**: Registered nodes will always be detected by `ros2top`
- **Immediate visibility**: No waiting for tracing or process scanning
- **Rich metadata**: Include custom information about your nodes
- **Multi-process support**: Perfect for complex applications
- **Heartbeat monitoring**: Detect unresponsive nodes even if process is running

## Node Detection

`ros2top` uses a **node registration system** for reliable node detection:

### Primary Method: Node Registration API

The most reliable way is for ROS2 nodes to explicitly register themselves:

```python
import ros2top

# Register your node
ros2top.register_node('/my_node', {'description': 'My awesome node'})

# Send periodic heartbeats (recommended)
ros2top.heartbeat('/my_node')

# Unregister when shutting down (optional - automatic cleanup on exit)
ros2top.unregister_node('/my_node')
```

### Automatic Cleanup

- Nodes are automatically unregistered when the process exits
- Stale registrations are cleaned up periodically
- Registry is stored in `~/.ros2top/registry/`

### Benefits of Registration API

- **Reliable**: No dependency on tracing or process matching
- **Fast**: Instant node detection without scanning
- **Accurate**: Direct PID mapping from the registering process
- **Simple**: Works with any ROS2 node type (Python, C++, etc.)
