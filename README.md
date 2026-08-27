# ROS2Top

A real-time monitor for ROS2 nodes showing CPU, RAM, and GPU usage - like `htop` but for ROS2 nodes.

Tested on **Humble**, **Jazzy**, **Kilted** and **Rolling**.

<!-- ![ROS2Top Demo]() -->

## Features

- 🔍 **Real-time monitoring** of all ROS2 nodes
- 💻 **CPU usage** tracking per node
- 🧠 **RAM usage** monitoring
- 🎮 **GPU usage** tracking (NVIDIA GPUs via NVML)
- 🖥️ **Terminal-based interface** using curses
- 🔄 **Auto-refresh** with configurable intervals
- 🏷️ **Process tree awareness** (includes child processes)
- 🔌 **`ros2 top` integration** - available as a ros2 CLI sub-command
- 🛰️ **Automatic node discovery** from the ROS graph, with no code changes (on supported middleware)
- 📦 **Component container aware** - composable nodes are grouped under the container hosting them
- 📝 **Node registration API** for reliable node-to-monitor communication

## Supported ROS 2 distributions

Verified in the official `ros:<distro>` containers: installed with `pip`, then
checked for the full unit suite, the `ros2 top` sub-command, and live
auto-discovery of C++, Python and composable nodes against their real PIDs.

| Distribution | Ubuntu | Default RMW | Auto-discovery | `ros2 top` | Status |
| ------------ | ------ | ----------- | -------------- | ---------- | ------ |
| **Humble** Hawksbill | 22.04 | `rmw_fastrtps_cpp` | ✅ | ✅ | Supported |
| **Jazzy** Jalisco | 24.04 | `rmw_fastrtps_cpp` | ✅ | ✅ | Supported |
| **Kilted** Kaiju | 24.04 | `rmw_fastrtps_cpp` | ✅ | ✅ | Supported |
| **Rolling** Ridley | 24.04 | `rmw_fastrtps_cpp` | ✅ | ✅ | Supported |

Iron Irwini is not tested; it reached end of life in November 2024.

Auto-discovery depends on the **middleware**, not the distribution. All four
default to Fast DDS, whose GUID carries the process id. Under `rmw_zenoh_cpp` or
Cyclone DDS the graph carries no PID, so nodes must register themselves - ros2top
detects this and says so in the status bar rather than showing an empty table.

## Installation

```bash
pip install ros2top
```

On **Ubuntu 24.04 and newer** (Jazzy, Kilted, Rolling) the system interpreter is
marked externally managed (PEP 668), so pip needs to be told explicitly:

```bash
pip install --break-system-packages ros2top
# or, preferred, a venv that can still see the ROS 2 packages:
python3 -m venv --system-site-packages ~/.venvs/ros2top
source ~/.venvs/ros2top/bin/activate && pip install ros2top
```

Humble (Ubuntu 22.04) needs no flag.

### From Source

```bash
git clone https://github.com/AhmedARadwan/ros2top.git
cd ros2top
pip install -e .
```

> **Note:** installing *from source* needs pip 23 or newer to read this
> project's `pyproject.toml` metadata. The pip 22.0.2 that ships with Ubuntu
> 22.04 installs an empty `UNKNOWN-0.0.0` package instead, and reports success.
> Run `pip install --upgrade pip` first, or install the released wheel with
> `pip install ros2top`, which works on every version tested.

## Requirements

- Python 3.8+
- NVIDIA drivers (for GPU monitoring)

### Python Dependencies

- `psutil>=5.8.0`
- `pynvml>=11.0.0`

### C++ Dependencies

- [nlohmann/json](https://github.com/nlohmann/json) - `sudo apt install nlohmann-json3-dev`

ros2top ships as a Python wheel, so its CMake config is installed wherever pip
put it, which is not usually on CMake's search path. `ros2top --cmake-dir`
reports the location:

```bash
colcon build --cmake-args -Dros2top_DIR=$(ros2top --cmake-dir)
```

See the [C++ example](examples/cpp/README.md) for a complete package.

## Usage

### Examples

- **[Python Example](examples/python/README.md)**: Complete ROS2 Python node with ros2top integration
- **[C++ Example](examples/cpp/README.md)**: Complete ROS2 C++ package with ros2top integration

### Basic Usage

```bash
# Run standalone
ros2top

# Or as a ros2 CLI sub-command, wherever ROS 2 is installed
ros2 top
```

Both are the same program: `ros2top` registers itself with `ros2cli` through an
entry point, so `ros2 top` appears in `ros2 --help` with no changes to ros2cli.
Every option below works with either form.

### Command Line Options

```bash
ros2top --help                # Show help
ros2top --refresh 2           # Refresh every 2 seconds (default: 0.1)
ros2top --no-auto-discovery   # Only show nodes that registered themselves
ros2top --doctor              # Diagnose why nodes are or aren't showing up
ros2top --cmake-dir           # Print the path find_package(ros2top) needs
ros2top --include-dir         # Print the C++ include path
ros2top --version             # Show version

ros2 top --refresh 2          # identical, via the ros2 CLI
```

If `ros2top` is not on your PATH after installing (`pip install --user` puts it
in `~/.local/bin`, which some shells do not add), run it as a module instead:

```bash
python3 -m ros2top
```

### Interactive Controls

The enhanced terminal UI provides responsive and interactive controls:

| Key        | Action                        |
| ---------- | ----------------------------- |
| `q` or `Q` | Quit application              |
| `h` or `H` | Show help dialog              |
| `r` or `R` | Force refresh node list       |
| `p` or `P` | Pause/resume monitoring       |
| `+` or `=` | Increase refresh rate         |
| `-`        | Decrease refresh rate         |
| `↑` / `↓`  | Navigate through nodes        |
| `Tab`      | Cycle focus between UI panels |
| `Space`    | Force immediate update        |
| `Home/End` | Jump to first/last node       |

## Terminal UI

### Visual Features

- **Color-coded usage bars**: Green (low), Yellow (medium), Red (high)
- **Real-time progress bars** for CPU, memory, and GPU
- **Interactive navigation** with keyboard shortcuts
- **Adaptive refresh rates** for optimal performance

### System Overview Panel

The top panel shows real-time system information:

- CPU usage (per-core or summary based on terminal size)
- Memory usage with progress bar
- GPU utilization and memory (if available)
- ROS2 status and active node count

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
# or
ros2 top --refresh 2
```

## How It Works

1. **Node discovery**: ros2top finds nodes two ways - automatically from the ROS
   graph, and from nodes that register themselves (see below).
2. **Resource Monitoring**: Uses `psutil` for CPU/RAM and `pynvml` for GPU metrics.
3. **Display**: Curses-based terminal interface for real-time updates.

### Node discovery

To show a node's resource usage, ros2top needs its **PID**, and the ROS graph
does not publish that. Two mechanisms fill the gap:

**Automatic (no code changes).** On middleware whose DDS GUID encodes the
process id - Fast DDS, the ROS 2 default - ros2top recovers the node-to-PID
mapping from the graph alone. Auto-discovered nodes are shown with a `~` prefix,
because their PID is *inferred* rather than reported.

**Registration (always works).** A node that calls `register_node()` states its
PID directly. This is required on middleware that does not carry the PID, such
as `rmw_zenoh_cpp` and Cyclone DDS, and it is more precise everywhere: registered
nodes report their own start time and can attach custom metadata.

The status bar shows the middleware in use and whether auto-discovery is
working:

```text
ROS2✓ | RMW:fastrtps | Auto✓ | Nodes:4      # discovered automatically
ROS2✓ | RMW:zenoh | Auto✗ register | Nodes:0  # registration required
```

If no nodes appear, ros2top explains why in the table area rather than showing
an empty list. Use `--no-auto-discovery` to ignore the graph and show only
registered nodes.

### Composable nodes

Nodes loaded into a component container all run in **one process**, so their
CPU, RAM and GPU usage cannot be separated - the numbers belong to the process,
not to any one node. ros2top shows the container heading its group with the
usage figures listed once, and the nodes it hosts indented beneath it:

```text
PID      Uptime  %CPU  RAM(MB)  GPU#  %GPU  GMEM(MB)  Node Name
3744993  10s     0.0   30.1     --    --    --        /my_container  (+2 nodes)
         06s                                            - /talker
         02s                                            - /listener
3742958  02s     0.0   27.0     --    --    --        /standalone_talker
```

Uptime stays per node, since a node composed into an already-running container
is younger than the process hosting it. Killing any node in a group ends the
whole process, taking every node in it - the kill dialog warns before it does.

## Troubleshooting

### Start here: `ros2top --doctor`

It prints what ros2top can and cannot see - the middleware in use, whether
auto-discovery works, every registry entry with the liveness of the process
behind it, the C++ integration paths, and GPU status. Most "no nodes appear"
reports are answered by it directly, and it is designed to be pasted into a bug
report.

### `ros2top: command not found` after installing

The console script is installed next to your Python, and that directory is not
always on PATH - `pip install --user` uses `~/.local/bin`. Either add it:

```bash
export PATH="$HOME/.local/bin:$PATH"
```

or skip PATH entirely with `python3 -m ros2top`.

### `ModuleNotFoundError` from a path inside a colcon workspace

If the traceback names a file under `<workspace>/install/ros2top/...`, a stale
colcon install of ros2top is shadowing the pip package, and it is that copy -
not the installed one - that is failing to import. Remove it and re-source:

```bash
rm -rf <workspace>/install/ros2top <workspace>/build/ros2top
source /opt/ros/$ROS_DISTRO/setup.bash
```

### `find_package(ros2top)` fails in colcon

CMake only searches a few system prefixes, and pip rarely installs into one of
them. Point CMake at the real location:

```bash
colcon build --cmake-args -Dros2top_DIR=$(ros2top --cmake-dir)
```

### No GPU monitoring

- Install NVIDIA drivers
- Install pynvml: `pip install pynvml`
- On Jetson this is expected: NVML does not support the integrated GPU, so the
  GPU columns read `--`. See [#2](https://github.com/AhmedARadwan/ros2top/issues/2).

### `pip install` succeeded but `ros2top` is missing

If you installed from source on Ubuntu 22.04, check what pip actually installed:

```bash
pip show ros2top      # 'UNKNOWN 0.0.0' means pip was too old
pip install --upgrade pip && pip install -e .
```

### Nodes not showing up

First check the status bar. It names the middleware in use and whether
auto-discovery is working.

- `Auto✗ register` - your middleware does not expose node PIDs (Zenoh, Cyclone).
  Nodes **must** call `register_node()` to appear. See the
  [Python](examples/python/README.md) and [C++](examples/cpp/README.md) examples.
- `Auto✓` but a node is missing - verify it is running with `ros2 node list`.
  Nodes on other machines are skipped on purpose: their PID is meaningless
  locally. A node is also skipped when its PID cannot be pinned down
  unambiguously, since showing the wrong process is worse than showing none.
- No status at all / `rclpy is not importable` - ROS 2 is not sourced, so the
  graph cannot be read. Source your installation, or use registration.

## Development

### Setup Development Environment

```bash
git clone https://github.com/AhmedARadwan/ros2top.git
cd ros2top
pip install -e .
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

```text
ros2top/
├── ros2top/                 # Python package
│   ├── __init__.py         # Package initialization and public API
│   ├── main.py             # CLI entry point
│   ├── node_monitor.py     # Core monitoring logic
│   ├── node_registry.py    # Node registration system
│   ├── graph_discovery.py  # Automatic node discovery from the ROS graph
│   ├── command/            # ros2cli plugin exposing `ros2 top`
│   │   └── top.py
│   ├── gpu_monitor.py      # GPU monitoring
│   ├── ros2_utils.py       # ROS2 utilities
│   └── ui/                 # User interface components
│       ├── __init__.py
│       ├── terminal_ui.py  # Main curses interface
│       ├── components.py   # UI components
│       └── layout.py       # UI layout management
├── include/                # C++ headers
│   └── ros2top/
│       └── ros2top.hpp     # C++ API for node registration
├── examples/               # Example integrations
│   ├── python/             # Python examples
│   │   ├── README.md
│   │   └── example_node.py
│   └── cpp/                # C++ examples
│       ├── README.md
│       └── example_monitored_node/  # Complete ROS2 package
├── tests/                  # Test suite
│   ├── __init__.py
│   └── test_ros2top.py
├── cmake/                  # CMake configuration
├── pyproject.toml          # Python build configuration
├── requirements.txt        # Python dependencies
├── LICENSE                 # MIT license
└── README.md              # This file
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

### v0.1.3

- Remove dependency on ROS2 to start ros2top.

### v0.1.2

- Enhance README

### v0.1.1

- Add example usage
- Enhance README

### v0.1.0

- Initial release
- Basic node monitoring with CPU, RAM, GPU usage
- Terminal interface with curses
- Command line options
- Node registration and process mapping

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
