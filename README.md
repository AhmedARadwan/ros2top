# ROS2Top

A real-time monitor for ROS2 nodes showing CPU, RAM, and GPU usage - like `htop` but for ROS2 nodes.

<!-- ![ROS2Top Demo]() -->

## Features

- 🔍 **Automatic node discovery** — scans running processes for colcon install-tree signatures (no code changes required in monitored nodes)
- 💻 **CPU usage** tracking per node
- 🧠 **RAM usage** monitoring
- 🎮 **GPU usage** tracking (NVIDIA GPUs via NVML; Jetson/Tegra via sysfs)
- 🖥️ **Terminal-based interface** using curses
- 🔄 **Auto-refresh** with configurable intervals
- 🏷️ **Process tree awareness** (includes child processes)

## Installation

### From PyPI (when published)

```bash
pip install ros2top
```

### From Source

```bash
git clone https://github.com/kiwicampus/ros2top-auto.git
cd ros2top
pip install -e .
```

## Requirements

- Python 3.8+
- NVIDIA drivers (for GPU monitoring)

### Python Dependencies

- `psutil>=5.8.0`
- `pynvml>=11.0.0`

### CPP Dependencies

- [nlohmann json](https://github.com/nlohmann/json) installed from source.

## Usage

### Examples

- **[Python Example](examples/python/README.md)**: Complete ROS2 Python node with ros2top integration
- **[C++ Example](examples/cpp/README.md)**: Complete ROS2 C++ package with ros2top integration

### Basic Usage

```bash
# Run ros2top
ros2top
```

### Command Line Options

```bash
ros2top --help                # Show help
ros2top --refresh 2          # Refresh every 2 seconds (default: 5)
ros2top --version           # Show version
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
```

## How It Works

1. **Auto-Discovery**: Scans running processes with `psutil`, keeping those whose executable path is inside a colcon `install/` tree (Python nodes under `ros2/` or `rover/`, C++ nodes under `lib/`, component containers). No registration required.
2. **Resource Monitoring**: Uses `psutil` for CPU/RAM and `pynvml` (or Jetson sysfs) for GPU metrics.
3. **Display**: Curses-based terminal interface for real-time updates.

## Troubleshooting

### No GPU monitoring

- Install NVIDIA drivers
- Install pynvml: `pip install pynvml`

### Nodes not showing up

- Verify nodes are running: `ros2 node list`
- Check node info: `ros2 node info /your_node`
- Some nodes might not have detectable PIDs

## Development

### Setup Development Environment

```bash
git clone https://github.com/kiwicampus/ros2top-auto.git
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
│   ├── discovery.py        # Automatic node discovery via psutil
│   ├── node_monitor.py     # Core monitoring logic
│   ├── gpu_monitor.py      # GPU monitoring (NVML + Jetson sysfs)
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

## Node Detection

`ros2top` uses **automatic process-based discovery** — no code changes are required in your nodes.

### How Discovery Works

`discovery.py` inspects all running processes via `psutil` and identifies ROS2 nodes by their executable path:

- **Python nodes**: executable inside `install/.../ros2/` or `install/.../rover/`
- **C++ nodes**: executable inside `install/.../lib/`
- **Component containers**: `component_container_isolated` processes (node name extracted from args)

### Nodes Not Showing Up?

- Confirm nodes were built with `colcon build` (the discovery looks for `install/` in the path)
- Nodes launched via `ros2 run` from a colcon workspace are detected automatically
- Use `ros2top --refresh 1` to poll more frequently
