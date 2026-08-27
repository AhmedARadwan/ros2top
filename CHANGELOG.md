# Changelog

All notable changes to this project are documented here.

The format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/).

## [0.5.0] - unreleased

The release that makes the C++ side work. Every reported C++ problem traced back
to the same handful of defects, and none of them could have been caught by the
test suite as it stood, because nothing in CI compiled C++ at all.

### Added

- **`ros2top --doctor`.** Reports the middleware in use, self-tests
  auto-discovery, lists every registry entry with the liveness of the process
  behind it and whether it will be listed, resolves the C++ integration paths,
  and checks GPU availability. Plain text, meant to be pasted into an issue.
  Every "ros2top shows nothing" report so far has come down to something
  invisible from the table: a middleware that carries no PID, nodes in another
  PID namespace, a registry directory owned by root, a stale lock, or ROS 2 not
  sourced.
- **`ros2top --cmake-dir` and `--include-dir`.** Print where the CMake config
  and the C++ header actually are, so `find_package(ros2top)` can be pointed at
  them: `colcon build --cmake-args -Dros2top_DIR=$(ros2top --cmake-dir)`.
  ([#9])
- **`python -m ros2top`.** Works when the console script is not on `PATH`, as
  after `pip install --user`. ([#5])
- **A logo, and a terminal demo recorded from a real session** - a component
  container hosting two composable nodes, registered C++ and Python nodes, and
  three nodes found from the graph alone, captured from the real UI in the
  official `ros:humble` container. ([#8])
- **CI.** Builds *and runs* the C++ example in the official `ros:humble`,
  `jazzy`, `kilted` and `rolling` containers, asserting on the registry it
  produces. Unit tests run on Python 3.8 through 3.13.

### Fixed

- **C++ nodes registered but never appeared.** `register_node()` took a
  `std::map<std::string, std::string>`, but the example and the C++ API
  documentation both pass a `nlohmann::json` containing arrays. The implicit
  conversion throws `type_error.302`, so registration was abandoned and the
  node only ever appeared via auto-discovery, if at all. `additional_info` is
  now any JSON object, as it always was in Python. ([#6])
- **The C++ example could not compile.** `heartbeat()` and `unregister_node()`
  returned `void` while the example assigned them to `bool`
  (`void value not ignored as it ought to be`). Both now return whether the
  registry was updated. ([#10])
- **`process_name` was garbage.** It was read with `std::getline()` from
  `/proc/self/cmdline`, which is NUL-separated with no trailing newline, so the
  whole command line came back as one string and the executable name was taken
  from the last argument - a node launched with `--params-file` recorded
  `launch_params_mu_ch5t_\0`. `cmdline` was always empty. Both now correct.
- **A crashed node blocked all later registrations.** The registry lock is a
  plain file and nothing checked its owner, so a process killed while holding it
  left it behind permanently, failing every subsequent registration in either
  language until it was deleted by hand. A lock owned by a dead PID is now
  cleared. The lock is also created with `O_CREAT|O_EXCL`, so two nodes starting
  together cannot both believe they hold it.
- **`find_package(ros2top)` outside a system-wide install**, and the CMake
  config being unable to find its own header when used from a source checkout.
  The imported target now carries `nlohmann_json` so consumers cannot forget
  it. ([#9])
- **The C++ example did not build on Rolling.** It used
  `ament_target_dependencies`, which `ament_cmake` no longer provides there. It
  now links the imported targets directly, which works on all four supported
  distributions.
- **`pip install -e .` failed outright** with
  `build backend is missing the build_editable hook`: `build-system` required
  `setuptools>=61`, and PEP 660 landed in 64.
- **The `license` field** used PEP 639's expression form, which requires
  `setuptools>=77` - a release that dropped Python 3.8, which this project
  still supports.
- **A healthy status bar rendered in the error colour.** It was hardcoded to the
  red "Error/High usage" pair, so `ROS2✓ | Auto✓` read as a failure. It now
  reflects the state it describes.

### Changed

- The C++ API reference described three functions that do not exist and gave
  `register_node()` a signature it did not have. It now describes the API that
  is there, including `AutoNodeRegistrar`, which was implemented but never
  mentioned. The maintainer's home directory is gone from four sets of
  instructions.
- The README is reorganised around what a reader needs in the order they need
  it, with the long-form material in collapsible sections. It previously
  documented node registration three times over.
- The release workflow runs `twine check`, installs the built wheel and asserts
  its version matches the tag before uploading. A PyPI upload cannot be
  replaced.

## [0.4.0] - 2026-08-05

### Added

- **`ros2 top`.** ros2top registers itself with `ros2cli` through an entry
  point, so it appears in `ros2 --help` with no changes to ros2cli.

### Fixed

- Unit tests no longer stand up an rclpy node and join the ROS graph, which
  besides being wrong for a unit test segfaulted on Humble.

## [0.3.0] - 2026-08-05

### Added

- **Automatic node discovery from the ROS graph.** Nodes no longer have to
  register themselves to be monitored. On middleware whose DDS GUID encodes the
  process id - Fast DDS, the ROS 2 default - the node-to-PID mapping is
  recovered from the graph alone. Auto-discovered nodes are marked `~`, since
  their PID is inferred rather than reported. Nodes that cannot be resolved
  unambiguously, and nodes on other machines, are omitted rather than guessed
  at. ([#11])

## [0.2.1] - 2026-08-02

### Fixed

- **Composable node accounting.** A process hosting N nodes was sampled N
  times, so its CPU, RAM and GPU memory were counted N times in any total.
- Composed nodes are now grouped under the component container hosting them,
  with the process's usage reported once.
- The registry keys entries by PID **and** node name. Neither alone is unique: a
  container hosts several nodes in one process, and several processes can run
  nodes of the same name. Entries written by older versions are still read.
- Killing acts on process identity, and drops every node the dead process
  hosted rather than only the selected one.

## [0.2.0] - 2025-09-02

### Added

- Kill a monitored process from the UI.
- GitHub Actions workflow for building and publishing to PyPI.

## [0.1.3] - 2025

- Remove the dependency on ROS 2 being present to start ros2top.

## [0.1.2] - 2025

- README improvements.

## [0.1.1] - 2025

- Add example usage.

## [0.1.0] - 2025

- Initial release: node monitoring with CPU, RAM and GPU usage, a curses
  terminal interface, command line options, and the node registration API.

[#5]: https://github.com/AhmedARadwan/ros2top/issues/5
[#6]: https://github.com/AhmedARadwan/ros2top/issues/6
[#8]: https://github.com/AhmedARadwan/ros2top/issues/8
[#9]: https://github.com/AhmedARadwan/ros2top/issues/9
[#10]: https://github.com/AhmedARadwan/ros2top/issues/10
[#11]: https://github.com/AhmedARadwan/ros2top/issues/11
