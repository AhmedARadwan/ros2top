#!/usr/bin/env python3
"""
Main entry point for ros2top
"""

import sys
import argparse
from . import __version__
from .node_monitor import NodeMonitor
from .ui.terminal_ui import run_ui, show_error_message


DESCRIPTION = 'Real-time monitor for ROS2 nodes showing CPU, RAM, and GPU usage'

EPILOG = """
Examples:
    ros2top                      # Run with default settings
    ros2top --refresh 2          # Refresh every 2 seconds
    ros2top --no-auto-discovery  # Only registered nodes
    ros2top --doctor             # Explain why nodes are or aren't showing up
    ros2top --cmake-dir          # Print the path find_package(ros2top) needs

    Also available as a ros2 CLI sub-command:
    ros2 top                     # identical to `ros2top`

    Controls:
    q/Q - Quit
    r/R - Force refresh node list
    h/H - Show help
"""


def add_arguments(parser):
    """
    Add ros2top's arguments to a parser.

    Shared with the ros2cli plugin, which is handed a subparser rather than
    creating its own, so `ros2top` and `ros2 top` cannot drift apart.
    """
    parser.add_argument(
        '--refresh', '-r',
        type=float,
        default=0.1,
        help='Node refresh interval in seconds (default: 0.1)'
    )
    
    parser.add_argument(
        '--no-auto-discovery',
        action='store_true',
        help='Only show nodes that registered with ros2top, never those found '
             'on the ROS graph'
    )

    parser.add_argument(
        '--doctor',
        action='store_true',
        help='Print a diagnostic report explaining what ros2top can and cannot '
             'see, then exit. Paste it into a bug report.'
    )

    parser.add_argument(
        '--cmake-dir',
        action='store_true',
        help='Print the directory holding ros2topConfig.cmake, for '
             'colcon build --cmake-args -Dros2top_DIR=$(ros2top --cmake-dir)'
    )

    parser.add_argument(
        '--include-dir',
        action='store_true',
        help="Print the directory holding ros2top's C++ header, for compilers "
             'invoked without CMake'
    )

    parser.add_argument(
        '--version', '-v',
        action='version',
        version=f'%(prog)s {__version__}'
    )

    return parser


def create_argument_parser():
    """Create command line argument parser for the standalone `ros2top`"""
    parser = argparse.ArgumentParser(
        description=DESCRIPTION,
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=EPILOG,
    )
    return add_arguments(parser)


def check_requirements():
    """Check if required dependencies are available"""
    try:
        import psutil
    except ImportError:
        show_error_message("psutil is required but not installed. Run: pip install psutil")
        return False
    
    try:
        import curses
    except ImportError:
        show_error_message("curses is required but not available on this system")
        return False
    
    return True


def _print_path(args) -> int:
    """
    Print an integration path for --cmake-dir / --include-dir.

    Prints the bare path and nothing else, so it can be used directly in a
    command substitution. Failures go to stderr with a non-zero exit, so a
    shell using it does not silently substitute an empty string.
    """
    from . import paths

    if getattr(args, 'cmake_dir', False):
        path, what = paths.cmake_dir(), 'ros2topConfig.cmake'
    else:
        path, what = paths.include_dir(), 'ros2top/ros2top.hpp'

    if path is None:
        print(f'ros2top: could not locate {what}. Install the wheel with '
              f'`pip install ros2top`, or run this from a source checkout.',
              file=sys.stderr)
        return 1

    print(path)
    return 0


def run(args) -> int:
    """
    Run the monitor with already-parsed arguments, returning an exit code.

    Returns rather than exits so the ros2cli plugin can hand the code back to
    ros2cli instead of tearing the process down from underneath it.
    """
    # Flags that print something and exit. Handled before check_requirements()
    # so that `--doctor` still works on a system where curses is the problem,
    # and before the UI so nothing tries to take over the terminal.
    if getattr(args, 'cmake_dir', False) or getattr(args, 'include_dir', False):
        return _print_path(args)

    if getattr(args, 'doctor', False):
        from .doctor import run as run_doctor
        return run_doctor()

    # Check requirements
    if not check_requirements():
        return 1

    # Validate arguments
    if args.refresh <= 0:
        show_error_message("Refresh interval must be positive")
        return 1

    # Create node monitor
    try:
        monitor = NodeMonitor(refresh_interval=args.refresh,
                              auto_discovery=not args.no_auto_discovery)
    except Exception as e:
        show_error_message(f"Failed to initialize node monitor: {e}")
        return 1

    # Run UI
    try:
        return 0 if run_ui(monitor) else 1
    except KeyboardInterrupt:
        print("\nGoodbye!")
        return 0
    except Exception as e:
        show_error_message(f"Unexpected error: {e}")
        return 1
    finally:
        # Cleanup background threads
        if hasattr(monitor, 'cleanup'):
            monitor.cleanup()


def main():
    """Entry point for the standalone `ros2top` command"""
    sys.exit(run(create_argument_parser().parse_args()))


if __name__ == '__main__':
    main()
