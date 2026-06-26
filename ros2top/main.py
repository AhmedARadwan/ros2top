#!/usr/bin/env python3
"""
Main entry point for ros2top
"""

import sys
import argparse
from . import __version__
from .node_monitor import NodeMonitor
from .ui.terminal_ui import run_ui, show_error_message


def create_argument_parser():
    """Create command line argument parser"""
    parser = argparse.ArgumentParser(
    description='Real-time monitor for ROS2 nodes showing CPU, RAM, and GPU usage',
    formatter_class=argparse.RawDescriptionHelpFormatter,
    epilog="""
Examples:
    ros2top-auto                    # Run with default settings
    ros2top-auto --refresh 1        # Faster discovery polling

Controls:
    q/Q - Quit
    r/R - Force refresh
    p/P - Pause/Resume
    k/K - Kill node
    +/- - Adjust speed
    h/H - Show help
    """
    )
    
    parser.add_argument(
        '--refresh', '-r',
        type=float,
        default=5.0,
        help='Node list refresh interval in seconds (default: 5.0)'
    )
    
    parser.add_argument(
        '--version', '-v',
        action='version',
        version=f'%(prog)s {__version__}'
    )
    
    return parser


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


def main():
    """Main entry point"""
    parser = create_argument_parser()
    args = parser.parse_args()
    
    # Check requirements
    if not check_requirements():
        sys.exit(1)
    
    # Validate arguments
    if args.refresh <= 0:
        show_error_message("Refresh interval must be positive")
        sys.exit(1)
    
    # Create node monitor
    try:
        monitor = NodeMonitor(refresh_interval=args.refresh)
    except Exception as e:
        show_error_message(f"Failed to initialize node monitor: {e}")
        sys.exit(1)
    
    # Run UI
    try:
        success = run_ui(monitor)
        if not success:
            sys.exit(1)
    except KeyboardInterrupt:
        print("\nGoodbye!")
    except Exception as e:
        show_error_message(f"Unexpected error: {e}")
        sys.exit(1)
    finally:
        # Cleanup background threads
        if hasattr(monitor, 'cleanup'):
            monitor.cleanup()


if __name__ == '__main__':
    main()
