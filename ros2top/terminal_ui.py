#!/usr/bin/env python3
"""
Terminal UI using curses for ros2top
"""

import curses
import time
from typing import List, Optional
from .node_monitor import NodeMonitor, NodeInfo


class TerminalUI:
    """Curses-based terminal interface for ros2top"""
    
    def __init__(self, monitor: NodeMonitor):
        self.monitor = monitor
        self.stdscr = None
        self.running = True
        
    def run(self, stdscr):
        """Main UI loop"""
        self.stdscr = stdscr
        curses.curs_set(0)  # Hide cursor
        stdscr.nodelay(True)  # Non-blocking input
        
        try:
            while self.running:
                self._update_display()
                self._handle_input()
                time.sleep(1)
        except KeyboardInterrupt:
            pass
        finally:
            self.monitor.shutdown()
    
    def _update_display(self):
        """Update the display with current node information"""
        if not self.stdscr:
            return
            
        self.stdscr.erase()
        
        # Update monitoring data
        self.monitor.update_nodes()
        self.monitor.cleanup_dead_processes()
        
        # Get node information
        nodes = self.monitor.get_node_info_list()
        
        # Display header
        self._draw_header()
        
        # Display system status
        self._draw_status(len(nodes))
        
        # Display node information
        self._draw_nodes(nodes)
        
        # Display help
        self._draw_help()
        
        self.stdscr.refresh()
    
    def _draw_header(self):
        """Draw the column headers"""
        if not self.stdscr:
            return
            
        try:
            if self.monitor.is_gpu_available():
                header = f"{'Node':<30} {'PID':>6} {'%CPU':>7} {'RAM(MB)':>8} {'GPU#':>4} {'GPU%':>5} {'GMEM':>6}"
            else:
                header = f"{'Node':<30} {'PID':>6} {'%CPU':>7} {'RAM(MB)':>8}"
            
            self.stdscr.addstr(0, 0, header)
            
            # Add separator line
            self.stdscr.addstr(1, 0, "─" * min(len(header), curses.COLS - 1))
            
        except curses.error:
            pass  # Ignore if we can't write to screen
    
    def _draw_status(self, node_count: int):
        """Draw system status information"""
        if not self.stdscr:
            return
            
        try:
            status_y = node_count + 3
            
            if not self.monitor.is_ros2_available():
                self.stdscr.addstr(status_y, 0, "⚠️  ROS2 not available! Make sure ROS2 is sourced.")
                return
            
            status_lines = [
                f"Nodes: {node_count}",
                f"GPUs: {self.monitor.get_gpu_count()}",
                f"ROS2: {'✓' if self.monitor.is_ros2_available() else '✗'}",
            ]
            
            status_text = " | ".join(status_lines)
            self.stdscr.addstr(status_y, 0, status_text)
            
        except curses.error:
            pass
    
    def _draw_nodes(self, nodes: List[NodeInfo]):
        """Draw node information"""
        if not self.stdscr:
            return
            
        max_y, max_x = self.stdscr.getmaxyx()
        
        for idx, node in enumerate(nodes):
            row = idx + 2  # Start after header and separator
            
            # Don't draw beyond screen
            if row >= max_y - 4:  # Leave space for status and help
                break
                
            try:
                if self.monitor.is_gpu_available():
                    if node.gpu_device_id >= 0:
                        line = (f"{node.name:<30} {node.pid:6} {node.cpu_percent:7.2f} "
                               f"{node.ram_mb:8.1f} {node.gpu_device_id:4} "
                               f"{node.gpu_utilization:5.1f} {node.gpu_memory_mb:6}")
                    else:
                        line = (f"{node.name:<30} {node.pid:6} {node.cpu_percent:7.2f} "
                               f"{node.ram_mb:8.1f} {'--':>4} {'--':>5} {'--':>6}")
                else:
                    line = f"{node.name:<30} {node.pid:6} {node.cpu_percent:7.2f} {node.ram_mb:8.1f}"
                
                # Truncate line if too long
                if len(line) > max_x - 1:
                    line = line[:max_x - 1]
                
                self.stdscr.addstr(row, 0, line)
                
            except curses.error:
                pass  # Ignore if we can't write to screen
    
    def _draw_help(self):
        """Draw help text"""
        if not self.stdscr:
            return
            
        try:
            max_y, _ = self.stdscr.getmaxyx()
            help_y = max_y - 2
            
            help_text = "Press 'q' to quit | 'r' to refresh | 'h' for help"
            self.stdscr.addstr(help_y, 0, help_text)
            
        except curses.error:
            pass
    
    def _handle_input(self):
        """Handle keyboard input"""
        if not self.stdscr:
            return
            
        try:
            key = self.stdscr.getch()
            
            if key == ord('q') or key == ord('Q'):
                self.running = False
            elif key == ord('r') or key == ord('R'):
                self.monitor.force_refresh()
            elif key == ord('h') or key == ord('H'):
                self._show_help_dialog()
                
        except curses.error:
            pass
    
    def _show_help_dialog(self):
        """Show help dialog"""
        if not self.stdscr:
            return
            
        height, width = self.stdscr.getmaxyx()
        
        # Help text
        help_lines = [
            "ROS2Top Help",
            "",
            "Controls:",
            "  q/Q - Quit",
            "  r/R - Force refresh node list",
            "  h/H - Show this help",
            "",
            "Columns:",
            "  Node    - ROS2 node name",
            "  PID     - Process ID",
            "  %CPU    - CPU usage percentage",
            "  RAM(MB) - RAM usage in megabytes",
            "  GPU#    - GPU device number",
            "  GPU%    - GPU utilization percentage",
            "  GMEM    - GPU memory usage in MB",
            "",
            "Press any key to continue..."
        ]
        
        # Calculate dialog size
        dialog_height = len(help_lines) + 4
        dialog_width = max(len(line) for line in help_lines) + 4
        dialog_width = min(dialog_width, width - 4)
        
        start_y = max(0, (height - dialog_height) // 2)
        start_x = max(0, (width - dialog_width) // 2)
        
        try:
            # Create dialog window
            dialog = curses.newwin(dialog_height, dialog_width, start_y, start_x)
            dialog.box()
            
            # Add help text
            for i, line in enumerate(help_lines):
                if i + 2 < dialog_height - 2:  # Leave space for borders
                    truncated_line = line[:dialog_width - 4]
                    dialog.addstr(i + 2, 2, truncated_line)
            
            dialog.refresh()
            
            # Wait for keypress
            dialog.nodelay(False)
            dialog.getch()
            
            # Clean up
            del dialog
            self.stdscr.clear()
            
        except curses.error:
            pass


def show_error_message(message: str):
    """Show error message when curses is not available"""
    print(f"Error: {message}")
    print("This tool requires a terminal that supports curses.")


def run_ui(monitor: NodeMonitor):
    """Run the terminal UI"""
    ui = TerminalUI(monitor)
    
    try:
        curses.wrapper(ui.run)
    except Exception as e:
        show_error_message(f"Failed to initialize terminal UI: {e}")
        return False
    
    return True
