#!/usr/bin/env python3
"""
Terminal UI using curses for ros2top
"""

import curses
import time
import psutil
from typing import List, Optional
from .node_monitor import NodeMonitor, NodeInfo


class TerminalUI:
    """Curses-based terminal interface for ros2top"""
    
    def __init__(self, monitor: NodeMonitor):
        self.monitor = monitor
        self.stdscr = None
        self.running = True
        self.scroll_offset = 0
        
        # Initialize colors if supported
        self.colors_available = False
        
    def run(self, stdscr):
        """Main UI loop"""
        self.stdscr = stdscr
        curses.curs_set(0)  # Hide cursor
        stdscr.nodelay(True)  # Non-blocking input
        
        # Initialize colors
        self._init_colors()
        
        try:
            while self.running:
                self._update_display()
                self._handle_input()
                time.sleep(0.5)  # Faster refresh for htop-like feel
        except KeyboardInterrupt:
            pass
        finally:
            self.monitor.shutdown()
    
    def _init_colors(self):
        """Initialize color pairs"""
        if curses.has_colors():
            curses.start_color()
            curses.use_default_colors()
            
            # Define color pairs
            curses.init_pair(1, curses.COLOR_GREEN, -1)    # Green for low usage
            curses.init_pair(2, curses.COLOR_YELLOW, -1)   # Yellow for medium usage
            curses.init_pair(3, curses.COLOR_RED, -1)      # Red for high usage
            curses.init_pair(4, curses.COLOR_CYAN, -1)     # Cyan for headers
            curses.init_pair(5, curses.COLOR_BLUE, -1)     # Blue for system info
            curses.init_pair(6, curses.COLOR_MAGENTA, -1)  # Magenta for GPU
            
            self.colors_available = True
    
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
        
        current_row = 0
        
        # Display system overview (htop style)
        current_row = self._draw_system_overview(current_row)
        
        # Add spacing
        current_row += 1
        
        # Display header
        current_row = self._draw_header(current_row)
        
        # Display node information
        current_row = self._draw_nodes(nodes, current_row)
        
        # Display help at bottom
        self._draw_help()
        
        self.stdscr.refresh()
    
    def _draw_system_overview(self, start_row: int) -> int:
        """Draw system overview like htop"""
        if not self.stdscr:
            return start_row
            
        try:
            max_y, max_x = self.stdscr.getmaxyx()
            current_row = start_row
            
            # System information
            memory = psutil.virtual_memory()
            
            # CPU usage bars
            cpu_percents = psutil.cpu_percent(percpu=True) or []
            current_row = self._draw_cpu_bars(cpu_percents, current_row, max_x, max_y)
            
            # Memory usage
            current_row = self._draw_memory_bar(memory, current_row)
            
            # GPU information
            current_row = self._draw_gpu_info(current_row)
            
            # ROS2 status
            current_row = self._draw_ros2_status(current_row)
            
            return current_row
            
        except curses.error:
            return start_row + 1
    
    def _draw_cpu_bars(self, cpu_percents: List[float], current_row: int, max_x: int, max_y: int) -> int:
        """Draw CPU usage bars"""
        if not cpu_percents:
            return current_row
            
        # Calculate how many CPUs we can fit per row
        cpus_per_row = max(1, (max_x - 10) // 25)  # Each CPU bar takes ~25 chars
        
        for i in range(0, len(cpu_percents), cpus_per_row):
            if current_row >= max_y - 5:  # Leave space for rest of UI
                break
                
            line = ""
            row_percents = []
            for j in range(cpus_per_row):
                cpu_idx = i + j
                if cpu_idx >= len(cpu_percents):
                    break
                    
                cpu_percent = cpu_percents[cpu_idx]
                row_percents.append(cpu_percent)
                bar = self._create_progress_bar(cpu_percent, 10)
                line += f"CPU{cpu_idx+1:2d}[{bar}]{cpu_percent:5.1f}% "
            
            if line:
                color = self._get_usage_color(max(row_percents) if row_percents else 0)
                self._addstr_with_color(current_row, 0, line, color)
                current_row += 1
        
        return current_row
    
    def _draw_memory_bar(self, memory, current_row: int) -> int:
        """Draw memory usage bar"""
        mem_percent = memory.percent
        mem_bar = self._create_progress_bar(mem_percent, 20)
        mem_line = f"Mem[{mem_bar}] {memory.used // (1024**3):.1f}G/{memory.total // (1024**3):.1f}G"
        self._addstr_with_color(current_row, 0, mem_line, self._get_usage_color(mem_percent))
        return current_row + 1
    
    def _draw_gpu_info(self, current_row: int) -> int:
        """Draw GPU information"""
        if not self.monitor.is_gpu_available():
            return current_row
            
        for gpu_id in range(self.monitor.get_gpu_count()):
            gpu_info = self.monitor.gpu_monitor.get_gpu_info(gpu_id)
            if gpu_info:
                gpu_mem_percent = (gpu_info['memory_used_mb'] / gpu_info['memory_total_mb']) * 100
                gpu_util_percent = gpu_info['utilization_gpu']
                
                gpu_bar = self._create_progress_bar(gpu_util_percent, 10)
                mem_bar = self._create_progress_bar(gpu_mem_percent, 10)
                
                gpu_line = f"GPU{gpu_id}[{gpu_bar}]{gpu_util_percent:5.1f}% Mem[{mem_bar}]{gpu_info['memory_used_mb']:5.0f}M/{gpu_info['memory_total_mb']:5.0f}M"
                self._addstr_with_color(current_row, 0, gpu_line, 6)  # Magenta for GPU
                current_row += 1
        
        return current_row
    
    def _draw_ros2_status(self, current_row: int) -> int:
        """Draw ROS2 status"""
        ros2_status = "✓ ROS2 Active" if self.monitor.is_ros2_available() else "✗ ROS2 Not Available"
        node_count = self.monitor.get_nodes_count()
        status_line = f"{ros2_status} | Nodes: {node_count}"
        self._addstr_with_color(current_row, 0, status_line, 4)  # Cyan
        return current_row + 1
    
    def _create_progress_bar(self, percent: float, width: int = 10) -> str:
        """Create a progress bar string"""
        filled = int((percent / 100.0) * width)
        bar = "█" * filled + "░" * (width - filled)
        return bar
    
    def _get_usage_color(self, percent: float) -> int:
        """Get color based on usage percentage"""
        if not self.colors_available:
            return 0
        if percent >= 80:
            return 3  # Red
        elif percent >= 50:
            return 2  # Yellow
        else:
            return 1  # Green
    
    def _addstr_with_color(self, y: int, x: int, text: str, color_pair: int = 0):
        """Add string with color if available"""
        try:
            if self.colors_available and color_pair > 0:
                self.stdscr.addstr(y, x, text, curses.color_pair(color_pair))
            else:
                self.stdscr.addstr(y, x, text)
        except curses.error:
            pass
    
    def _draw_header(self, start_row: int) -> int:
        """Draw the column headers"""
        if not self.stdscr:
            return start_row
            
        try:
            if self.monitor.is_gpu_available():
                header = f"{'Node':<30} {'PID':>6} {'%CPU':>7} {'RAM(MB)':>8} {'GPU#':>4} {'GPU%':>5} {'GMEM':>6}"
            else:
                header = f"{'Node':<30} {'PID':>6} {'%CPU':>7} {'RAM(MB)':>8}"
            
            self._addstr_with_color(start_row, 0, header, 4)  # Cyan for headers
            
            # Add separator line
            separator = "─" * min(len(header), curses.COLS - 1)
            self.stdscr.addstr(start_row + 1, 0, separator)
            
            return start_row + 2
            
        except curses.error:
            return start_row + 2
    
    def _draw_nodes(self, nodes: List[NodeInfo], start_row: int) -> int:
        """Draw node information"""
        if not self.stdscr:
            return start_row
            
        max_y, max_x = self.stdscr.getmaxyx()
        current_row = start_row
        
        for idx, node in enumerate(nodes):
            if idx < self.scroll_offset:
                continue
                
            display_row = current_row + (idx - self.scroll_offset)
            
            # Don't draw beyond screen
            if display_row >= max_y - 2:  # Leave space for help
                break
                
            try:
                line = self._format_node_line(node)
                cpu_color = self._get_usage_color(node.cpu_percent)
                
                # Truncate line if too long
                if len(line) > max_x - 1:
                    line = line[:max_x - 1]
                
                self._addstr_with_color(display_row, 0, line, cpu_color)
                
            except curses.error:
                break  # Stop if we can't write to screen
        
        return max_y - 2
    
    def _format_node_line(self, node: NodeInfo) -> str:
        """Format a single node line"""
        if self.monitor.is_gpu_available():
            if node.gpu_device_id >= 0:
                return (f"{node.name:<30} {node.pid:6} {node.cpu_percent:7.2f} "
                       f"{node.ram_mb:8.1f} {node.gpu_device_id:4} "
                       f"{node.gpu_utilization:5.1f} {node.gpu_memory_mb:6}")
            else:
                return (f"{node.name:<30} {node.pid:6} {node.cpu_percent:7.2f} "
                       f"{node.ram_mb:8.1f} {'--':>4} {'--':>5} {'--':>6}")
        else:
            return f"{node.name:<30} {node.pid:6} {node.cpu_percent:7.2f} {node.ram_mb:8.1f}"
    
    def _draw_help(self):
        """Draw help text"""
        if not self.stdscr:
            return
            
        try:
            max_y, _ = self.stdscr.getmaxyx()
            help_y = max_y - 2
            
            help_text = "Press 'q' to quit | 'r' to refresh | 'h' for help | ↑↓ scroll"
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
            elif key == curses.KEY_UP:
                self.scroll_offset = max(0, self.scroll_offset - 1)
            elif key == curses.KEY_DOWN:
                max_nodes = self.monitor.get_nodes_count()
                self.scroll_offset = min(max_nodes - 1, self.scroll_offset + 1)
                
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
            "  q/Q      - Quit",
            "  r/R      - Force refresh node list",
            "  h/H      - Show this help",
            "  Up/Down  - Scroll through nodes",
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
            "Colors:",
            "  Green   - Low usage (< 50%)",
            "  Yellow  - Medium usage (50-80%)",
            "  Red     - High usage (> 80%)",
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
