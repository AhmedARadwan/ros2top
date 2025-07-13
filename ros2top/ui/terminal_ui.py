#!/usr/bin/env python3
"""
Enhanced terminal UI for ros2top with responsive design
"""

import curses
import time
import signal
import psutil
from typing import List, Optional, Dict, Any
from ..node_monitor import NodeMonitor, NodeInfo
from .components import (
    UIComponent, Rect, ColorScheme, StatusBar, ProgressBar, 
    Table, Panel
)
from .layout import LayoutManager, ResponsiveLayout


class TerminalUI:
    """Enhanced terminal interface with responsive design"""
    
    def __init__(self, monitor: NodeMonitor):
        self.monitor = monitor
        self.stdscr = None
        self.running = True
        self.layout_manager = None
        self.responsive_layout = ResponsiveLayout()
        self.colors = ColorScheme()
        
        # UI Components
        self.system_panel = None
        self.nodes_table = None
        self.help_panel = None
        
        # Section boundaries (initialized with defaults)
        self.system_section = None
        self.table_section = None
        self.controls_section = None
        
        # State
        self.show_help = False
        self.last_update = 0
        self.update_interval = 1.0  # seconds
        self.paused = False
        
        # Statistics
        self.stats = {
            'updates': 0,
            'start_time': time.time(),
            'nodes_peak': 0
        }
        
    def run(self, stdscr):
        """Main UI loop"""
        self.stdscr = stdscr
        self._setup_terminal()
        self._init_colors()
        self._init_signal_handlers()
        
        # Create layout manager
        self.layout_manager = LayoutManager(stdscr)
        
        try:
            while self.running:
                self._update_ui()
                self._handle_input()
                
                # Adaptive refresh rate
                if self.paused:
                    time.sleep(0.5)
                else:
                    time.sleep(0.1)
                    
        except KeyboardInterrupt:
            pass
        except Exception as e:
            self._show_error(f"UI Error: {e}")
        finally:
            self.monitor.shutdown()
    
    def _setup_terminal(self):
        """Setup terminal settings"""
        curses.curs_set(0)  # Hide cursor
        self.stdscr.nodelay(True)  # Non-blocking input
        self.stdscr.timeout(100)  # 100ms timeout for input
        curses.noecho()
        curses.cbreak()
        
    def _init_colors(self):
        """Initialize color scheme"""
        if not curses.has_colors():
            return
            
        curses.start_color()
        curses.use_default_colors()
        
        # Define color pairs
        color_pairs = [
            (1, curses.COLOR_CYAN, -1),     # Header
            (2, curses.COLOR_GREEN, -1),    # Success/Low usage
            (3, curses.COLOR_YELLOW, -1),   # Warning/Medium usage  
            (4, curses.COLOR_RED, -1),      # Error/High usage
            (5, curses.COLOR_BLUE, -1),     # Info
            (6, curses.COLOR_MAGENTA, -1),  # Accent
            (7, curses.COLOR_WHITE, -1),    # Dim
        ]
        
        for pair_num, fg, bg in color_pairs:
            try:
                curses.init_pair(pair_num, fg, bg)
            except curses.error:
                pass
        
        # Update color scheme
        self.colors = ColorScheme(
            normal=0,
            header=1,
            success=2,
            warning=3,
            error=4,
            info=5,
            accent=6,
            dim=7
        )
    
    def _init_signal_handlers(self):
        """Setup signal handlers for graceful shutdown"""
        def signal_handler(signum, frame):
            self.running = False
            
        signal.signal(signal.SIGINT, signal_handler)
        signal.signal(signal.SIGTERM, signal_handler)
    
    def _create_ui_components(self, width: int, height: int):
        """Create UI components based on terminal size"""
        self.layout_manager.clear_components()
        
        # Calculate section heights with adaptive system section
        # Calculate needed height for system section based on CPU count (new format)
        cpu_count = psutil.cpu_count() or 1
        cpu_display_width = 35  # " 1  [|||||||             12.3%]  " format
        cpus_per_row = max(1, width // cpu_display_width)
        cpu_rows_needed = (cpu_count + cpus_per_row - 1) // cpus_per_row  # Ceiling division
        
        # System section needs: CPU rows + RAM row + GPU rows (if available) + padding
        min_system_height = cpu_rows_needed + 1  # CPU rows + RAM
        if self.monitor.is_gpu_available():
            min_system_height += self.monitor.get_gpu_count() * 2  # GPU util + GPU mem per GPU
        min_system_height += 1  # Extra padding
        
        # Use the larger of minimum needed or quarter of terminal height
        system_height = max(min_system_height, height // 4)
        # But cap it to not take more than half the terminal
        system_height = min(system_height, height // 2)
        
        controls_height = 2  # Last two lines for shortcuts
        table_height = height - system_height - controls_height  # Middle section
        
        # Store section boundaries
        self.system_section = {
            'start_y': 0,
            'height': system_height,
            'width': width
        }
        
        self.table_section = {
            'start_y': system_height,
            'height': table_height,
            'width': width
        }
        
        self.controls_section = {
            'start_y': height - controls_height,
            'height': controls_height,
            'width': width
        }
        
        # Create table component for middle section
        headers = ["PID", "Uptime", "%CPU", "RAM(MB)", "GPU#", "%GPU", "GMEM(MB)", "Node Name"]
        self.nodes_table = Table(
            Rect(0, self.table_section['start_y'], width, table_height),
            headers
        )
        self.nodes_table.selectable = True
        self.layout_manager.add_component(self.nodes_table)
    
    def _calculate_system_panel_height(self, config: Dict) -> int:
        """Calculate height needed for system panel"""
        base_height = 3  # Minimum for memory + header
        
        if config['show_detailed_cpu']:
            cpu_count = psutil.cpu_count() or 1
            cpu_rows = (cpu_count + 3) // 4  # 4 CPUs per row
            base_height += cpu_rows
        else:
            base_height += 1  # Single CPU line
            
        if config['show_gpu'] and self.monitor.is_gpu_available():
            base_height += self.monitor.get_gpu_count()
            
        return min(base_height + 2, 12)  # Cap at 12 lines
    
    def _get_table_headers(self, config: Dict) -> List[str]:
        """Get table headers based on layout configuration"""
        headers = ["Node", "PID", "%CPU", "RAM(MB)"]
        
        if config['show_gpu'] and self.monitor.is_gpu_available():
            headers.extend(["GPU#", "GPU%", "GMEM(MB)"])
        
        # Add more columns for wider screens
        if config['size_class'] in ['medium', 'large']:
            headers.append("Status")
            
        if config['size_class'] == 'large':
            headers.extend(["Uptime", "Command"])
            
        return headers
    
    def _format_uptime(self, start_time: float) -> str:
        """
        Format uptime from process start time
        
        Args:
            start_time: Unix timestamp of process start
            
        Returns:
            Formatted uptime string in DDd:HHh:MMm:SSs format
        """
        try:
            current_time = time.time()
            uptime_seconds = int(current_time - start_time)
            
            # Calculate components
            days = uptime_seconds // 86400
            hours = (uptime_seconds % 86400) // 3600
            minutes = (uptime_seconds % 3600) // 60
            seconds = uptime_seconds % 60
            
            # Format based on duration
            if days > 0:
                return f"{days:02d}d:{hours:02d}h:{minutes:02d}m:{seconds:02d}s"
            elif hours > 0:
                return f"{hours:02d}h:{minutes:02d}m:{seconds:02d}s"
            elif minutes > 0:
                return f"{minutes:02d}m:{seconds:02d}s"
            else:
                return f"{seconds:02d}s"
        except (ValueError, TypeError, OSError):
            return "unknown"
    
    def _update_ui(self):
        """Update UI components with current data"""
        current_time = time.time()
        
        # Check if terminal was resized
        terminal_resized = self.layout_manager.check_resize()
        
        # Check if we need to update
        if (current_time - self.last_update < self.update_interval and not terminal_resized):
            return
            
        try:
            height, width = self.stdscr.getmaxyx()
            
            # Only recreate components if terminal was resized OR components don't exist
            if not self.nodes_table or terminal_resized:
                self._create_ui_components(width, height)
            
            # Update monitoring data
            if not self.paused:
                self.monitor.update_nodes()
                self.monitor.cleanup_dead_processes()
                self.stats['updates'] += 1
            
            # Clear screen and draw everything
            self.stdscr.erase()
            
            # Draw the three sections
            self._draw_system_overview()      # Top quarter section
            self._update_nodes_table()        # Update table data
            self._draw_controls_section()     # Bottom two lines
            
            # Draw layout manager components (table)
            self.layout_manager.draw(self.colors)
            
            self.stdscr.refresh()
            self.last_update = current_time
            
        except curses.error as e:
            # Handle terminal too small or other display errors
            self._show_minimal_ui(f"Terminal too small or display error: {e}")
    
    def _draw_system_overview(self):
        """Draw system overview in the top quarter section"""
        if not self.stdscr or not self.system_section:
            return
        
        try:
            section = self.system_section
            current_row = section['start_y']
            max_width = section['width']
            
            # Get system data
            memory = psutil.virtual_memory()
            cpu_percents = psutil.cpu_percent(percpu=True) if not self.paused else []
            
            # Draw CPU usage with new format: " 1  [|||||||             12.3%]  2  [||||||||||||        45.6%]"
            if cpu_percents and current_row < section['start_y'] + section['height'] - 1:
                # Calculate CPU display parameters for new format
                # Format: " 1  [|||||||             12.3%]  " = ~35 characters
                cpu_display_width = 35
                cpus_per_row = max(1, max_width // cpu_display_width)
                
                # Display CPUs row by row
                for i in range(0, len(cpu_percents), cpus_per_row):
                    if current_row >= section['start_y'] + section['height'] - 1:
                        break
                        
                    line = ""
                    line_width = 0
                    
                    for j in range(cpus_per_row):
                        cpu_idx = i + j
                        if cpu_idx >= len(cpu_percents):
                            break
                            
                        cpu_percent = cpu_percents[cpu_idx]
                        bar = self._create_progress_bar(cpu_percent, 20)
                        cpu_display = f"{cpu_idx + 1:2}  [{bar}{cpu_percent:5.1f}%]  "
                        
                        # Check if adding this CPU would exceed terminal width
                        if line_width + len(cpu_display) > max_width:
                            break
                            
                        line += cpu_display
                        line_width += len(cpu_display)
                    
                    if line and current_row < section['start_y'] + section['height']:
                        # Get the highest CPU usage in this row for color coding
                        row_cpu_percents = cpu_percents[i:i+j+1] if j >= 0 else cpu_percents[i:i+1]
                        color = self._get_usage_color(max(row_cpu_percents))
                        self._addstr_with_color(current_row, 0, line.rstrip(), color)
                        current_row += 1
            
            # Draw RAM usage with new format: "  Mem[||||||||||    3.20G/  8.00G]"
            if current_row < section['start_y'] + section['height']:
                mem_percent = memory.percent
                mem_gb_used = memory.used / (1024**3)
                mem_gb_total = memory.total / (1024**3)
                
                # Create properly aligned memory display
                mem_text = f"{mem_gb_used:5.2f}G/{mem_gb_total:6.2f}G"
                mem_bar_display = self._create_aligned_bar_with_text(mem_percent, mem_text, 50)
                mem_line = f"RAM {mem_bar_display}"
                self._addstr_with_color(current_row, 0, mem_line, self._get_usage_color(mem_percent))
                current_row += 1
            
            # Draw GPU usage and GPU RAM usage with new format
            if self.monitor.is_gpu_available() and current_row < section['start_y'] + section['height']:
                for gpu_id in range(self.monitor.get_gpu_count()):
                    if current_row >= section['start_y'] + section['height']:
                        break
                        
                    gpu_info = self.monitor.gpu_monitor.get_gpu_info(gpu_id)
                    if gpu_info:
                        gpu_util = gpu_info['utilization_gpu']
                        gpu_mem_used = gpu_info['memory_used_mb']
                        gpu_mem_total = gpu_info['memory_total_mb']
                        gpu_mem_percent = (gpu_mem_used / gpu_mem_total) * 100 if gpu_mem_total > 0 else 0
                        
                        # GPU utilization line: "   GPU [|||||||             12.3%]"
                        gpu_util_text = f"{gpu_util:5.1f}%"
                        gpu_bar_display = self._create_aligned_bar_with_text(gpu_util, gpu_util_text, 50)
                        gpu_line = f"GPU {gpu_bar_display}"
                        self._addstr_with_color(current_row, 0, gpu_line, self._get_usage_color(gpu_util))
                        current_row += 1

                        # GPU memory line: "GMEM[|                 512M/  2.00G]"
                        if current_row < section['start_y'] + section['height']:
                            gpu_mem_gb = gpu_mem_total / 1024  # Convert MB to GB
                            gpu_mem_text = f"{gpu_mem_used:6.0f}M/{gpu_mem_gb:6.2f}G"
                            gpu_mem_bar_display = self._create_aligned_bar_with_text(gpu_mem_percent, gpu_mem_text, 50)
                            gpu_mem_line = f"GMEM{gpu_mem_bar_display}"
                            self._addstr_with_color(current_row, 0, gpu_mem_line, self._get_usage_color(gpu_mem_percent))
                            current_row += 1
                
        except curses.error:
            pass
    
    def _draw_controls_section(self):
        """Draw controls/shortcuts in the bottom section"""
        if not self.stdscr or not self.controls_section:
            return
        
        try:
            section = self.controls_section
            
            # Line 1: Main controls
            controls_line1 = "q:Quit  h:Help  r:Refresh  p:Pause/Resume  ↑↓:Navigate  Tab:Focus"
            self._addstr_with_color(section['start_y'], 0, controls_line1[:section['width']], 0)
            
            # Line 2: Status and additional info
            if section['height'] > 1:
                ros2_status = "ROS2✓" if self.monitor.is_ros2_available() else "ROS2✗"
                node_count = self.monitor.get_nodes_count()
                status_info = f"{ros2_status} | Nodes:{node_count} | +/-:Speed | Space:Update"
                self._addstr_with_color(section['start_y'] + 1, 0, status_info[:section['width']], 4)
                
        except curses.error:
            pass
    
    def _create_progress_bar(self, percent: float, width: int = 20) -> str:
        """Create a progress bar string in new style with pipes and spaces"""
        filled = int((percent / 100.0) * width)
        bar = "|" * filled + " " * (width - filled)
        return bar
    
    def _create_aligned_bar_with_text(self, percent: float, text: str, total_width: int = 30) -> str:
        """Create a progress bar with right-aligned text within brackets"""
        # Calculate bar width (total width minus text length minus brackets)
        text_len = len(text)
        bar_width = total_width - text_len - 2  # -2 for brackets []
        
        if bar_width < 1:
            bar_width = 1
            
        filled = int((percent / 100.0) * bar_width)
        empty = bar_width - filled
        
        # Adjust empty space to accommodate text
        if empty >= text_len:
            # Text fits in empty space
            bar = "|" * filled + " " * (empty - text_len) + text
        else:
            # Text doesn't fit, reduce bar
            available_bar = max(1, bar_width - text_len)
            filled = min(filled, available_bar)
            bar = "|" * filled + " " * (bar_width - filled - text_len) + text
            
        return f"[{bar}]"
    
    def _get_usage_color(self, percent: float) -> int:
        """Get color based on usage percentage"""
        if not curses.has_colors():
            return 0
        if percent >= 80:
            return 4  # Red/Error
        elif percent >= 50:
            return 3  # Yellow/Warning
        else:
            return 2  # Green/Success
    
    def _addstr_with_color(self, y: int, x: int, text: str, color_pair: int = 0):
        """Add string with color if available"""
        try:
            max_y, max_x = self.stdscr.getmaxyx()
            if y >= max_y or x >= max_x:
                return
                
            # Truncate text if too long
            available_width = max_x - x
            if len(text) > available_width:
                text = text[:available_width]
                
            if curses.has_colors() and color_pair > 0:
                self.stdscr.addstr(y, x, text, curses.color_pair(color_pair))
            else:
                self.stdscr.addstr(y, x, text)
        except curses.error:
            pass
    
    def _update_nodes_table(self):
        """Update nodes table data with specified format"""
        if not self.nodes_table:
            return
            
        nodes = self.monitor.get_node_info_list()
        
        # Calculate available width for node name column
        # Fixed widths for other columns: PID(7), Uptime(8), %CPU(6), RAM(MB)(8), GPU#(4), %GPU(6), GMEM(MB)(9)
        fixed_columns_width = 7 + 8 + 6 + 8 + 4 + 6 + 9  # Total: 48 chars
        separators_width = 7  # 7 separators between 8 columns
        available_width = self.table_section['width'] if hasattr(self, 'table_section') and self.table_section else 80
        node_name_width = max(20, available_width - fixed_columns_width - separators_width)
        
        # Convert to table rows with specified columns:
        # PID, Uptime, %CPU, RAM(MB), GPU#, %GPU, GMEM(MB), Node Name
        rows = []
        for node in nodes:
            # Calculate uptime using the new formatting function
            uptime = self._format_uptime(node.start_time)
            
            # Get node name - use full available width
            node_name = node.name if node.name else "unknown"
            # Truncate to available width
            if len(node_name) > node_name_width:
                node_name = node_name[:node_name_width - 3] + "..."
            
            row = [
                str(node.pid),                    # PID
                uptime,                          # Uptime
                f"{node.cpu_percent:.1f}",       # %CPU
                f"{node.ram_mb:.1f}",            # RAM(MB)
            ]
            
            # Add GPU columns
            if node.gpu_device_id >= 0:
                row.extend([
                    str(node.gpu_device_id),           # GPU#
                    f"{node.gpu_utilization:.1f}",     # %GPU
                    f"{node.gpu_memory_mb:.0f}",       # GMEM(MB)
                ])
            else:
                row.extend(["--", "--", "--"])
            
            row.append(node_name)                # Node Name
            
            rows.append(row)
        
        self.nodes_table.set_data(rows)
    
    def _handle_input(self):
        """Handle keyboard input"""
        try:
            key = self.stdscr.getch()
            if key == -1:  # No input
                return
                
            # Global keys
            if key == ord('q') or key == ord('Q'):
                self.running = False
            elif key == ord('r') or key == ord('R'):
                self.monitor.force_refresh()
                self.last_update = 0  # Force immediate update
            elif key == ord('p') or key == ord('P'):
                self.paused = not self.paused
            elif key == ord('h') or key == ord('H'):
                self._show_help_dialog()
            elif key == ord('+') or key == ord('='):
                self.update_interval = max(0.5, self.update_interval - 0.5)
            elif key == ord('-'):
                self.update_interval = min(5.0, self.update_interval + 0.5)
            else:
                # Pass to layout manager
                if self.layout_manager:
                    self.layout_manager.handle_key(key)
                    
        except curses.error:
            pass
    
    def _show_help_dialog(self):
        """Show help dialog"""
        if not self.stdscr:
            return
            
        height, width = self.stdscr.getmaxyx()
        
        help_lines = [
            "ros2top Enhanced Terminal UI",
            "",
            "Global Controls:",
            "  q/Q      - Quit application",
            "  r/R      - Force refresh data", 
            "  p/P      - Pause/resume updates",
            "  h/H      - Show this help",
            "  +/=      - Faster updates",
            "  -        - Slower updates",
            "",
            "Navigation:",
            "  ↑/↓      - Navigate table rows",
            "  Tab      - Switch focus between panels",
            "  Home/End - Jump to first/last row",
            "",
            "Features:",
            "  • Responsive layout adapts to terminal size",
            "  • Real-time CPU, memory, and GPU monitoring",
            "  • Automatic node discovery via registry",
            "  • Color-coded usage indicators",
            "",
            "Color Legend:",
            "  Green    - Low usage (< 50%)",
            "  Yellow   - Medium usage (50-80%)",
            "  Red      - High usage (> 80%)",
            "",
            "Press any key to continue..."
        ]
        
        # Calculate dialog dimensions
        dialog_width = min(max(len(line) for line in help_lines) + 4, width - 4)
        dialog_height = min(len(help_lines) + 4, height - 4)
        
        start_y = (height - dialog_height) // 2
        start_x = (width - dialog_width) // 2
        
        try:
            # Create dialog window
            dialog = curses.newwin(dialog_height, dialog_width, start_y, start_x)
            dialog.box()
            
            # Add content
            for i, line in enumerate(help_lines[:dialog_height - 4]):
                if len(line) <= dialog_width - 4:
                    dialog.addstr(i + 2, 2, line)
                else:
                    dialog.addstr(i + 2, 2, line[:dialog_width - 4])
            
            dialog.refresh()
            
            # Wait for input
            dialog.nodelay(False)
            dialog.getch()
            
            # Cleanup
            del dialog
            self.stdscr.clear()
            
        except curses.error:
            pass
    
    def _show_minimal_ui(self, message: str):
        """Show minimal UI when terminal is too small"""
        try:
            self.stdscr.erase()
            self.stdscr.addstr(0, 0, "ros2top - Terminal too small")
            self.stdscr.addstr(1, 0, message[:curses.COLS-1] if message else "")
            self.stdscr.addstr(2, 0, "Resize terminal or press 'q' to quit")
            self.stdscr.refresh()
        except curses.error:
            pass
    
    def _show_error(self, message: str):
        """Show error message"""
        try:
            self.stdscr.addstr(0, 0, f"ERROR: {message}")
            self.stdscr.refresh()
            time.sleep(2)
        except curses.error:
            pass


def show_error_message(message: str):
    """Show error message when curses is not available"""
    print(f"Error: {message}")
    print("This tool requires a terminal that supports curses.")


def run_ui(monitor: NodeMonitor):
    """Run the enhanced terminal UI"""
    ui = TerminalUI(monitor)
    
    try:
        curses.wrapper(ui.run)
        return True
    except Exception as e:
        print(f"Failed to start enhanced UI: {e}")
        print("Terminal may not support required features.")
        return False
