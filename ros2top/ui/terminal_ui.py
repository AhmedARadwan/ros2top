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
        self.status_bar = None
        self.system_panel = None
        self.nodes_table = None
        self.help_panel = None
        
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
        layout_config = self.responsive_layout.get_layout_config(width, height)
        
        self.layout_manager.clear_components()
        
        # Status bar (top)
        self.status_bar = StatusBar(Rect(0, 0, width, 1))
        self.layout_manager.add_component(self.status_bar)
        
        # System overview - draw directly without panel
        system_height = self._calculate_system_panel_height(layout_config)
        self.system_start_row = 1
        
        # Nodes table
        table_y = 1 + system_height
        table_height = height - table_y - 2  # Reserve space for help
        
        headers = self._get_table_headers(layout_config)
        self.nodes_table = Table(
            Rect(0, table_y, width, table_height),
            headers
        )
        self.nodes_table.selectable = True
        self.layout_manager.add_component(self.nodes_table)
        
        # Help/status bar (bottom)
        help_bar = StatusBar(Rect(0, height - 1, width, 1))
        self._update_help_bar(help_bar)
        self.layout_manager.add_component(help_bar)
    
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
    
    def _update_ui(self):
        """Update UI components with current data"""
        current_time = time.time()
        
        # Check if we need to update
        if (current_time - self.last_update < self.update_interval and 
            not self.layout_manager.check_resize()):
            return
            
        try:
            height, width = self.stdscr.getmaxyx()
            
            # Recreate components if needed
            if not self.status_bar or self.layout_manager.check_resize():
                self._create_ui_components(width, height)
            
            # Update monitoring data
            if not self.paused:
                self.monitor.update_nodes()
                self.monitor.cleanup_dead_processes()
                self.stats['updates'] += 1
            
            # Clear screen and draw everything
            self.stdscr.erase()
            
            # Update and draw components
            self._update_status_bar()
            self._update_nodes_table()
            
            # Draw system overview directly
            self._draw_system_overview()
            
            # Draw layout manager components
            self.layout_manager.draw(self.colors)
            
            self.stdscr.refresh()
            self.last_update = current_time
            
        except curses.error as e:
            # Handle terminal too small or other display errors
            self._show_minimal_ui(f"Terminal too small or display error: {e}")
    
    def _draw_system_overview(self):
        """Draw system overview directly on screen"""
        if not self.stdscr:
            return
        
        try:
            max_y, max_x = self.stdscr.getmaxyx()
            current_row = self.system_start_row
            
            # Get system data
            memory = psutil.virtual_memory()
            cpu_percents = psutil.cpu_percent(percpu=True) if not self.paused else []
            
            # Draw CPU usage bars
            if cpu_percents:
                # Group CPUs for display
                cpus_per_row = max(1, (max_x - 10) // 25)  # Each CPU bar takes ~25 chars
                
                for i in range(0, len(cpu_percents), cpus_per_row):
                    if current_row >= max_y - 5:  # Leave space for rest of UI
                        break
                        
                    line = ""
                    max_cpu = 0
                    for j in range(cpus_per_row):
                        cpu_idx = i + j
                        if cpu_idx >= len(cpu_percents):
                            break
                            
                        cpu_percent = cpu_percents[cpu_idx]
                        max_cpu = max(max_cpu, cpu_percent)
                        bar = self._create_progress_bar(cpu_percent, 10)
                        line += f"CPU{cpu_idx+1:2d}[{bar}]{cpu_percent:5.1f}% "
                    
                    if line:
                        color = self._get_usage_color(max_cpu)
                        self._addstr_with_color(current_row, 0, line.rstrip(), color)
                        current_row += 1
            
            # Draw memory usage
            if current_row < max_y - 4:
                mem_percent = memory.percent
                mem_bar = self._create_progress_bar(mem_percent, 20)
                mem_line = f"Mem[{mem_bar}] {memory.used // (1024**3):.1f}G/{memory.total // (1024**3):.1f}G"
                self._addstr_with_color(current_row, 0, mem_line, self._get_usage_color(mem_percent))
                current_row += 1
            
            # Draw GPU info
            if current_row < max_y - 3 and self.monitor.is_gpu_available():
                for gpu_id in range(self.monitor.get_gpu_count()):
                    if current_row >= max_y - 3:
                        break
                        
                    gpu_info = self.monitor.gpu_monitor.get_gpu_info(gpu_id)
                    if gpu_info:
                        gpu_mem_percent = (gpu_info['memory_used_mb'] / gpu_info['memory_total_mb']) * 100
                        gpu_util_percent = gpu_info['utilization_gpu']
                        
                        gpu_bar = self._create_progress_bar(gpu_util_percent, 10)
                        mem_bar = self._create_progress_bar(gpu_mem_percent, 10)
                        
                        gpu_line = f"GPU{gpu_id}[{gpu_bar}]{gpu_util_percent:5.1f}% Mem[{mem_bar}]{gpu_info['memory_used_mb']:5.0f}M/{gpu_info['memory_total_mb']:5.0f}M"
                        self._addstr_with_color(current_row, 0, gpu_line, 6)  # Magenta for GPU
                        current_row += 1
            
            # ROS2 status
            if current_row < max_y - 2:
                ros2_status = "✓ ROS2 Active" if self.monitor.is_ros2_available() else "✗ ROS2 Not Available"
                node_count = self.monitor.get_nodes_count()
                status_line = f"{ros2_status} | Nodes: {node_count}"
                self._addstr_with_color(current_row, 0, status_line, 4)  # Cyan
                
        except curses.error:
            pass
    
    def _create_progress_bar(self, percent: float, width: int = 10) -> str:
        """Create a progress bar string"""
        filled = int((percent / 100.0) * width)
        bar = "█" * filled + "░" * (width - filled)
        return bar
    
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
    
    def _update_status_bar(self):
        """Update status bar content"""
        if not self.status_bar:
            return
            
        # Main status
        uptime = time.time() - self.stats['start_time']
        status_text = f"ros2top - {self.stats['updates']} updates - {uptime:.0f}s uptime"
        
        if self.paused:
            status_text += " [PAUSED]"
            
        self.status_bar.set_text(status_text)
        
        # Right side items
        self.status_bar.clear_items()
        
        # ROS2 status
        if self.monitor.is_ros2_available():
            self.status_bar.add_item("ROS2✓", self.colors.success)
        else:
            self.status_bar.add_item("ROS2✗", self.colors.error)
        
        # Node count
        node_count = self.monitor.get_nodes_count()
        self.stats['nodes_peak'] = max(self.stats['nodes_peak'], node_count)
        self.status_bar.add_item(f"Nodes:{node_count}", self.colors.info)
    
    def _update_nodes_table(self):
        """Update nodes table data"""
        if not self.nodes_table:
            return
            
        nodes = self.monitor.get_node_info_list()
        
        # Convert to table rows
        rows = []
        for node in nodes:
            row = [
                node.name,
                str(node.pid),
                f"{node.cpu_percent:.1f}",
                f"{node.ram_mb:.1f}"
            ]
            
            # Add GPU columns if enabled
            if self.monitor.is_gpu_available():
                if node.gpu_device_id >= 0:
                    row.extend([
                        str(node.gpu_device_id),
                        f"{node.gpu_utilization:.1f}",
                        f"{node.gpu_memory_mb:.0f}"
                    ])
                else:
                    row.extend(["--", "--", "--"])
            
            # Add extra columns for larger screens
            headers = self.nodes_table.headers
            if "Status" in headers:
                row.append("Running")  # Could add more sophisticated status
                
            if "Uptime" in headers:
                # Simple uptime calculation
                row.append("--")  # Placeholder
                
            if "Command" in headers:
                # Process command line (simplified)
                try:
                    proc = psutil.Process(node.pid)
                    cmd = " ".join(proc.cmdline()[:3])  # First 3 args
                    row.append(cmd[:20])  # Truncate
                except (psutil.Error, AttributeError):
                    row.append("--")
            
            rows.append(row)
        
        self.nodes_table.set_data(rows)
    
    def _update_help_bar(self, help_bar: StatusBar):
        """Update help bar with key bindings"""
        help_text = "q:Quit r:Refresh p:Pause h:Help ↑↓:Navigate Tab:Focus"
        help_bar.set_text(help_text)
    
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
