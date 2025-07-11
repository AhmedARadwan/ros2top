#!/usr/bin/env python3
"""
Base UI components for ros2top terminal interface
"""

import curses
import time
from abc import ABC, abstractmethod
from typing import List, Tuple, Optional, Dict, Any
from dataclasses import dataclass


@dataclass
class Rect:
    """Rectangle for UI positioning"""
    x: int
    y: int
    width: int
    height: int
    
    @property
    def right(self) -> int:
        return self.x + self.width
    
    @property
    def bottom(self) -> int:
        return self.y + self.height
    
    def contains_point(self, x: int, y: int) -> bool:
        return self.x <= x < self.right and self.y <= y < self.bottom


@dataclass
class ColorScheme:
    """Color scheme for the UI"""
    normal: int = 0
    header: int = 1
    success: int = 2
    warning: int = 3
    error: int = 4
    info: int = 5
    accent: int = 6
    dim: int = 7


class UIComponent(ABC):
    """Base class for UI components"""
    
    def __init__(self, rect: Rect):
        self.rect = rect
        self.visible = True
        self.focused = False
        self.dirty = True  # Needs redraw
        
    @abstractmethod
    def draw(self, stdscr, colors: ColorScheme):
        """Draw the component"""
        pass
    
    def resize(self, new_rect: Rect):
        """Resize the component"""
        self.rect = new_rect
        self.dirty = True
    
    def handle_key(self, key: int) -> bool:
        """Handle keyboard input. Return True if handled."""
        return False
    
    def update(self) -> bool:
        """Update component state. Return True if redraw needed."""
        return False


class StatusBar(UIComponent):
    """Status bar component"""
    
    def __init__(self, rect: Rect, text: str = ""):
        super().__init__(rect)
        self.text = text
        self.items: List[Tuple[str, int]] = []  # (text, color)
    
    def set_text(self, text: str):
        if self.text != text:
            self.text = text
            self.dirty = True
    
    def add_item(self, text: str, color: int = 0):
        self.items.append((text, color))
        self.dirty = True
    
    def clear_items(self):
        if self.items:
            self.items.clear()
            self.dirty = True
    
    def draw(self, stdscr, colors: ColorScheme):
        if not self.visible:
            return
            
        try:
            # Clear the line
            stdscr.addstr(self.rect.y, self.rect.x, " " * self.rect.width)
            
            # Draw main text
            if self.text:
                text = self.text[:self.rect.width]
                stdscr.addstr(self.rect.y, self.rect.x, text)
            
            # Draw items from right to left
            x_pos = self.rect.right - 1
            for text, color in reversed(self.items):
                if x_pos - len(text) >= self.rect.x:
                    x_pos -= len(text)
                    if color and curses.has_colors():
                        stdscr.addstr(self.rect.y, x_pos, text, curses.color_pair(color))
                    else:
                        stdscr.addstr(self.rect.y, x_pos, text)
                    x_pos -= 1  # Space between items
                else:
                    break
                    
        except curses.error:
            pass
        
        self.dirty = False


class ProgressBar(UIComponent):
    """Progress bar component"""
    
    def __init__(self, rect: Rect, value: float = 0.0, max_value: float = 100.0):
        super().__init__(rect)
        self.value = value
        self.max_value = max_value
        self.label = ""
        self.show_percentage = True
        self.bar_style = "█░"  # filled, empty
    
    def set_value(self, value: float):
        if abs(self.value - value) > 0.01:  # Avoid unnecessary updates
            self.value = value
            self.dirty = True
    
    def set_label(self, label: str):
        if self.label != label:
            self.label = label
            self.dirty = True
    
    def draw(self, stdscr, colors: ColorScheme):
        if not self.visible or self.rect.width < 3:
            return
            
        try:
            percentage = (self.value / self.max_value) * 100 if self.max_value > 0 else 0
            percentage = max(0, min(100, percentage))
            
            # Calculate bar width (reserve space for label and percentage)
            label_width = len(self.label) + 1 if self.label else 0
            percentage_width = 6 if self.show_percentage else 0  # " 100%"
            bar_width = max(1, self.rect.width - label_width - percentage_width)
            
            # Draw label
            x_pos = self.rect.x
            if self.label:
                stdscr.addstr(self.rect.y, x_pos, self.label)
                x_pos += len(self.label)
            
            # Draw progress bar
            filled_width = int((percentage / 100.0) * bar_width)
            empty_width = bar_width - filled_width
            
            bar_text = self.bar_style[0] * filled_width + self.bar_style[1] * empty_width
            
            # Choose color based on percentage
            if percentage >= 80:
                color = colors.error
            elif percentage >= 60:
                color = colors.warning
            else:
                color = colors.success
            
            if curses.has_colors() and color:
                stdscr.addstr(self.rect.y, x_pos, bar_text, curses.color_pair(color))
            else:
                stdscr.addstr(self.rect.y, x_pos, bar_text)
            
            x_pos += bar_width
            
            # Draw percentage
            if self.show_percentage and x_pos < self.rect.right:
                percentage_text = f"{percentage:5.1f}%"
                stdscr.addstr(self.rect.y, x_pos, percentage_text)
                
        except curses.error:
            pass
        
        self.dirty = False


class Table(UIComponent):
    """Table component for displaying data"""
    
    def __init__(self, rect: Rect, headers: List[str]):
        super().__init__(rect)
        self.headers = headers
        self.rows: List[List[str]] = []
        self.column_widths: List[int] = []
        self.selected_row = 0
        self.scroll_offset = 0
        self.sortable = True
        self.sort_column = 0
        self.sort_ascending = True
        self.selectable = True
        
        self._calculate_column_widths()
    
    def _calculate_column_widths(self):
        """Calculate optimal column widths"""
        if not self.headers:
            return
            
        # Start with header widths
        self.column_widths = [len(header) for header in self.headers]
        
        # Adjust based on data
        for row in self.rows:
            for i, cell in enumerate(row[:len(self.column_widths)]):
                self.column_widths[i] = max(self.column_widths[i], len(str(cell)))
        
        # Special handling for Command column (last column) to use remaining width
        if len(self.column_widths) > 0:
            # Calculate space used by all columns except the last one
            other_columns_width = sum(self.column_widths[:-1]) + len(self.column_widths) - 1  # separators
            remaining_width = self.rect.width - other_columns_width
            
            # Give the last column (Command) all remaining space, with a minimum width
            if remaining_width > 10:  # Minimum width for command column
                self.column_widths[-1] = remaining_width
            else:
                # If not enough space, shrink all columns proportionally except command
                total_width = sum(self.column_widths) + len(self.column_widths) - 1  # separators
                if total_width > self.rect.width:
                    # Keep command column at minimum and shrink others
                    self.column_widths[-1] = max(10, self.rect.width // 4)  # At least 1/4 of width for command
                    available_for_others = self.rect.width - self.column_widths[-1] - len(self.column_widths) + 1
                    if available_for_others > 0 and len(self.column_widths) > 1:
                        other_total = sum(self.column_widths[:-1])
                        if other_total > 0:
                            scale_factor = available_for_others / other_total
                            for i in range(len(self.column_widths) - 1):
                                self.column_widths[i] = max(3, int(self.column_widths[i] * scale_factor))
    
    def set_data(self, rows: List[List[str]]):
        """Set table data"""
        self.rows = rows
        self._calculate_column_widths()
        self.dirty = True
    
    def add_row(self, row: List[str]):
        """Add a single row"""
        self.rows.append(row)
        self._calculate_column_widths()
        self.dirty = True
    
    def clear(self):
        """Clear all data"""
        if self.rows:
            self.rows.clear()
            self.selected_row = 0
            self.scroll_offset = 0
            self.dirty = True
    
    def handle_key(self, key: int) -> bool:
        """Handle keyboard input"""
        if not self.selectable or not self.rows:
            return False
            
        if key == curses.KEY_UP:
            if self.selected_row > 0:
                self.selected_row -= 1
                if self.selected_row < self.scroll_offset:
                    self.scroll_offset = self.selected_row
                self.dirty = True
            return True
        elif key == curses.KEY_DOWN:
            if self.selected_row < len(self.rows) - 1:
                self.selected_row += 1
                visible_rows = self.rect.height - 2  # header + separator
                if self.selected_row >= self.scroll_offset + visible_rows:
                    self.scroll_offset = self.selected_row - visible_rows + 1
                self.dirty = True
            return True
        elif key == curses.KEY_HOME:
            self.selected_row = 0
            self.scroll_offset = 0
            self.dirty = True
            return True
        elif key == curses.KEY_END:
            self.selected_row = len(self.rows) - 1
            visible_rows = self.rect.height - 2
            self.scroll_offset = max(0, len(self.rows) - visible_rows)
            self.dirty = True
            return True
        
        return False
    
    def scroll_up(self):
        """Scroll up one row"""
        if self.selected_row > 0:
            self.selected_row -= 1
            if self.selected_row < self.scroll_offset:
                self.scroll_offset = self.selected_row
            self.dirty = True
    
    def scroll_down(self):
        """Scroll down one row"""
        if self.selected_row < len(self.rows) - 1:
            self.selected_row += 1
            visible_rows = self.rect.height - 2  # header + separator
            if self.selected_row >= self.scroll_offset + visible_rows:
                self.scroll_offset = self.selected_row - visible_rows + 1
            self.dirty = True
    
    def draw(self, stdscr, colors: ColorScheme):
        """Draw the table"""
        if not self.visible or self.rect.height < 2:
            return
            
        try:
            current_row = self.rect.y
            
            # Draw headers
            self._draw_header_row(stdscr, colors, current_row)
            current_row += 1
            
            # Draw separator
            if current_row < self.rect.bottom:
                separator = "─" * self.rect.width
                stdscr.addstr(current_row, self.rect.x, separator[:self.rect.width])
                current_row += 1
            
            # Draw data rows
            visible_rows = self.rect.height - 2  # header + separator
            end_row = min(len(self.rows), self.scroll_offset + visible_rows)
            
            for i in range(self.scroll_offset, end_row):
                if current_row >= self.rect.bottom:
                    break
                    
                row = self.rows[i]
                is_selected = (i == self.selected_row) and self.selectable
                
                self._draw_data_row(stdscr, colors, current_row, row, is_selected)
                current_row += 1
                
        except curses.error:
            pass
        
        self.dirty = False
    
    def _draw_header_row(self, stdscr, colors: ColorScheme, y: int):
        """Draw the header row"""
        x_pos = self.rect.x
        
        for i, (header, width) in enumerate(zip(self.headers, self.column_widths)):
            if x_pos >= self.rect.right:
                break
                
            # Special handling for the last column (Command) - use remaining width
            if i == len(self.headers) - 1:
                remaining_width = self.rect.right - x_pos
                header_text = header[:remaining_width] if remaining_width > 0 else ""
            else:
                # Truncate header if necessary for other columns
                header_text = header[:width].ljust(width)
            
            if curses.has_colors():
                stdscr.addstr(y, x_pos, header_text, curses.color_pair(colors.header))
            else:
                stdscr.addstr(y, x_pos, header_text)
            
            x_pos += len(header_text)
            
            # Add separator only if not the last column and there's space
            if i < len(self.headers) - 1 and x_pos < self.rect.right:
                stdscr.addstr(y, x_pos, " ")
                x_pos += 1
    
    def _draw_data_row(self, stdscr, colors: ColorScheme, y: int, row: List[str], is_selected: bool):
        """Draw a data row"""
        x_pos = self.rect.x
        
        # Clear the line first if selected
        if is_selected:
            stdscr.addstr(y, self.rect.x, " " * self.rect.width)
        
        for i, (cell, width) in enumerate(zip(row, self.column_widths)):
            if x_pos >= self.rect.right:
                break
                
            # Special handling for the last column (Command) - use remaining width
            if i == len(self.column_widths) - 1:
                remaining_width = self.rect.right - x_pos
                cell_text = str(cell)[:remaining_width] if remaining_width > 0 else ""
            else:
                # Truncate cell if necessary for other columns
                cell_text = str(cell)[:width].ljust(width)
            
            if is_selected and curses.has_colors():
                stdscr.addstr(y, x_pos, cell_text, curses.color_pair(colors.accent) | curses.A_REVERSE)
            else:
                stdscr.addstr(y, x_pos, cell_text)
            
            x_pos += len(cell_text)
            
            # Add separator only if not the last column and there's space
            if i < len(row) - 1 and x_pos < self.rect.right:
                if is_selected and curses.has_colors():
                    stdscr.addstr(y, x_pos, " ", curses.color_pair(colors.accent) | curses.A_REVERSE)
                else:
                    stdscr.addstr(y, x_pos, " ")
                x_pos += 1


class Panel(UIComponent):
    """Panel component that can contain other components"""
    
    def __init__(self, rect: Rect, title: str = ""):
        super().__init__(rect)
        self.title = title
        self.components: List[UIComponent] = []
        self.border = True
        self.focused_component = 0
    
    def add_component(self, component: UIComponent):
        """Add a component to the panel"""
        self.components.append(component)
        self.dirty = True
    
    def remove_component(self, component: UIComponent):
        """Remove a component from the panel"""
        if component in self.components:
            self.components.remove(component)
            self.dirty = True
    
    def clear_components(self):
        """Clear all components"""
        self.components.clear()
        self.focused_component = 0
        self.dirty = True
    
    def handle_key(self, key: int) -> bool:
        """Handle keyboard input"""
        # First try focused component
        if (0 <= self.focused_component < len(self.components) and 
            self.components[self.focused_component].handle_key(key)):
            return True
        
        # Handle panel navigation
        if key == ord('\t'):  # Tab to next component
            if self.components:
                self.focused_component = (self.focused_component + 1) % len(self.components)
                self.dirty = True
            return True
        elif key == curses.KEY_BTAB:  # Shift+Tab to previous component
            if self.components:
                self.focused_component = (self.focused_component - 1) % len(self.components)
                self.dirty = True
            return True
        
        return False
    
    def update(self) -> bool:
        """Update all components"""
        needs_redraw = self.dirty
        
        for component in self.components:
            if component.update():
                needs_redraw = True
        
        return needs_redraw
    
    def resize(self, new_rect: Rect):
        """Resize panel and redistribute component space"""
        super().resize(new_rect)
        
        # Calculate available space for components
        if not self.components:
            return
            
        content_x = self.rect.x + (1 if self.border else 0)
        content_y = self.rect.y + (1 if self.border else 0)
        content_width = self.rect.width - (2 if self.border else 0)
        content_height = self.rect.height - (2 if self.border else 0)
        
        # Account for title
        if self.title:
            content_y += 1
            content_height -= 1
        
        # Distribute space evenly among components
        if content_height > 0:
            component_height = max(1, content_height // len(self.components))
            
            for i, component in enumerate(self.components):
                comp_y = content_y + i * component_height
                
                # Ensure we don't exceed panel bounds
                if comp_y >= self.rect.bottom:
                    break
                    
                # Adjust height for last component to fill remaining space
                if i == len(self.components) - 1:
                    comp_height = self.rect.bottom - comp_y - (1 if self.border else 0)
                else:
                    comp_height = component_height
                
                comp_rect = Rect(
                    x=content_x,
                    y=comp_y,
                    width=max(1, content_width),
                    height=max(1, comp_height)
                )
                component.resize(comp_rect)
    
    def draw(self, stdscr, colors: ColorScheme):
        """Draw the panel and all its components"""
        if not self.visible:
            return
            
        try:
            # Draw border
            if self.border:
                # Simple border
                for y in range(self.rect.height):
                    for x in range(self.rect.width):
                        screen_y = self.rect.y + y
                        screen_x = self.rect.x + x
                        
                        if y == 0 or y == self.rect.height - 1:
                            char = "─"
                        elif x == 0 or x == self.rect.width - 1:
                            char = "│"
                        else:
                            continue
                        
                        stdscr.addstr(screen_y, screen_x, char)
                
                # Corners
                stdscr.addstr(self.rect.y, self.rect.x, "┌")
                stdscr.addstr(self.rect.y, self.rect.right - 1, "┐")
                stdscr.addstr(self.rect.bottom - 1, self.rect.x, "└")
                stdscr.addstr(self.rect.bottom - 1, self.rect.right - 1, "┘")
            
            # Draw title
            if self.title:
                title_y = self.rect.y + (1 if self.border else 0)
                title_text = f" {self.title} "
                if len(title_text) > self.rect.width - 2:
                    title_text = title_text[:self.rect.width - 2]
                
                title_x = self.rect.x + (self.rect.width - len(title_text)) // 2
                if curses.has_colors():
                    stdscr.addstr(title_y, title_x, title_text, curses.color_pair(colors.header))
                else:
                    stdscr.addstr(title_y, title_x, title_text)
            
            # Draw components
            for i, component in enumerate(self.components):
                component.focused = (i == self.focused_component)
                component.draw(stdscr, colors)
                
        except curses.error:
            pass
        
        self.dirty = False
