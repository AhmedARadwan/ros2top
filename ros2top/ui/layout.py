#!/usr/bin/env python3
"""
Layout management for responsive UI
"""

import curses
from typing import List, Tuple, Optional, Dict
from .components import UIComponent, Rect, ColorScheme


class LayoutManager:
    """Manages UI layout and resizing"""
    
    def __init__(self, stdscr):
        self.stdscr = stdscr
        self.components: List[UIComponent] = []
        self.focused_component = 0
        self._last_size = (0, 0)
        
    def add_component(self, component: UIComponent):
        """Add a component to be managed"""
        self.components.append(component)
        
    def remove_component(self, component: UIComponent):
        """Remove a component"""
        if component in self.components:
            self.components.remove(component)
            if self.focused_component >= len(self.components):
                self.focused_component = max(0, len(self.components) - 1)
    
    def clear_components(self):
        """Clear all components"""
        self.components.clear()
        self.focused_component = 0
    
    def check_resize(self) -> bool:
        """Check if terminal was resized and update layout"""
        try:
            height, width = self.stdscr.getmaxyx()
            current_size = (height, width)
            
            if current_size != self._last_size:
                self._last_size = current_size
                self._update_layout(width, height)
                return True
                
        except curses.error:
            pass
        
        return False
    
    def mark_resize(self):
        """Force a resize check on next call"""
        self._last_size = (0, 0)
    
    def _update_layout(self, width: int, height: int):
        """Update component layouts after resize"""
        # Components should handle their own resizing
        for component in self.components:
            if hasattr(component, 'resize'):
                component.resize(width, height)
    
    def handle_key(self, key: int):
        """Handle keyboard input for layout navigation"""
        if key == ord('\t'):  # Tab key
            self._cycle_focus()
        elif self.components and self.focused_component < len(self.components):
            focused = self.components[self.focused_component]
            if hasattr(focused, 'handle_key'):
                focused.handle_key(key)
    
    def _cycle_focus(self):
        """Cycle focus to next focusable component"""
        focusable_count = sum(1 for c in self.components if getattr(c, 'focusable', False))
        if focusable_count > 0:
            attempts = 0
            while attempts < len(self.components):
                self.focused_component = (self.focused_component + 1) % len(self.components)
                if getattr(self.components[self.focused_component], 'focusable', False):
                    break
                attempts += 1
    
    def _update_layout(self, width: int, height: int):
        """Update component layout based on new size"""
        # This is a simple implementation - can be enhanced
        if not self.components:
            return
            
        # Simple vertical stacking
        component_height = max(1, height // len(self.components))
        
        for i, component in enumerate(self.components):
            y = i * component_height
            h = component_height
            
            # Last component gets remaining space
            if i == len(self.components) - 1:
                h = height - y
            
            component.resize(Rect(0, y, width, h))
    
    def handle_key(self, key: int) -> bool:
        """Handle keyboard input"""
        # Try focused component first
        if (0 <= self.focused_component < len(self.components) and 
            self.components[self.focused_component].handle_key(key)):
            return True
        
        # Handle global navigation
        if key == ord('\t'):  # Tab to next component
            if self.components:
                self.focused_component = (self.focused_component + 1) % len(self.components)
            return True
        elif key == curses.KEY_BTAB:  # Shift+Tab
            if self.components:
                self.focused_component = (self.focused_component - 1) % len(self.components)
            return True
        
        return False
    
    def update(self) -> bool:
        """Update all components"""
        needs_redraw = self.check_resize()
        
        for component in self.components:
            if component.update():
                needs_redraw = True
        
        return needs_redraw
    
    def draw(self, colors: ColorScheme):
        """Draw all components"""
        for i, component in enumerate(self.components):
            component.focused = (i == self.focused_component)
            component.draw(self.stdscr, colors)


class ResponsiveLayout:
    """Responsive layout that adapts to terminal size"""
    
    def __init__(self):
        self.breakpoints = {
            'small': 80,
            'medium': 120,
            'large': 160
        }
        
    def get_layout_config(self, width: int, height: int) -> Dict:
        """Get layout configuration based on terminal size"""
        config = {
            'size_class': 'small',
            'columns': 1,
            'show_gpu': True,
            'show_detailed_cpu': False,
            'table_rows': height - 10,  # Reserve space for header/footer
            'progress_bar_width': 10,
        }
        
        if width >= self.breakpoints['medium']:
            config.update({
                'size_class': 'medium',
                'columns': 2,
                'show_detailed_cpu': True,
                'progress_bar_width': 15,
            })
        
        if width >= self.breakpoints['large']:
            config.update({
                'size_class': 'large',
                'columns': 3,
                'progress_bar_width': 20,
            })
        
        # Adjust for height
        if height < 20:
            config['show_gpu'] = False
            config['show_detailed_cpu'] = False
        
        return config
