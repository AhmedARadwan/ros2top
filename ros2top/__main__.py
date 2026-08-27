#!/usr/bin/env python3
"""
Support ``python -m ros2top``.

The ``ros2top`` console script lands in a bin directory that is not always on
PATH -- ``pip install --user`` puts it in ~/.local/bin, which several shells do
not add by default -- and the resulting "command not found" looks like a broken
install. Running the module works regardless of PATH.
"""

from .main import main

if __name__ == '__main__':
    main()
