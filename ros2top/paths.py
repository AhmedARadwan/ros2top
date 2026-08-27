#!/usr/bin/env python3
"""
Where ros2top's C++ integration files ended up.

The header and the CMake config are installed as data files, so their location
depends on how ros2top was installed: a normal wheel puts them under
``sys.prefix``, ``pip install --user`` under ``site.USER_BASE``, a venv under
that venv, and a source checkout does not install them at all.

CMake only searches a handful of system prefixes, so ``find_package(ros2top)``
fails for every one of those cases except the system-wide install -- which is
what people hit when the example package would not configure. Rather than ask
users to guess the prefix, ros2top reports it: ``ros2top --cmake-dir``.
"""

import site
import sys
from pathlib import Path
from typing import List, Optional

# Installed layout, relative to an installation prefix
_CMAKE_SUFFIX = Path('share') / 'ros2top' / 'cmake'
_CONFIG_NAME = 'ros2topConfig.cmake'
_HEADER_SUFFIX = Path('include') / 'ros2top' / 'ros2top.hpp'


def _prefixes() -> List[Path]:
    """Installation prefixes worth checking, most specific first"""
    here = Path(__file__).resolve()

    candidates = [
        # The prefix this very package was installed into. Walking up from
        # __file__ covers venvs, --user installs and unusual --prefix values
        # without having to know which of them happened.
        *[parent for parent in here.parents
          if (parent / _CMAKE_SUFFIX / _CONFIG_NAME).is_file()],
        Path(sys.prefix),
        Path(sys.base_prefix),
    ]

    user_base = getattr(site, 'USER_BASE', None)
    if user_base:
        candidates.append(Path(user_base))

    # Deduplicate, preserving order
    seen = set()
    unique = []
    for path in candidates:
        if path not in seen:
            seen.add(path)
            unique.append(path)
    return unique


def source_tree_cmake_dir() -> Optional[Path]:
    """
    The repository's own ``cmake/`` directory, when running from a checkout.

    An editable or plain source install never copies the data files anywhere,
    so this is the only copy that exists. Pointing CMake straight at the
    checkout works because ros2topConfig.cmake locates the header relative to
    itself.
    """
    repo = Path(__file__).resolve().parent.parent
    if (repo / 'cmake' / _CONFIG_NAME).is_file() and (repo / _HEADER_SUFFIX).is_file():
        return repo / 'cmake'
    return None


def cmake_dir() -> Optional[Path]:
    """
    Directory holding ros2topConfig.cmake, or None if it cannot be found.

    Pass it to CMake as ``-Dros2top_DIR=...``.
    """
    for prefix in _prefixes():
        candidate = prefix / _CMAKE_SUFFIX
        if (candidate / _CONFIG_NAME).is_file():
            return candidate
    return source_tree_cmake_dir()


def include_dir() -> Optional[Path]:
    """Directory to add to a compiler's include path, or None if not found"""
    for prefix in _prefixes():
        if (prefix / _HEADER_SUFFIX).is_file():
            return prefix / 'include'

    repo = Path(__file__).resolve().parent.parent
    if (repo / _HEADER_SUFFIX).is_file():
        return repo / 'include'
    return None
