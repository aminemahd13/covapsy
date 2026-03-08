"""Pytest path bootstrap for local ROS2 workspace source layout.

Allows running tests from ``mycar/ros2_ws`` without an installed overlay.
"""

from __future__ import annotations

import pathlib
import sys


def _add_src_packages_to_path() -> None:
    ws_root = pathlib.Path(__file__).resolve().parent
    src_dir = ws_root / "src"
    if not src_dir.exists():
        return

    for pkg_dir in src_dir.iterdir():
        if not pkg_dir.is_dir():
            continue
        py_pkg = pkg_dir / pkg_dir.name
        if py_pkg.is_dir() and str(pkg_dir) not in sys.path:
            sys.path.insert(0, str(pkg_dir))


_add_src_packages_to_path()

