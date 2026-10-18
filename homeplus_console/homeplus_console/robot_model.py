"""Robot model for the 3D plan view: the URDF and its meshes from homeplus_urdf_description.

homeplus_urdf.xacro is plain URDF plus three includes (materials, transmissions, gazebo) that the
browser doesn't need, and it uses no xacro properties, so dropping the includes is enough to serve it
without ROS. On the ROS machine the /robot_description parameter can replace this (same content).
"""
from __future__ import annotations

import os
import xml.etree.ElementTree as ET
from functools import lru_cache
from pathlib import Path
from typing import Optional

XACRO_NS = "http://www.ros.org/wiki/xacro"


def description_dir() -> Path:
    env = os.environ.get("HOMEPLUS_URDF_DIR")
    if env:
        return Path(env)
    # homeplus_console/homeplus_console/robot_model.py → repo root
    return Path(__file__).resolve().parents[2] / "homeplus_urdf_description"


@lru_cache(maxsize=1)
def urdf_xml() -> Optional[str]:
    src = description_dir() / "urdf" / "homeplus_urdf.xacro"
    if not src.exists():
        return None
    ET.register_namespace("xacro", XACRO_NS)
    root = ET.parse(src).getroot()
    for parent in root.iter():
        for child in list(parent):
            if isinstance(child.tag, str) and child.tag.startswith("{" + XACRO_NS + "}"):
                parent.remove(child)
    # gazebo-only blocks are noise for the viewer, and the viewer colours robots by role (now / plan)
    for g in root.findall("gazebo") + root.findall("material"):
        root.remove(g)
    for parent in root.iter():
        for m in parent.findall("material"):
            parent.remove(m)
    return ET.tostring(root, encoding="unicode")


def mesh_path(name: str) -> Optional[Path]:
    p = (description_dir() / "meshes" / name).resolve()
    if p.parent != (description_dir() / "meshes").resolve() or not p.exists() or p.suffix.lower() != ".stl":
        return None
    return p
