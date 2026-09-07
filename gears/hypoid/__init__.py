"""Gleason/ISO Method 1 hypoid gear geometry."""

from .geometry import (
    HypoidSection,
    HypoidSetGeometry,
    HypoidMethod1Geometry,
    HypoidMemberGeometry,
    HypoidThicknessGeometry,
    blank_outline,
    compute_set,
    hypoid_tooth_space_loop,
    section_cone_distances,
    section_count,
    tooth_space_section,
)
from .params import HypoidSetParams
from .validate import validate

__all__ = [
    "HypoidMemberGeometry", "HypoidMethod1Geometry", "HypoidSection", "HypoidSetGeometry",
    "HypoidThicknessGeometry",
    "HypoidSetParams", "blank_outline", "compute_set",
    "hypoid_tooth_space_loop", "section_cone_distances", "section_count",
    "tooth_space_section", "validate",
]
