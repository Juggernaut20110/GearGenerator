"""Bevel gears, straight and spiral: parameters, geometry and validation."""

from .geometry import (
    CrownTrace,
    MemberGeometry,
    SetGeometry,
    ToothSpaceSection,
    blank_outline,
    blank_reach_past_back_cone,
    compute_set,
    guide_spiral,
    inv,
    phase_at_cone_distance,
    section_cone_distances,
    section_count,
    to_cone_3d,
    tooth_space_section,
)
from .params import BevelSetParams
from .validate import Issue, ValidationResult, validate

__all__ = [
    "BevelSetParams",
    "CrownTrace",
    "Issue",
    "MemberGeometry",
    "SetGeometry",
    "ToothSpaceSection",
    "ValidationResult",
    "blank_outline",
    "blank_reach_past_back_cone",
    "compute_set",
    "guide_spiral",
    "inv",
    "phase_at_cone_distance",
    "section_cone_distances",
    "section_count",
    "to_cone_3d",
    "tooth_space_section",
    "validate",
]
