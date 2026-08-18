"""Straight bevel gears: parameters, geometry and validation."""

from .geometry import (
    MemberGeometry,
    SetGeometry,
    ToothSpaceSection,
    blank_outline,
    blank_reach_past_back_cone,
    compute_set,
    inv,
    to_cone_3d,
    tooth_space_section,
)
from .params import BevelSetParams
from .validate import Issue, ValidationResult, validate

__all__ = [
    "BevelSetParams",
    "Issue",
    "MemberGeometry",
    "SetGeometry",
    "ToothSpaceSection",
    "ValidationResult",
    "blank_outline",
    "blank_reach_past_back_cone",
    "compute_set",
    "inv",
    "to_cone_3d",
    "tooth_space_section",
    "validate",
]
