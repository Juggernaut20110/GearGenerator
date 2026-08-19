"""Planetary gear trains: a sun, N planets and an internal ring gear.

Composed out of the spur type rather than owning a second involute. Every member
is a spur gear and every mesh is a spur mesh; what lives here is the train - the
tooth-count relation, the assembly condition, where each planet sits, and how
each member is clocked so that all of them mesh at once.
"""

from .geometry import (
    MEMBERS,
    PlanetarySetGeometry,
    centre_distance_from_ring,
    compute_set,
    planet_ring_params,
    sun_planet_params,
)
from .params import PlanetarySetParams
from .validate import Issue, ValidationResult, validate

__all__ = [
    "Issue",
    "MEMBERS",
    "PlanetarySetGeometry",
    "PlanetarySetParams",
    "ValidationResult",
    "centre_distance_from_ring",
    "compute_set",
    "planet_ring_params",
    "sun_planet_params",
    "validate",
]
