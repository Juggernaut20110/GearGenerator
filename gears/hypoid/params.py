"""User-facing parameters for a Gleason/ISO Method 1 hypoid pair.

Lengths are millimetres and angles are degrees at this boundary.  The module
is the outer transverse module of the wheel, matching the published Method 1
input convention (``d_e2 = module * z2``).  The solver exposes the generated
mean normal module in its result rather than pretending it is an input.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from ..params_io import JsonParams


@dataclass(frozen=True)
class HypoidSetParams(JsonParams):
    """Inputs for one external hypoid pair.

    The advanced factors are the Method 1 data-type-I defaults used by the
    published anchor.  They remain fields so a saved preset can retain a
    cutter/design choice without changing the basic GUI surface.
    """

    module: float                 # outer transverse module of the wheel
    z1: int                       # pinion teeth
    z2: int                       # wheel teeth
    face_width: float             # wheel face width, mm
    bore: float                   # pinion bore diameter, mm
    hub_thickness: float          # pinion backing, mm

    pressure_angle: float = 20.0  # nominal normal pressure angle, degrees
    shaft_angle: float = 90.0
    offset: float = 0.0           # signed common-normal axis offset, mm
    spiral_angle: float = 35.0    # pinion mean spiral angle, degrees
    hand: str = "right"
    cutter_radius: float | None = None
    backlash: float = 0.0
    min_root_thickness: float = 0.5

    # ISO Method 1 data-type-I factors.  These are intentionally editable in
    # JSON/API use but are not put in the first GUI pass.
    profile_shift: float = 0.35       # x_hm1 - c_ham
    depth_factor: float = 2.0         # k_d
    clearance_factor: float = 0.125   # k_c
    thickness_factor: float = 0.10    # k_t
    gear_addendum_angle: float = 1.0  # theta_a2, degrees
    gear_dedendum_angle: float = 4.0  # theta_f2, degrees

    @property
    def alpha(self) -> float:
        return math.radians(self.pressure_angle)

    @property
    def sigma(self) -> float:
        return math.radians(self.shaft_angle)

    @property
    def psi1(self) -> float:
        value = math.radians(self.spiral_angle)
        return value if self.hand == "right" else -value

    @property
    def beta(self) -> float:
        """Signed pinion spiral angle in radians."""
        return self.psi1

    @property
    def hypoid_offset(self) -> float:
        """Descriptive alias for callers that prefer the full term."""
        return self.offset

    @property
    def ratio(self) -> float:
        return self.z2 / self.z1

    @property
    def wheel_outer_diameter(self) -> float:
        return self.module * self.z2

    @property
    def wheel_outer_radius(self) -> float:
        return self.wheel_outer_diameter / 2.0

    @classmethod
    def with_defaults(cls, module: float, z1: int, z2: int, **overrides):
        """Construct a practical pair around the requested outer module."""
        # A Method 1 hypoid face is normally narrower than the wheel radius.
        face_width = min(0.18 * module * z2, 10.0 * module)
        defaults = {
            "face_width": round(face_width, 2),
            "bore": round(max(0.22 * module * z1, 4.0), 1),
            "hub_thickness": round(2.5 * module, 2),
            "offset": 0.0,
            "spiral_angle": 35.0,
            "cutter_radius": round(0.375 * module * z2, 4),
            "min_root_thickness": round(0.25 * module, 2),
        }
        defaults.update(overrides)
        return cls(module=module, z1=z1, z2=z2, **defaults)
