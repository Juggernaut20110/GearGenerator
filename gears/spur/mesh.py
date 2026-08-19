"""How the two spur members sit relative to each other. Pure math - no COM.

Coordinate system, matching `geometry`: each part is built with its own axis
along +Z, its front face at z = 0, and a tooth **space** centred on the part's
+X direction at z = 0.

In the assembly the pinion keeps that frame unchanged. The gear is **translated**
along +X by the centre distance and not rotated at all - its axis stays parallel
to the pinion's and pointing the same way. That is the whole arrangement, and it
is what makes the spur case so much shorter than the bevel one: no shaft angle,
no shared apex, no tilt.

Two consequences worth stating, because both are easy to get wrong:

* the contact point sits on the line of centres, at (r_p1, 0). In the pinion's
  frame that is angle 0, where its tooth space already is; in the gear's frame
  it is at angle **pi**, looking back toward the pinion. So the gear needs the
  same clocking a bevel gear needs, by an entirely different argument - see
  `placement.gear_clocking`.
* the two parts must be cut with **opposite hands**. There is no flip anywhere
  in the placement, so same-hand helices would cross instead of meshing.
  `geometry.compute_set` signs each member's twist for this.
"""

from __future__ import annotations

from ..placement import (
    Matrix3,
    angle_between,
    angular_velocity_ratio,
    apply,
    axis_of,
    gear_clocking,
    gear_mate_ratio,
    pinion_placement,
    rot_z,
    to_array_data,
)
from .geometry import SpurSetGeometry

__all__ = [
    "Matrix3",
    "angle_between",
    "angular_velocity_ratio",
    "apply",
    "axis_of",
    "gear_clocking",
    "gear_mate_ratio",
    "gear_placement",
    "gear_translation",
    "pinion_placement",
    "to_array_data",
]


def gear_placement(clocking: float) -> Matrix3:
    """Spin the gear about its own axis. That is the whole rotation."""
    return rot_z(clocking)


def gear_translation(geo: SpurSetGeometry) -> tuple[float, float, float]:
    """Where the gear's origin goes: along +X by the centre distance. mm.

    Millimetres, like everything else upstream of `gears.sw`. `to_array_data`
    wants metres, so the caller in the SOLIDWORKS layer maps `mm()` over this -
    the unit conversion stays at that boundary and nowhere else.
    """
    return (geo.centre_distance, 0.0, 0.0)
