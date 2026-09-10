"""How the two spur members sit relative to each other. Pure math - no COM.

Coordinate system, matching `geometry`: each part is built with its own axis
along +Z, its front face at z = 0, and a tooth **space** centred on the part's
+X direction at z = 0.

In the assembly the pinion keeps that frame unchanged. The gear is **translated**
along +X by the working centre distance and not rotated at all - its axis stays parallel
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

An internal pair
----------------
Still a pure translation along X with no rotation, at the working distance
`a_w` (the reference value is `m_t (z2 - z1) / 2`) rather than the external
sum, because the pinion runs *inside* the ring. Three things change,
and the third is the one that bites.

**The hands agree** rather than opposing. The rule has not been reversed; the
reason it produced opposite hands for an external pair is that the two members
face each other, and here the ring wraps around the pinion and faces the same
way. Still no flip in the placement.

**The pair turns the same way.** `angular_velocity_ratio` carries the sign and
it is positive for an internal pair - which is what a planetary train is built
out of.

**The ring goes to -a_w, not +a_w**, and this is not a free choice. Internally
tangent pitch circles touch on the far side of the small one from the large
one's centre: with the pinion at the origin and the ring centre at (-a_w, 0), the
contact point is at (+r_p1, 0), because r_p1 + a_w = r_p2. Put the ring at (+a_w, 0)
instead and the contact lands at (-r_p1, 0) - **angle pi in the pinion's frame**,
where the pinion has whatever its tooth count happens to put there rather than
the tooth space it is built with at angle 0. The pinion would then need clocking
too, and it is the one member in this whole codebase that never does.

So the contact sits at angle **0** in both frames, and the clocking follows from
that: the pinion presents its space there by construction, so the ring must
present a tooth *centre* at its own angle 0. Its spaces are at k*tau and its
tooth centres at (k + 1/2)*tau, so the rotation is half an angular pitch, with
no parity to think about - see `internal_gear_clocking`. That is a different
answer from the external pair's `gear_clocking`, which aims at pi instead, and
the two are not interchangeable.
"""

from __future__ import annotations

import math

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
    "clocking_for",
    "gear_clocking",
    "gear_mate_ratio",
    "gear_placement",
    "gear_translation",
    "internal_gear_clocking",
    "pinion_placement",
    "to_array_data",
]


def gear_placement(clocking: float) -> Matrix3:
    """Spin the gear about its own axis. That is the whole rotation."""
    return rot_z(clocking)


def gear_translation(geo: SpurSetGeometry) -> tuple[float, float, float]:
    """Where the gear's origin goes: along X by the working distance ``a_w``.

    **Positive X for an external pair, negative for an internal one.** See the
    module docstring: internally tangent pitch circles touch on the far side of
    the pinion from the ring's centre, so putting the ring at -a_w is what keeps
    the contact point at the pinion's angle 0, where its tooth space already is.

    Millimetres, like everything else upstream of `gears.sw`. `to_array_data`
    wants metres, so the caller in the SOLIDWORKS layer maps `mm()` over this -
    the unit conversion stays at that boundary and nowhere else.
    """
    sign = -1.0 if geo.params.internal else 1.0
    return (sign * geo.working_centre_distance, 0.0, 0.0)


def internal_gear_clocking(z: int) -> float:
    """Rotation putting one of the ring's tooth centres at its own angle 0.

    The internal counterpart of `placement.gear_clocking`, and a different
    answer rather than the same one reached differently. The contact line sits
    at angle **0** in the ring's frame - not pi - because the ring is placed at
    -a_w; and the ring is built with a tooth *space* centred on angle 0, so what
    is wanted is the nearest tooth centre brought to 0.

    Tooth centres sit at (k + 1/2) * tau, so the nearest is half a pitch away
    whatever the tooth count. No parity case, and none of the floating-point
    fragility `gear_clocking` has to work around - that comes from `pi` not
    being a whole number of pitches, and 0 always is.
    """
    return math.pi / z


def clocking_for(geo: SpurSetGeometry) -> float:
    """The clocking the gear needs, whichever kind of pair this is."""
    if geo.params.internal:
        return internal_gear_clocking(geo.gear.z)
    return gear_clocking(geo.gear.z)
