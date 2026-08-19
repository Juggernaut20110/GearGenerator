"""User-facing input parameters for an involute spur gear set.

Angles are in degrees here because this is what the GUI edits; everything
downstream of `geometry.compute_set` works in radians.

Normal or transverse?
---------------------
`module` and `pressure_angle` are the **normal** values - measured in the plane
perpendicular to the tooth, which is the plane a hob or a cutter works in. That
is the convention every catalogue and every cutter is sold under, and it is what
makes a helical gear cuttable with the same tool as a straight one of the same
normal module.

The involute itself, though, lives in the **transverse** plane - the plane
perpendicular to the axis, which is where a section through the part is taken.
So the geometry works throughout in

    m_t     = m_n / cos(beta)
    alpha_t = atan(tan(alpha_n) / cos(beta))

and both are exposed here so nothing downstream repeats the conversion. At
beta = 0 they collapse to the normal values and a straight spur gear falls out
of the same code with no special case.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from ..params_io import JsonParams


@dataclass(frozen=True)
class SpurSetParams(JsonParams):
    """The seven editable inputs, plus two form factors kept out of the GUI."""

    module: float               # mm, NORMAL module
    z1: int                     # pinion tooth count
    z2: int                     # gear tooth count
    face_width: float           # mm, along the axis
    bore: float                 # mm, diameter
    hub_thickness: float        # mm, boss behind the back face

    pressure_angle: float = 20.0   # degrees, NORMAL
    helix_angle: float = 0.0       # degrees; 0 is a straight spur gear

    # Which way the PINION's teeth wind, looking along +Z. The gear always takes
    # the opposite hand: the two are placed on parallel axes by a pure
    # translation with no flip, so same-hand teeth would cross rather than mesh.
    hand: str = "right"

    # Not exposed in the v1 GUI, but part of the geometry.
    fillet_factor: float = 0.2  # root fillet radius as a multiple of module
    backlash: float = 0.0       # mm, circular backlash removed from tooth thickness

    # --- radian and transverse accessors, so downstream code never repeats them ---

    @property
    def alpha_n(self) -> float:
        """Normal pressure angle in radians."""
        return math.radians(self.pressure_angle)

    @property
    def beta(self) -> float:
        """Helix angle in radians, signed by hand: right-hand is positive."""
        magnitude = math.radians(self.helix_angle)
        return magnitude if self.hand == "right" else -magnitude

    @property
    def transverse_module(self) -> float:
        return self.module / math.cos(self.beta)

    @property
    def alpha_t(self) -> float:
        """Transverse pressure angle in radians - the involute's own angle."""
        return math.atan2(math.tan(self.alpha_n), math.cos(self.beta))

    @property
    def ratio(self) -> float:
        return self.z2 / self.z1

    @property
    def centre_distance(self) -> float:
        """Standard centre distance, mm. No profile shift, so this is exact."""
        return self.transverse_module * (self.z1 + self.z2) / 2.0

    @classmethod
    def with_defaults(cls, module: float, z1: int, z2: int, **overrides):
        """Build a set with sensible face width / bore / hub for the given size.

        Face width is the usual 10 * m_n for straight teeth. For helical teeth it
        is at least one **axial pitch** as well, `pi * m_n / sin(beta)`, so the
        axial contact ratio reaches 1: below that the pair pays the thrust-load
        cost of a helix without buying the overlapping handover that is the whole
        point of one. The bore and hub are rules of thumb. Any of them can be
        overridden.
        """
        beta = math.radians(overrides.get("helix_angle", 0.0))
        face_width = 10.0 * module
        if abs(math.sin(beta)) > 1e-9:
            face_width = max(face_width, math.pi * module / abs(math.sin(beta)))

        # The bore is sized against the transverse pitch diameter, which is the
        # one the blank is actually as big as.
        m_t = module / math.cos(beta)

        defaults = {
            "face_width": round(face_width, 2),
            "bore": round(max(0.25 * m_t * z1, 4.0), 1),
            "hub_thickness": round(2.5 * module, 2),
        }
        defaults.update(overrides)
        return cls(module=module, z1=z1, z2=z2, **defaults)
