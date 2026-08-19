"""User-facing input parameters for a straight bevel gear set.

Angles are in degrees here because this is what the GUI edits; everything
downstream of `geometry.compute_set` works in radians.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from ..params_io import JsonParams


@dataclass(frozen=True)
class BevelSetParams(JsonParams):
    """The nine editable inputs, plus two form factors kept out of the GUI."""

    module: float               # mm, transverse module at the outer (large) end
    z1: int                     # pinion tooth count
    z2: int                     # gear tooth count
    face_width: float           # mm, along the pitch cone
    bore: float                 # mm, diameter
    hub_thickness: float        # mm, backing behind the outer root point

    # Axial rim kept behind the outer root point, before the flat back begins.
    #
    # Without it the flat back passes exactly through that point, so the material
    # under the tooth root tapers to nothing at the heel. The wedge it leaves has
    # an included angle of 90 - root_angle, which is a harmless 70 degrees on a
    # 17-tooth pinion but a 25 degree feather edge on its 43-tooth mate - the
    # bigger the ratio, the thinner the gear's heel. Setting this to zero
    # restores the old outline exactly.
    min_root_thickness: float = 0.5    # mm, measured along the axis

    pressure_angle: float = 20.0   # degrees
    shaft_angle: float = 90.0      # degrees

    # Not exposed in the v1 GUI, but part of the geometry.
    fillet_factor: float = 0.2  # root fillet radius as a multiple of module
    backlash: float = 0.0       # mm, circular backlash removed from tooth thickness

    # --- radian accessors, so downstream code never repeats the conversion ---

    @property
    def alpha(self) -> float:
        """Pressure angle in radians."""
        return math.radians(self.pressure_angle)

    @property
    def sigma(self) -> float:
        """Shaft angle in radians."""
        return math.radians(self.shaft_angle)

    @property
    def ratio(self) -> float:
        return self.z2 / self.z1

    @classmethod
    def with_defaults(cls, module: float, z1: int, z2: int, **overrides):
        """Build a set with sensible face width / bore / hub for the given size.

        Face width follows the usual bevel limit of min(Ao/3, 10*m); the bore,
        hub and root rim are rules of thumb that keep the blank manufacturable.
        Any of them can be overridden.
        """
        # Ao needs the pitch angle, which needs the shaft angle - resolve it here
        # rather than importing geometry (which would be a circular import).
        sigma = math.radians(overrides.get("shaft_angle", 90.0))
        delta1 = math.atan2(math.sin(sigma), z2 / z1 + math.cos(sigma))
        outer_cone_dist = module * z1 / (2.0 * math.sin(delta1))

        defaults = {
            "face_width": round(min(outer_cone_dist / 3.0, 10.0 * module), 2),
            "bore": round(max(0.25 * module * z1, 4.0), 1),
            "hub_thickness": round(2.5 * module, 2),
            "min_root_thickness": round(0.25 * module, 2),
        }
        defaults.update(overrides)
        return cls(module=module, z1=z1, z2=z2, **defaults)

    # Preset save/load for the GUI comes from JsonParams.
