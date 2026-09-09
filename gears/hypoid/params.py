"""User-facing parameters for a Gleason/ISO Method 1 hypoid pair.

Lengths are millimetres and angles are degrees at this boundary.  The module
is the outer transverse module of the wheel, matching the published Method 1
input convention (``d_e2 = module * z2``).  The solver exposes the calculated
mean normal module in its result rather than pretending it is an input.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from ..params_io import JsonParams


# CAD-only convention for the approximate Tredgold section.  This is not the
# cutter-head radius and cannot represent a cutter blade edge without the
# missing machine/cutter data.
HYPOID_DEFAULT_ROOT_FILLET_FACTOR = 0.1
MAX_SPIRAL_ANGLE = 60.0


def _hand_sign(hand: str) -> float:
    """Return the sole public orientation sign for a hypoid spiral."""
    if hand == "right":
        return 1.0
    if hand == "left":
        return -1.0
    raise ValueError("hand must be 'right' or 'left'")


@dataclass(frozen=True)
class HypoidSetParams(JsonParams):
    """Inputs for one external hypoid pair.

    The advanced factors are the Method 1 data-type-II inputs used by the
    published anchor.  They remain fields so a saved preset can retain a
    cutter/design choice without changing the basic GUI surface.  Lengths are
    millimetres, tooth counts are dimensionless integers, and all public
    angles are degrees.  ``backlash`` is specifically ISO outer transverse
    backlash ``j_et2`` at the wheel outer pitch cone; it is not mean-normal
    backlash.  ``spiral_angle`` is a non-negative pinion mean spiral-angle
    magnitude from 0 through 60 degrees.  ``hand`` is its sole orientation
    control; right hand is positive in signed ``psi1`` and left hand is
    negative.
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
    spiral_angle: float = 35.0    # pinion mean spiral-angle magnitude, degrees
    hand: str = "right"
    cutter_radius: float | None = None
    # Compatibility name retained for the CLI/GUI and saved JSON schema.
    # For hypoids this value is specifically the ISO outer transverse
    # backlash allowance j_et2, measured at the wheel outer pitch cone.
    backlash: float = 0.0
    min_root_thickness: float = 0.5

    # ISO Method 1 data-type-I factors.  These are intentionally editable in
    # JSON/API use but are not put in the first GUI pass.
    # Type-II inputs.  ``gear_mean_addendum_factor`` is c_ham, not a profile
    # shift.  ISO converts it to the type-I profile-shift coefficient before
    # calculating the member addenda.
    gear_mean_addendum_factor: float = 0.35  # c_ham
    depth_factor: float = 2.0               # k_d
    clearance_factor: float = 0.125         # k_c
    thickness_factor: float = 0.10    # k_t
    gear_addendum_angle: float = 1.0  # Method 1 theta_a2, degrees
    gear_dedendum_angle: float = 4.0  # Method 1 theta_f2, degrees

    # Kept after the pre-existing fields so positional construction of the
    # advanced Method 1 inputs remains compatible.  This is an approximate
    # circular tooth-root fillet in the developed Tredgold section, not cutter
    # blade geometry.  None selects the documented module-based default.
    root_fillet_radius: float | None = None

    @property
    def alpha(self) -> float:
        return math.radians(self.pressure_angle)

    @property
    def sigma(self) -> float:
        return math.radians(self.shaft_angle)

    @property
    def spiral_sign(self) -> float:
        """Signed orientation selected by ``hand`` (right positive)."""
        return _hand_sign(self.hand)

    @property
    def spiral_angle_radians(self) -> float:
        """Non-negative public spiral magnitude converted to radians."""
        if not math.isfinite(self.spiral_angle) or not (
            0.0 <= self.spiral_angle <= MAX_SPIRAL_ANGLE
        ):
            raise ValueError(
                "spiral_angle must be a finite non-negative magnitude between "
                f"0 and {MAX_SPIRAL_ANGLE} degrees"
            )
        return math.radians(self.spiral_angle)

    @property
    def psi1(self) -> float:
        """Signed pinion mean spiral angle; ``hand`` supplies its sign."""
        return self.spiral_sign * self.spiral_angle_radians

    @property
    def beta(self) -> float:
        """Signed pinion spiral angle in radians."""
        return self.psi1

    @classmethod
    def _migrate_json_data(cls, data: dict):
        """Canonicalize the historical signed hypoid spiral field.

        Before the public convention was made explicit, a negative
        ``spiral_angle`` was accepted and its sign participated in ``psi1``.
        At this JSON boundary only, preserve that old signed orientation by
        taking the magnitude and reversing the stored hand.  New direct/API
        construction is not migrated and is rejected by validation/the
        radians property.
        """
        migrated = dict(data)
        angle = migrated.get("spiral_angle")
        hand = migrated.get("hand", "right")
        if (
            isinstance(angle, (int, float))
            and math.isfinite(angle)
            and angle < 0.0
            and hand in ("right", "left")
        ):
            migrated["spiral_angle"] = abs(angle)
            migrated["hand"] = "left" if hand == "right" else "right"
        return migrated

    @property
    def hypoid_offset(self) -> float:
        """Descriptive alias for callers that prefer the full term."""
        return self.offset

    @property
    def outer_transverse_backlash(self) -> float:
        """ISO ``j_et2``; the convention of the public ``backlash`` input."""
        return self.backlash

    @property
    def effective_root_fillet_radius(self) -> float:
        """Approximate Tredgold root fillet radius used by section building.

        This is not a cutter blade-edge radius.  A true hypoid root fillet
        requires cutter blade and machine geometry, which this model does not
        contain.  ``None`` selects the documented CAD-only default of 0.1
        times the input outer transverse module.
        """
        if self.root_fillet_radius is None:
            return HYPOID_DEFAULT_ROOT_FILLET_FACTOR * self.module
        return self.root_fillet_radius

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
