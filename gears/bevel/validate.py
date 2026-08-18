"""Parameter validation. No COM, no GUI.

Errors block a build; warnings let it proceed but say what is unusual. The
split matters because plenty of perfectly buildable bevel sets are outside
textbook proportions, and the tool should not refuse to draw them.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from .geometry import compute_set
from .params import BevelSetParams

# Practical bounds. Deliberately generous - these are sanity rails, not a
# design standard.
MIN_TEETH = 6
MIN_PRESSURE_ANGLE = 14.5
MAX_PRESSURE_ANGLE = 25.0


@dataclass(frozen=True)
class Issue:
    field: str
    message: str

    def __str__(self) -> str:
        return f"{self.field}: {self.message}"


@dataclass
class ValidationResult:
    errors: list[Issue] = field(default_factory=list)
    warnings: list[Issue] = field(default_factory=list)

    @property
    def ok(self) -> bool:
        return not self.errors

    def error(self, field_: str, message: str) -> None:
        self.errors.append(Issue(field_, message))

    def warn(self, field_: str, message: str) -> None:
        self.warnings.append(Issue(field_, message))


def _check_basics(p: BevelSetParams, r: ValidationResult) -> None:
    """Checks that must pass before the geometry can even be computed."""
    if p.module <= 0:
        r.error("module", "must be greater than zero")
    if p.z1 < MIN_TEETH:
        r.error("z1", f"pinion needs at least {MIN_TEETH} teeth")
    if p.z2 < MIN_TEETH:
        r.error("z2", f"gear needs at least {MIN_TEETH} teeth")
    if not (MIN_PRESSURE_ANGLE <= p.pressure_angle <= MAX_PRESSURE_ANGLE):
        r.error(
            "pressure_angle",
            f"must be between {MIN_PRESSURE_ANGLE} and {MAX_PRESSURE_ANGLE} degrees",
        )
    if not (0.0 < p.shaft_angle < 180.0):
        r.error("shaft_angle", "must be between 0 and 180 degrees (exclusive)")
    if p.face_width <= 0:
        r.error("face_width", "must be greater than zero")
    if p.bore < 0:
        r.error("bore", "cannot be negative")
    if p.hub_thickness < 0:
        r.error("hub_thickness", "cannot be negative")
    if p.min_root_thickness < 0:
        r.error("min_root_thickness", "cannot be negative")


def validate(p: BevelSetParams) -> ValidationResult:
    """Validate a parameter set. Never raises; report everything it can."""
    result = ValidationResult()
    _check_basics(p, result)
    if not result.ok:
        return result

    geo = compute_set(p)

    # --- cone angle sanity -------------------------------------------------
    for member in (geo.pinion, geo.gear):
        if member.pitch_angle <= 0.0:
            result.error(
                "shaft_angle",
                f"{member.name} pitch cone angle is {member.pitch_angle_deg:.2f} deg; "
                "this shaft angle and ratio do not form a valid bevel pair",
            )
        elif member.pitch_angle >= math.pi / 2.0:
            result.error(
                "shaft_angle",
                f"{member.name} pitch cone angle is {member.pitch_angle_deg:.2f} deg, "
                "which is an internal or crown bevel - not supported in v1",
            )
    if not result.ok:
        return result

    # --- face width --------------------------------------------------------
    if p.face_width >= geo.outer_cone_dist:
        result.error(
            "face_width",
            f"must be less than the outer cone distance "
            f"({geo.outer_cone_dist:.2f} mm)",
        )
        return result

    face_limit = min(geo.outer_cone_dist / 3.0, 10.0 * p.module)
    if p.face_width > face_limit:
        result.warn(
            "face_width",
            f"{p.face_width:.2f} mm exceeds the usual limit of "
            f"min(Ao/3, 10*m) = {face_limit:.2f} mm; the teeth get very small "
            "at the inner end",
        )

    # --- bore vs. the material actually available --------------------------
    min_wall = max(p.module, 1.0)
    for member in (geo.pinion, geo.gear):
        inner_root_r = (
            geo.section_scale * member.virtual_root_r * math.cos(member.pitch_angle)
        )
        if p.bore / 2.0 + min_wall >= inner_root_r:
            result.error(
                "bore",
                f"{p.bore:.2f} mm leaves no wall under the {member.name} teeth "
                f"(root radius at the inner end is {inner_root_r:.2f} mm; "
                f"need {min_wall:.2f} mm of wall)",
            )

    # --- undercut ----------------------------------------------------------
    undercut_limit = 2.0 / math.sin(p.alpha) ** 2
    for member in (geo.pinion, geo.gear):
        if member.virtual_teeth < undercut_limit:
            result.warn(
                member.name,
                f"virtual tooth count {member.virtual_teeth:.1f} is below the "
                f"undercut limit of {undercut_limit:.1f} at "
                f"{p.pressure_angle:g} deg; the flank will be undercut near the root",
            )

    # --- proportions -------------------------------------------------------
    ratio = geo.params.ratio
    if not (0.1 <= ratio <= 10.0):
        result.warn(
            "z2",
            f"gear ratio {ratio:.2f}:1 is outside the usual 1:10 to 10:1 range "
            "for straight bevels",
        )

    # --- material under the teeth at the heel ------------------------------
    # With no root rim the flat back runs straight through the outer root point,
    # so the rim under the teeth tapers to a knife edge whose included angle is
    # 90 - root_angle. Steep is harmless; shallow is a feather, and it gets
    # shallower the further the ratio is from 1:1.
    for member in (geo.pinion, geo.gear):
        wedge = 90.0 - member.root_angle_deg
        if p.min_root_thickness <= 0.0 and wedge < 45.0:
            result.warn(
                "min_root_thickness",
                f"the {member.name}'s root cone meets its back face at "
                f"{wedge:.1f} deg, so the material under the teeth tapers to "
                "nothing at the heel; give it a root rim",
            )

    if p.hub_thickness < geo.whole_depth:
        result.warn(
            "hub_thickness",
            f"{p.hub_thickness:.2f} mm is less than the whole tooth depth "
            f"({geo.whole_depth:.2f} mm); there may be too little backing "
            "behind the root cone",
        )

    # --- does a tooth space actually exist at the root? --------------------
    # Evaluate at the root circle, not the base circle. On a gear with many
    # virtual teeth the root sits *above* the base circle, where the space is
    # comfortably open even though extrapolating down to the base circle would
    # give a negative width.
    for member in (geo.pinion, geo.gear):
        from .geometry import inv, top_land  # keeps the module import-light

        half_pitch = math.pi / member.virtual_teeth
        psi0 = (math.pi * p.module / 2.0 - p.backlash / 2.0) / (
            2.0 * member.virtual_pitch_r
        ) + inv(p.alpha)
        r_eval = max(member.virtual_root_r, member.virtual_base_r)
        alpha_r = math.acos(min(1.0, member.virtual_base_r / r_eval))
        if half_pitch - psi0 + inv(alpha_r) <= 0.0:
            result.error(
                member.name,
                "tooth space closes up at the root; reduce the module or "
                "increase the tooth count",
            )

        # --- pointed teeth ------------------------------------------------
        # The Gleason long addendum lands the pinion close to pointed on
        # low-tooth-count, high-ratio sets. Zero top land is unbuildable;
        # a thin one is buildable but fragile and worth saying out loud.
        land = top_land(member.virtual_tip_r, member.virtual_base_r, psi0)
        if land <= 0.0:
            result.error(
                member.name,
                f"teeth come to a point before the tip ({member.z} teeth, "
                f"addendum {member.addendum:.2f} mm); increase the tooth count "
                "or reduce the module",
            )
        elif land < 0.2 * p.module:
            result.warn(
                member.name,
                f"top land is only {land:.2f} mm ({land / p.module:.2f} x "
                "module); the teeth are nearly pointed",
            )

    return result
