"""Spur parameter validation. No COM, no GUI.

Errors block a build; warnings let it proceed but say what is unusual. Same
contract as the bevel validator: never raises, reports everything it can find in
one pass, and early-returns only where carrying on would ask `compute_set` to
divide by zero.
"""

from __future__ import annotations

import math

from ..validate import (
    MAX_PRESSURE_ANGLE,
    MIN_PRESSURE_ANGLE,
    MIN_TEETH,
    Issue,
    ValidationResult,
)
from .geometry import WHOLE_DEPTH_FACTOR, compute_set, undercut_limit
from .params import SpurSetParams

__all__ = ["Issue", "ValidationResult", "validate"]

# A helix past this is a screw gear in all but name, and the axial thrust it
# throws at the bearings stops being an afterthought.
MAX_HELIX_ANGLE = 45.0

# Below this the pair is losing contact between tooth pairs often enough to be
# noisy even though it still transmits.
MIN_COMFORTABLE_CONTACT_RATIO = 1.1


def _check_basics(p: SpurSetParams, r: ValidationResult) -> None:
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
    if not (-MAX_HELIX_ANGLE < p.helix_angle < MAX_HELIX_ANGLE):
        r.error(
            "helix_angle",
            f"must be between -{MAX_HELIX_ANGLE} and {MAX_HELIX_ANGLE} degrees "
            "(exclusive)",
        )
    if p.hand not in ("right", "left"):
        r.error("hand", "must be 'right' or 'left'")
    if p.face_width <= 0:
        r.error("face_width", "must be greater than zero")
    if p.bore < 0:
        r.error("bore", "cannot be negative")
    if p.hub_thickness < 0:
        r.error("hub_thickness", "cannot be negative")
    if p.backlash < 0:
        r.error("backlash", "cannot be negative")
    if p.fillet_factor < 0:
        r.error("fillet_factor", "cannot be negative")


def validate(p: SpurSetParams) -> ValidationResult:
    """Validate a parameter set. Never raises; report everything it can."""
    result = ValidationResult()
    _check_basics(p, result)
    if not result.ok:
        return result

    geo = compute_set(p)
    whole_depth = WHOLE_DEPTH_FACTOR * p.module

    # --- ratio -------------------------------------------------------------
    if not (0.1 <= p.ratio <= 10.0):
        result.warn(
            "z2",
            f"ratio is {p.ratio:.2f}:1; outside 1:10 to 10:1 a single stage is "
            "rarely the right answer",
        )

    # --- undercut ----------------------------------------------------------
    #
    # Reported, not designed around. Without a profile shift there is nothing
    # the geometry can do about it, and the model will not even *show* it: the
    # root below the base circle is drawn as a radial line, not as the trochoid
    # a real cutter leaves. So a part that undercuts in reality comes out of
    # here looking sound, which is exactly why this warning has to be loud.
    z_min = undercut_limit(geo.transverse_pressure_angle, p.beta)
    for member in (geo.pinion, geo.gear):
        if member.z < z_min:
            result.warn(
                "z1" if member.name == "pinion" else "z2",
                f"{member.name} has {member.z} teeth, below the undercut limit of "
                f"{z_min:.1f} for a {math.degrees(geo.transverse_pressure_angle):.1f} "
                "degree transverse pressure angle; a real cutter would undercut "
                "the flank near the root, which this model does not show",
            )

    # --- contact ratio -----------------------------------------------------
    eps_a = geo.transverse_contact_ratio
    if eps_a < 1.0:
        result.error(
            "z1",
            f"transverse contact ratio is {eps_a:.2f}; below 1.0 the pair loses "
            "contact between tooth pairs and cannot transmit continuous motion",
        )
    elif eps_a < MIN_COMFORTABLE_CONTACT_RATIO:
        result.warn(
            "z1",
            f"transverse contact ratio is only {eps_a:.2f}; a single tooth pair "
            "carries the load for most of the mesh",
        )

    # --- helix overlap -----------------------------------------------------
    #
    # The point of a helix is that the axial overlap hands the load from one
    # tooth to the next gradually instead of all at once. Below one axial pitch
    # of face width there is no full handover, so the pair takes on the thrust
    # load a helix causes without collecting what it is for.
    eps_b = geo.axial_contact_ratio
    if 0.0 < eps_b < 1.0:
        result.warn(
            "face_width",
            f"axial contact ratio is {eps_b:.2f}; a face width of at least one "
            f"axial pitch ({geo.axial_pitch:.2f} mm) is what makes a helix worth "
            "its thrust load",
        )

    # --- blank ------------------------------------------------------------
    min_wall = max(p.module, 1.0)
    for member in (geo.pinion, geo.gear):
        wall = member.root_r - p.bore / 2.0
        if wall <= 0.0:
            result.error(
                "bore",
                f"bore is wider than {member.name}'s root circle "
                f"({2.0 * member.root_r:.2f} mm); there is no material under the teeth",
            )
        elif wall < min_wall:
            result.warn(
                "bore",
                f"only {wall:.2f} mm of wall under {member.name}'s teeth; "
                f"{min_wall:.2f} mm is the usual minimum",
            )

    if 0.0 < p.hub_thickness < whole_depth:
        result.warn(
            "hub_thickness",
            f"hub is {p.hub_thickness:.2f} mm, less than the whole tooth depth "
            f"({whole_depth:.2f} mm); it will barely register as a boss",
        )

    # --- face width --------------------------------------------------------
    #
    # The usual band. Narrow is merely weak, but wide is a real problem: load
    # spreads unevenly across a face that cannot be held parallel, and past
    # about 15 * m the misalignment costs more than the extra width buys.
    ratio_b_m = p.face_width / p.module
    if ratio_b_m < 3.0:
        result.warn(
            "face_width",
            f"face width is {ratio_b_m:.1f} * module; under 3 the teeth carry "
            "very little load for their size",
        )
    elif ratio_b_m > 15.0:
        result.warn(
            "face_width",
            f"face width is {ratio_b_m:.1f} * module; past about 15 the load "
            "will not spread evenly across a face this wide",
        )

    # --- tooth form --------------------------------------------------------
    _check_tooth_form(geo, p, result)

    return result


def _check_tooth_form(geo, p: SpurSetParams, result: ValidationResult) -> None:
    """Whether the space closes at the root and the tooth stays blunt at the tip."""
    from ..involute import inv, top_land

    for member in (geo.pinion, geo.gear):
        field = "z1" if member.name == "pinion" else "z2"

        # Does the tooth space still have width where the flanks meet the root?
        # Evaluated at max(root, base) deliberately: below the base circle the
        # flank is a radial line, so the narrowest *involute* point is the base
        # circle and asking about anything inside it is asking about a fiction.
        r = max(member.root_r, member.base_r)
        alpha_r = math.acos(min(1.0, member.base_r / r))
        space_half_width = member.half_pitch - member.psi0 + inv(alpha_r)
        if space_half_width <= 0.0:
            result.error(
                field,
                f"{member.name}'s tooth space closes up at the root; the flanks "
                "cross before they reach it",
            )

        land = top_land(member.tip_r, member.base_r, member.psi0)
        if land <= 0.0:
            result.error(
                field,
                f"{member.name}'s teeth come to a point before the tip radius; "
                f"{member.z} teeth is too few at this pressure angle",
            )
        elif land < 0.2 * p.module:
            result.warn(
                field,
                f"{member.name}'s top land is only {land:.3f} mm "
                f"({land / p.module:.2f} * module); the tips are nearly pointed",
            )
