"""Spur parameter validation. No COM, no GUI.

Errors block a build; warnings let it proceed but say what is unusual. Same
contract as the bevel validator: never raises, reports everything it can find in
one pass, and early-returns only where carrying on would ask `compute_set` to
divide by zero.
"""

from __future__ import annotations

import math

from ..validate import (
    MAX_BACKLASH_FRACTION,
    MAX_PRESSURE_ANGLE,
    MIN_PRESSURE_ANGLE,
    MIN_TEETH,
    Issue,
    ValidationResult,
)
from ..involute import inv
from .geometry import (
    compute_set,
    undercut_limit,
)
from .params import ROOT_GEOMETRY_MODES, SpurSetParams

__all__ = ["Issue", "ValidationResult", "validate"]

# A helix past this is a screw gear in all but name, and the axial thrust it
# throws at the bearings stops being an afterthought.
MAX_HELIX_ANGLE = 45.0

# Below this the pair is losing contact between tooth pairs often enough to be
# noisy even though it still transmits.
MIN_COMFORTABLE_CONTACT_RATIO = 1.1

# Conservative fallback for unverified internal trimming (fighting)
# interference. The shifted geometry checks the ring tip against its base
# circle and the pinion tip against the ring root using the actual radii and
# working distance; no verified closed-form x-dependent trimming criterion is
# available in this model, so this guard remains intentionally conservative.
MIN_INTERNAL_TOOTH_DIFFERENCE = 10


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
    if p.internal and p.z2 - p.z1 < MIN_INTERNAL_TOOTH_DIFFERENCE:
        # Checked here rather than below because everything downstream divides
        # by the centre distance, which goes to zero as the counts converge.
        r.error(
            "z2",
            f"an internal pair needs the ring to have at least "
            f"{MIN_INTERNAL_TOOTH_DIFFERENCE} more teeth than the pinion; "
            f"{p.z2} - {p.z1} = {p.z2 - p.z1}",
        )
    if p.internal and p.rim_thickness < 0:
        r.error("rim_thickness", "cannot be negative")
    if p.bore < 0:
        r.error("bore", "cannot be negative")
    if p.hub_thickness < 0:
        r.error("hub_thickness", "cannot be negative")
    if p.backlash < 0:
        r.error("backlash", "cannot be negative")
    if p.fillet_factor < 0:
        r.error("fillet_factor", "cannot be negative")
    if not math.isfinite(p.profile_shift_1):
        r.error("profile_shift_1", "must be finite")
    if not math.isfinite(p.profile_shift_2):
        r.error("profile_shift_2", "must be finite")
    if not math.isfinite(p.basic_rack_addendum_factor):
        r.error("basic_rack_addendum_factor", "must be finite")
    elif p.basic_rack_addendum_factor <= 0:
        r.error("basic_rack_addendum_factor", "must be greater than zero")
    if not math.isfinite(p.basic_rack_clearance_factor):
        r.error("basic_rack_clearance_factor", "must be finite")
    elif p.basic_rack_clearance_factor < 0:
        r.error("basic_rack_clearance_factor", "cannot be negative")
    if not math.isfinite(p.basic_rack_root_radius_factor):
        r.error("basic_rack_root_radius_factor", "must be finite")
    elif p.basic_rack_root_radius_factor < 0:
        r.error("basic_rack_root_radius_factor", "cannot be negative")
    if p.root_geometry not in ROOT_GEOMETRY_MODES:
        choices = ", ".join(ROOT_GEOMETRY_MODES)
        r.error("root_geometry", f"must be one of {choices}")
    if p.working_centre_distance is not None:
        if not math.isfinite(p.working_centre_distance):
            r.error("working_centre_distance", "must be finite")
        elif p.working_centre_distance <= 0:
            r.error("working_centre_distance", "must be greater than zero")


def validate(p: SpurSetParams) -> ValidationResult:
    """Validate a parameter set. Never raises; report everything it can."""
    result = ValidationResult()
    _check_basics(p, result)
    if not result.ok:
        return result

    try:
        geo = compute_set(p)
    except ValueError as exc:
        result.error("working_centre_distance", str(exc))
        return result

    # An explicit working distance and the two x_i values are two descriptions
    # of the same pair condition.  Keep them from silently disagreeing.  The
    # inverse-involute relation is intentionally written here rather than
    # copied into params.py, where it would be an unvalidated input property.
    if p.working_centre_distance is not None:
        q = p.z2 - p.z1 if p.internal else p.z1 + p.z2
        implied_shift = q * (
            inv(geo.working_pressure_angle) - inv(geo.reference_pressure_angle)
        ) / (2.0 * math.tan(p.alpha_n))
        if not math.isclose(
            implied_shift,
            p.profile_shift_combination,
            rel_tol=0.0,
            abs_tol=1e-9,
        ):
            result.error(
                "working_centre_distance",
                f"implies profile-shift combination {implied_shift:.6g}, "
                f"but x combination is {p.profile_shift_combination:.6g}",
            )
    whole_depth = geo.whole_depth

    # --- ratio -------------------------------------------------------------
    if not (0.1 <= p.ratio <= 10.0):
        result.warn(
            "z2",
            f"ratio is {p.ratio:.2f}:1; outside 1:10 to 10:1 a single stage is "
            "rarely the right answer",
        )

    # --- backlash ----------------------------------------------------------
    if p.backlash > MAX_BACKLASH_FRACTION * geo.circular_pitch:
        result.warn(
            "backlash",
            f"{p.backlash:.3f} mm is "
            f"{p.backlash / geo.circular_pitch:.1%} of the circular pitch "
            f"({geo.circular_pitch:.2f} mm); each member loses half of it off "
            "its tooth thickness",
        )

    # --- undercut ----------------------------------------------------------
    #
    # Reported, not designed around. The legacy mode does not show the cutter
    # envelope: its root below the base circle is a radial line, not the
    # generated root a real cutter leaves. The explicit rack_generated mode
    # does show that envelope for external straight gears, but the warning is
    # still useful because it identifies a form-limited pinion and helical or
    # internal members have separate generation/interference rules.
    # An internal member is exempt: undercut is what a rack cutter does to a
    # convex flank as it rolls past, and a ring gear's flank is concave. It is
    # cut by a shaper rather than a hob, and what limits it is the tip and
    # trimming interference checked further down instead.
    z_min = undercut_limit(geo.transverse_pressure_angle, p.beta)
    for member in (geo.pinion, geo.gear):
        if not member.internal and member.z < z_min:
            form_note = (
                "the selected rack-generated root shows that form limit"
                if member.generated_root_r is not None
                else "the legacy root approximation does not show the generated undercut"
            )
            result.warn(
                "z1" if member.name == "pinion" else "z2",
                f"{member.name} has {member.z} teeth, below the undercut limit of "
                f"{z_min:.1f} for a {math.degrees(geo.transverse_pressure_angle):.1f} "
                f"degree transverse pressure angle; a real cutter would undercut "
                f"the flank near the root, and {form_note}",
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
    #
    # A ring gear has no bore to check. Its inner surface *is* the toothed one,
    # so there is nothing for a bore to be, and what stands behind its teeth is
    # the rim rather than a wall over a hole.
    min_wall = max(p.module, 1.0)
    for member in (geo.pinion, geo.gear):
        if member.internal:
            rim = p.rim_thickness
            if rim <= 0.0:
                result.error(
                    "rim_thickness",
                    "the ring has no rim outside its root circle; there is no "
                    "material behind the teeth",
                )
            elif rim < min_wall:
                result.warn(
                    "rim_thickness",
                    f"only {rim:.2f} mm of rim behind the ring's teeth; "
                    f"{min_wall:.2f} mm is the usual minimum",
                )
            continue

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

    # --- the internal pair's own failure modes -----------------------------
    if p.internal:
        _check_internal_mesh(geo, p, result)

    return result


def _check_internal_mesh(geo, p: SpurSetParams, result: ValidationResult) -> None:
    """The clearances an internal pair has and an external one does not.

    An external pair cannot foul anywhere except at the flanks, because the two
    blanks only ever approach each other. A ring gear wraps *around* its pinion,
    so there are two more places for them to meet, and neither shows up in the
    contact ratio.
    """
    pinion, ring = geo.pinion, geo.gear
    a = geo.working_centre_distance

    # --- radial clearance at the far side ----------------------------------
    #
    # The pinion's tip has to clear the ring's root all the way round, not only
    # where they mesh. Directly opposite the mesh the pinion's tip reaches
    # `a + ra1` from the ring's axis, and the ring's root circle has to be
    # outside that.
    #
    # This is an exact nominal-circle check for the far-side radial clearance.
    # For standard proportions it comes out at the standard clearance and
    # nothing else: a + ra1 = m(z2 + 2)/2 and rf2 = m(z2 + 2.5)/2, so the gap is
    # 0.25*m. With profile shift, the actual shifted tip/root radii and the
    # working centre distance are used instead.
    far_reach = a + pinion.tip_r
    if far_reach >= ring.root_r:
        result.error(
            "z2",
            f"the pinion's tip reaches {far_reach:.2f} mm from the ring's axis "
            f"but the ring's root circle is at {ring.root_r:.2f} mm, so the two "
            "collide on the far side of the mesh",
        )

    # --- trimming (fighting) interference ----------------------------------
    #
    # NOT computed here, and it would be dishonest to pretend otherwise. The
    # closed-form criterion involves the working pressure angle and both tip
    # pressure angles, and every published form of it is easy to get backwards -
    # the first version of this function shipped one that flagged the anchor
    # pair as a collision because it compared the two tip circles directly, when
    # a meshing internal pair's tip circles are *supposed* to overlap. That is
    # where the mesh is.
    #
    # What guards it instead is MIN_INTERNAL_TOOTH_DIFFERENCE, checked in
    # `_check_basics`. Ten teeth of difference is the standard rule of thumb
    # precisely because it keeps a standard-proportioned pair clear of trimming,
    # and a rule that is honest about being a rule beats a formula that might be
    # inverted. If this ever needs to be exact, the way to get it right is to
    # trace the ring's tip corner through a mesh cycle in the pinion's frame and
    # measure - not to copy an unverified criterion out of a table.


def _check_tooth_form(geo, p: SpurSetParams, result: ValidationResult) -> None:
    """Whether the space closes at the root and the tooth stays blunt at the tip."""
    from ..involute import inv, internal_tooth_width, top_land

    for member in (geo.pinion, geo.gear):
        field = "z1" if member.name == "pinion" else "z2"

        if member.internal:
            _check_internal_tooth_form(
                member, field, p, result
            )
            continue

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


def _check_internal_tooth_form(
    member, field: str, p: SpurSetParams, result: ValidationResult,
) -> None:
    """The same two questions about a ring gear, asked at the other end.

    A ring gear's tooth is narrowest at its **tip**, which is its innermost
    radius, and its space is narrowest at its **root**, which is its outermost -
    both the reverse of an external gear, because the whole tooth is turned
    inside out. So the two checks swap which radius they are evaluated at, and
    the formulae swap with them.
    """
    from ..involute import internal_space_width, internal_tooth_width

    # The flank has to *be* an involute over the whole tooth. An external gear
    # can fall below its base circle and get a radial line drawn instead, which
    # is a standard simplification; a ring gear whose tip is inside its base
    # circle has no involute flank at all and there is nothing honest to draw.
    if member.tip_r < member.base_r:
        result.error(
            field,
            f"the ring's tip radius ({member.tip_r:.2f} mm) is inside its base "
            f"circle ({member.base_r:.2f} mm), so its flank has no involute at "
            "all for the selected profile shift and rack addendum",
        )
        return

    space = internal_space_width(member.root_r, member.base_r, member.psi0)
    if space <= 0.0:
        result.error(
            field,
            "the ring's tooth space closes up at the root; the flanks cross "
            "before they reach it",
        )

    land = internal_tooth_width(
        member.tip_r, member.base_r, member.psi0, member.half_pitch
    )
    if land <= 0.0:
        result.error(
            field,
            f"the ring's teeth come to a point before the tip radius; "
            f"{member.z} teeth is too few at this pressure angle",
        )
    elif land < 0.2 * p.module:
        result.warn(
            field,
            f"the ring's top land is only {land:.3f} mm "
            f"({land / p.module:.2f} * module); the tips are nearly pointed",
        )
