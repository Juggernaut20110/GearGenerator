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
    tooth_space_section,
    undercut_limit,
)
from .params import ROOT_GEOMETRY_MODES, TIP_ALTERATION_MODES, SpurSetParams

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

# The geometry is in millimetres.  This is below the tolerances used by the
# CAD consumers and is only used to distinguish a collapsed ordering from a
# valid, very small clearance.
GEOMETRY_TOLERANCE = 1e-9
CLEARANCE_WARNING_TOLERANCE = 1e-6


def _check_basics(p: SpurSetParams, r: ValidationResult) -> None:
    """Checks that must pass before the geometry can even be computed."""
    if not _is_finite(p.module):
        r.error("module", "must be finite")
    elif p.module <= 0:
        r.error("module", "must be greater than zero")

    for field, value, label in (
        ("z1", p.z1, "pinion"),
        ("z2", p.z2, "gear"),
    ):
        if not _is_integer_tooth_count(value):
            r.error(field, f"{label} tooth count must be an integer")
        elif value < MIN_TEETH:
            r.error(field, f"{label} needs at least {MIN_TEETH} teeth")

    if not _is_finite(p.pressure_angle):
        r.error("pressure_angle", "must be finite")
    elif not (MIN_PRESSURE_ANGLE <= p.pressure_angle <= MAX_PRESSURE_ANGLE):
        r.error(
            "pressure_angle",
            f"must be between {MIN_PRESSURE_ANGLE} and {MAX_PRESSURE_ANGLE} degrees",
        )
    if not _is_finite(p.helix_angle):
        r.error("helix_angle", "must be finite")
    elif not (-MAX_HELIX_ANGLE < p.helix_angle < MAX_HELIX_ANGLE):
        r.error(
            "helix_angle",
            f"must be between -{MAX_HELIX_ANGLE} and {MAX_HELIX_ANGLE} degrees "
            "(exclusive)",
        )
    if p.hand not in ("right", "left"):
        r.error("hand", "must be 'right' or 'left'")
    for field, value in (
        ("face_width", p.face_width),
        ("bore", p.bore),
        ("hub_thickness", p.hub_thickness),
        ("backlash", p.backlash),
        ("fillet_factor", p.fillet_factor),
        ("rim_thickness", p.rim_thickness),
    ):
        if not _is_finite(value):
            r.error(field, "must be finite")
    if _is_finite(p.face_width) and p.face_width <= 0:
        r.error("face_width", "must be greater than zero")

    if (
        _is_integer_tooth_count(p.z1)
        and _is_integer_tooth_count(p.z2)
        and p.internal
        and p.z2 <= p.z1
    ):
        r.error("z2", "an internal ring must have more teeth than the pinion")
    if (
        _is_integer_tooth_count(p.z1)
        and _is_integer_tooth_count(p.z2)
        and p.internal
        and p.z2 - p.z1 < MIN_INTERNAL_TOOTH_DIFFERENCE
    ):
        # Checked here rather than below because everything downstream divides
        # by the centre distance, which goes to zero as the counts converge.
        r.error(
            "z2",
            f"an internal pair needs the ring to have at least "
            f"{MIN_INTERNAL_TOOTH_DIFFERENCE} more teeth than the pinion; "
            f"{p.z2} - {p.z1} = {p.z2 - p.z1}",
        )
    if _is_finite(p.rim_thickness) and p.internal and p.rim_thickness < 0:
        r.error("rim_thickness", "cannot be negative")
    if _is_finite(p.bore) and p.bore < 0:
        r.error("bore", "cannot be negative")
    if _is_finite(p.hub_thickness) and p.hub_thickness < 0:
        r.error("hub_thickness", "cannot be negative")
    if _is_finite(p.backlash) and p.backlash < 0:
        r.error("backlash", "cannot be negative")
    if _is_finite(p.fillet_factor) and p.fillet_factor < 0:
        r.error("fillet_factor", "cannot be negative")
    if not _is_finite(p.profile_shift_1):
        r.error("profile_shift_1", "must be finite")
    if not _is_finite(p.profile_shift_2):
        r.error("profile_shift_2", "must be finite")
    if not _is_finite(p.basic_rack_addendum_factor):
        r.error("basic_rack_addendum_factor", "must be finite")
    elif p.basic_rack_addendum_factor <= 0:
        r.error("basic_rack_addendum_factor", "must be greater than zero")
    if not _is_finite(p.basic_rack_clearance_factor):
        r.error("basic_rack_clearance_factor", "must be finite")
    elif p.basic_rack_clearance_factor < 0:
        r.error("basic_rack_clearance_factor", "cannot be negative")
    if (
        _is_finite(p.basic_rack_addendum_factor)
        and _is_finite(p.basic_rack_clearance_factor)
        and not _is_finite(p.basic_rack_dedendum_factor)
    ):
        r.error("basic_rack_clearance_factor", "gives a non-finite dedendum")
    if not _is_finite(p.basic_rack_root_radius_factor):
        r.error("basic_rack_root_radius_factor", "must be finite")
    elif p.basic_rack_root_radius_factor < 0:
        r.error("basic_rack_root_radius_factor", "cannot be negative")
    if p.root_geometry not in ROOT_GEOMETRY_MODES:
        choices = ", ".join(ROOT_GEOMETRY_MODES)
        r.error("root_geometry", f"must be one of {choices}")
    if p.tip_alteration_mode not in TIP_ALTERATION_MODES:
        choices = ", ".join(TIP_ALTERATION_MODES)
        r.error("tip_alteration_mode", f"must be one of {choices}")
    if p.tip_alteration_coefficient is not None and not _is_finite(
        p.tip_alteration_coefficient
    ):
        r.error("tip_alteration_coefficient", "must be finite")
    if (
        p.tip_alteration_mode == "explicit"
        and p.tip_alteration_coefficient is None
    ):
        r.error(
            "tip_alteration_coefficient",
            "is required when tip_alteration_mode is explicit",
        )
    if (
        p.tip_alteration_mode != "explicit"
        and p.tip_alteration_coefficient is not None
    ):
        r.error(
            "tip_alteration_coefficient",
            "is only accepted when tip_alteration_mode is explicit",
        )
    if p.working_centre_distance is not None:
        if not _is_finite(p.working_centre_distance):
            r.error("working_centre_distance", "must be finite")
        elif p.working_centre_distance <= 0:
            r.error("working_centre_distance", "must be greater than zero")


def _is_finite(value) -> bool:
    """Return whether ``value`` is a finite real number without raising."""
    try:
        return math.isfinite(value)
    except (TypeError, ValueError):
        return False


def _is_integer_tooth_count(value) -> bool:
    """Tooth counts are discrete; bools and fractional counts are invalid."""
    return isinstance(value, int) and not isinstance(value, bool)


def _check_operating_geometry(geo, result: ValidationResult) -> None:
    """Check the reference/working pair relationships used by the mesh.

    These are model invariants, not additional design limits.  In particular,
    the assembly must use ``a_w`` and ``alpha_wt`` while the reference circles
    remain fixed by ``m_t`` and tooth count.
    """
    if not _is_finite(geo.reference_centre_distance) or geo.reference_centre_distance <= 0.0:
        result.error(
            "reference_centre_distance",
            "must be a positive finite distance",
        )

    alpha_w = geo.working_pressure_angle
    if not _is_finite(alpha_w) or not (0.0 < alpha_w < math.pi / 2.0):
        result.error(
            "working_pressure_angle",
            "must be finite and strictly between 0 and 90 degrees",
        )

    working_distance = geo.working_centre_distance
    if not _is_finite(working_distance) or working_distance <= 0.0:
        result.error(
            "working_centre_distance",
            "must be a positive finite distance",
        )

    pinion, gear = geo.pinion, geo.gear
    reference_distance = (
        gear.reference_r - pinion.reference_r
        if gear.internal
        else gear.reference_r + pinion.reference_r
    )
    if not math.isclose(
        geo.reference_centre_distance,
        reference_distance,
        rel_tol=0.0,
        abs_tol=GEOMETRY_TOLERANCE,
    ):
        result.error(
            "reference_centre_distance",
            "does not match the two reference radii",
        )

    if _is_finite(working_distance):
        expected_working_distance = (
            gear.working_r - pinion.working_r
            if gear.internal
            else gear.working_r + pinion.working_r
        )
        if not math.isclose(
            working_distance,
            expected_working_distance,
            rel_tol=0.0,
            abs_tol=GEOMETRY_TOLERANCE,
        ):
            result.error(
                "working_centre_distance",
                "does not match the two working pitch radii",
            )

    if _is_finite(alpha_w) and 0.0 < alpha_w < math.pi / 2.0:
        for member in (pinion, gear):
            expected_working_r = member.base_r / math.cos(alpha_w)
            if not math.isclose(
                member.working_r,
                expected_working_r,
                rel_tol=0.0,
                abs_tol=GEOMETRY_TOLERANCE,
            ):
                field = "z1" if member.name == "pinion" else "z2"
                result.error(
                    field,
                    f"{member.name}'s working radius does not match its base "
                    "radius and working pressure angle",
                )


def validate(p: SpurSetParams) -> ValidationResult:
    """Validate a parameter set. Never raises; report everything it can."""
    result = ValidationResult()
    _check_basics(p, result)
    if not result.ok:
        return result

    try:
        geo = compute_set(p)
    except (ValueError, OverflowError, ZeroDivisionError) as exc:
        result.error("working_centre_distance", str(exc))
        return result

    _check_operating_geometry(geo, result)
    _check_tip_alteration_geometry(geo, p, result)

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
    if p.root_geometry == "rack_generated" and (
        p.internal or abs(p.beta) > GEOMETRY_TOLERANCE
    ):
        result.warn(
            "root_geometry",
            "rack-generated root geometry is currently verified only for "
            "external straight gears; the affected member uses the legacy "
            "root approximation",
        )

    for member in (geo.pinion, geo.gear):
        if member.internal:
            continue
        z_min = undercut_limit(
            geo.transverse_pressure_angle,
            member.beta,
            member.profile_shift,
            p.basic_rack_addendum_factor,
            dedendum_factor=p.basic_rack_dedendum_factor,
            root_radius_factor=p.basic_rack_root_radius_factor,
        )
        if member.z < z_min:
            form_note = (
                "the selected rack-generated root shows that form limit"
                if member.generated_root_r is not None
                else "the selected root approximation does not show the generated undercut"
            )
            result.warn(
                "z1" if member.name == "pinion" else "z2",
                f"{member.name} has {member.z} teeth, below the undercut limit of "
                f"{z_min:.1f} for a {math.degrees(geo.transverse_pressure_angle):.1f} "
                f"degree transverse pressure angle at x={member.profile_shift:.3g}; "
                f"a real cutter would undercut "
                f"the flank near the root, and {form_note}",
            )

    # --- contact ratio -----------------------------------------------------
    if geo.path_of_contact < -GEOMETRY_TOLERANCE:
        result.error(
            "z1",
            f"actual path of contact is negative: {geo.path_of_contact:.6g} mm",
        )
    if geo.contact_ratio_basis not in {"active_profile", "nominal_full_involute", "approximate"}:
        result.error(
            "z1",
            f"unknown contact-ratio basis {geo.contact_ratio_basis!r}",
        )
    eps_a = geo.transverse_contact_ratio
    if not _is_finite(eps_a) or eps_a < 0.0:
        result.error("z1", f"transverse contact ratio is invalid: {eps_a!r}")
    elif eps_a < 1.0:
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

    eps_b = geo.overlap_ratio
    if not _is_finite(eps_b) or eps_b < 0.0:
        result.error("face_width", f"overlap ratio is invalid: {eps_b!r}")

    eps_g = geo.total_contact_ratio
    if not _is_finite(eps_g) or eps_g < 0.0:
        result.error("z1", f"total contact ratio is invalid: {eps_g!r}")
    elif eps_g < 1.0:
        result.error(
            "z1",
            f"total contact ratio is {eps_g:.2f}; below 1.0 the pair has no "
            "continuous line/area of contact",
        )

    # --- helix overlap -----------------------------------------------------
    #
    # The point of a helix is that the axial overlap hands the load from one
    # tooth to the next gradually instead of all at once. Below one axial pitch
    # of face width there is no full handover, so the pair takes on the thrust
    # load a helix causes without collecting what it is for.
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


def _check_tip_alteration_geometry(geo, p: SpurSetParams, result: ValidationResult) -> None:
    """Validate ISO working depth, tip clearances, and altered tip form."""
    if not _is_finite(geo.working_depth) or geo.working_depth <= 0.0:
        result.error(
            "tip_alteration_coefficient",
            f"tip alteration gives a non-positive working depth "
            f"({geo.working_depth!r})",
        )

    warning_limit = CLEARANCE_WARNING_TOLERANCE * max(1.0, p.module)
    for field, clearance in (
        ("tip_clearance_1", geo.tip_clearance_1),
        ("tip_clearance_2", geo.tip_clearance_2),
    ):
        if not _is_finite(clearance):
            result.error(field, "is not finite")
        elif clearance < -GEOMETRY_TOLERANCE:
            result.error(
                field,
                f"is negative ({clearance:.6g} mm); tip/root interference "
                "is not supported",
            )
        elif clearance <= warning_limit:
            result.warn(
                field,
                f"is only {clearance:.6g} mm; the tip/root condition is "
                "at the numerical or manufacturing limit",
            )


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
    """Validate radii, thickness, and the actual tooth-space boundary."""
    from ..involute import inv, internal_tooth_width, top_land

    for member in (geo.pinion, geo.gear):
        field = "z1" if member.name == "pinion" else "z2"

        if not _check_member_radii(member, field, result):
            continue

        if member.reference_tooth_thickness <= GEOMETRY_TOLERANCE:
            result.error(
                field,
                f"{member.name}'s reference tooth thickness is "
                f"{member.reference_tooth_thickness:.6g} mm; it must be positive",
            )

        # These are warnings, not arbitrary x limits.  A profile shift that
        # makes a nominal addendum or dedendum non-positive can still leave an
        # ordered, drawable involute; the geometric checks below decide whether
        # the resulting tooth actually fails.
        shift_field = (
            "profile_shift_1" if member.name == "pinion" else "profile_shift_2"
        )
        if member.addendum <= GEOMETRY_TOLERANCE:
            result.error(
                shift_field,
                f"{member.name}'s nominal addendum is {member.addendum:.6g} mm; "
                "the selected profile shift and tip alteration give no "
                "positive addendum",
            )
        if member.dedendum <= GEOMETRY_TOLERANCE:
            result.warn(
                shift_field,
                f"{member.name}'s nominal dedendum is {member.dedendum:.6g} mm; "
                "the selected profile shift gives no positive dedendum",
            )

        if member.internal:
            _check_internal_tooth_form(
                geo, member, field, p, result
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
                f"{member.name}'s top land is {land:.6g} mm; the teeth come to "
                f"a point before the tip radius; "
                f"{member.z} teeth is too few at this pressure angle",
            )
        elif land < 0.2 * p.module:
            result.warn(
                field,
                f"{member.name}'s top land is only {land:.3f} mm "
                f"({land / p.module:.2f} * module); the tips are nearly pointed",
            )

        _check_tooth_space_loop(geo, member.name, field, result)


def _check_member_radii(member, field: str, result: ValidationResult) -> bool:
    """Check radius positivity and the directional root/tip ordering."""
    names = (
        ("reference radius", member.reference_r),
        ("base radius", member.base_r),
        ("tip radius", member.tip_r),
        ("root radius", member.root_r),
    )
    valid = True
    for name, radius in names:
        if not _is_finite(radius) or radius <= GEOMETRY_TOLERANCE:
            result.error(
                field,
                f"{member.name}'s {name} must be a positive finite radius; "
                f"got {radius!r}",
            )
            valid = False
    if not valid:
        return False

    if member.internal:
        if member.tip_r >= member.root_r - GEOMETRY_TOLERANCE:
            result.error(
                field,
                f"the ring's tip radius ({member.tip_r:.6g} mm) must be smaller "
                f"than its root radius ({member.root_r:.6g} mm)",
            )
    elif member.root_r >= member.tip_r - GEOMETRY_TOLERANCE:
        result.error(
            field,
            f"{member.name}'s root radius ({member.root_r:.6g} mm) must be "
            f"smaller than its tip radius ({member.tip_r:.6g} mm)",
        )

    # An external gear may use a radial below-base segment, but its tip still
    # has to reach beyond the base circle.  An internal ring has no honest
    # radial fallback: its whole generated space must contain an involute.
    if member.tip_r <= member.base_r + GEOMETRY_TOLERANCE:
        result.error(
            field,
            f"{member.name}'s tip radius ({member.tip_r:.6g} mm) does not clear "
            f"its base circle ({member.base_r:.6g} mm); there is no valid "
            "involute flank",
        )

    if member.generated_root_r is not None:
        if not _is_finite(member.generated_root_r) or member.generated_root_r <= 0.0:
            result.error(field, f"{member.name}'s generated root radius is invalid")
        elif member.generated_root_r > member.root_r + GEOMETRY_TOLERANCE:
            result.error(
                field,
                f"{member.name}'s generated root radius lies outside its nominal "
                "root radius",
            )
    if member.root_form_r is not None:
        if not _is_finite(member.root_form_r) or member.root_form_r <= 0.0:
            result.error(field, f"{member.name}'s root-form radius is invalid")
        elif member.tip_r <= member.root_form_r + GEOMETRY_TOLERANCE:
            result.error(
                field,
                f"{member.name}'s altered tip radius ({member.tip_r:.6g} mm) "
                f"does not clear its active root-form radius "
                f"({member.root_form_r:.6g} mm)",
            )
    if member.tip_form_r is not None and member.active_tip_r is not None:
        if member.internal:
            if member.active_tip_r < member.tip_form_r - GEOMETRY_TOLERANCE:
                result.error(
                    field,
                    f"{member.name}'s active tip diameter lies inside its tip "
                    f"form ({member.active_tip_r:.6g} mm radius)",
                )
        elif member.active_tip_r > member.tip_form_r + GEOMETRY_TOLERANCE:
            result.error(
                field,
                f"{member.name}'s active tip radius ({member.active_tip_r:.6g} mm) "
                "lies beyond its tip form",
            )
    if member.start_active_profile_r is not None:
        if member.internal:
            if member.active_tip_r is not None and member.start_active_profile_r < member.active_tip_r - GEOMETRY_TOLERANCE:
                result.error(field, f"{member.name}'s active root lies inside its active tip")
            if member.root_form_r is not None and member.start_active_profile_r > member.root_form_r + GEOMETRY_TOLERANCE:
                result.error(field, f"{member.name}'s active root lies beyond its root form")
        else:
            if member.root_form_r is not None and member.start_active_profile_r < member.root_form_r - GEOMETRY_TOLERANCE:
                result.error(field, f"{member.name}'s active root lies below its root form")
            if member.active_tip_r is not None and member.start_active_profile_r > member.active_tip_r + GEOMETRY_TOLERANCE:
                result.error(field, f"{member.name}'s active root lies beyond its active tip")
    return True


def _check_internal_tooth_form(
    geo, member, field: str, p: SpurSetParams, result: ValidationResult,
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
    if member.tip_r <= member.base_r + GEOMETRY_TOLERANCE:
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
            f"the ring's top land is {land:.6g} mm; the teeth come to a point "
            f"before the tip radius; "
            f"{member.z} teeth is too few at this pressure angle",
        )
    elif land < 0.2 * p.module:
        result.warn(
            field,
            f"the ring's top land is only {land:.3f} mm "
            f"({land / p.module:.2f} * module); the tips are nearly pointed",
        )

    _check_tooth_space_loop(geo, member.name, field, result)


def _check_tooth_space_loop(
    geo, member_name: str, field: str, result: ValidationResult,
) -> None:
    """Reject a collapsed, degenerate, or self-intersecting space boundary."""
    try:
        section = tooth_space_section(geo, member_name)
    except (ValueError, OverflowError, ZeroDivisionError) as exc:
        result.error(field, f"tooth space could not be generated: {exc}")
        return

    loop = section.loop_2d
    if len(loop) < 3:
        result.error(field, "tooth space has fewer than three boundary points")
        return
    if any(
        not _is_finite(coordinate)
        for point in loop
        for coordinate in point
    ):
        result.error(field, "tooth space boundary contains a non-finite point")
        return

    area2 = sum(
        a[0] * b[1] - b[0] * a[1]
        for a, b in zip(loop, loop[1:] + loop[:1])
    )
    if abs(area2) <= GEOMETRY_TOLERANCE:
        result.error(field, "tooth space boundary has zero enclosed area")
        return

    if _loop_has_self_intersection(loop):
        result.error(
            field,
            "tooth space boundary self-intersects and cannot form a valid cut",
        )


def _loop_has_self_intersection(loop) -> bool:
    """Return whether any non-adjacent closed-loop edges intersect."""
    count = len(loop)
    for first in range(count):
        a, b = loop[first], loop[(first + 1) % count]
        for second in range(first + 1, count):
            if second == first + 1 or (first == 0 and second == count - 1):
                continue
            c, d = loop[second], loop[(second + 1) % count]
            if _segments_intersect(a, b, c, d):
                return True
    return False


def _segments_intersect(a, b, c, d) -> bool:
    """Closed-segment intersection, including non-adjacent touching."""
    tolerance = GEOMETRY_TOLERANCE
    if (
        max(a[0], b[0]) < min(c[0], d[0]) - tolerance
        or max(c[0], d[0]) < min(a[0], b[0]) - tolerance
        or max(a[1], b[1]) < min(c[1], d[1]) - tolerance
        or max(c[1], d[1]) < min(a[1], b[1]) - tolerance
    ):
        return False

    def cross(o, p, q):
        return (p[0] - o[0]) * (q[1] - o[1]) - (
            p[1] - o[1]
        ) * (q[0] - o[0])

    def sign(value):
        if value > tolerance:
            return 1
        if value < -tolerance:
            return -1
        return 0

    def on_segment(o, p, q):
        return (
            abs(cross(o, p, q)) <= tolerance
            and min(o[0], p[0]) - tolerance <= q[0] <= max(o[0], p[0]) + tolerance
            and min(o[1], p[1]) - tolerance <= q[1] <= max(o[1], p[1]) + tolerance
        )

    ab_c, ab_d = sign(cross(a, b, c)), sign(cross(a, b, d))
    cd_a, cd_b = sign(cross(c, d, a)), sign(cross(c, d, b))
    if ab_c * ab_d < 0 and cd_a * cd_b < 0:
        return True
    return (
        (ab_c == 0 and on_segment(a, b, c))
        or (ab_d == 0 and on_segment(a, b, d))
        or (cd_a == 0 and on_segment(c, d, a))
        or (cd_b == 0 and on_segment(c, d, b))
    )
