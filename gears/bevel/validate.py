"""Parameter validation. No COM, no GUI.

Errors block a build; warnings let it proceed but say what is unusual. The
split matters because plenty of perfectly buildable bevel sets are outside
textbook proportions, and the tool should not refuse to draw them.
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
from .geometry import compute_set, phase_at_cone_distance, section_count
from .params import BevelSetParams

__all__ = ["Issue", "ValidationResult", "validate"]

# Past this a spiral bevel gear is throwing more axial thrust at its bearings
# than the smoother mesh is worth. 35 is the Gleason standard and the usual
# answer; the band either side of it is where design choices actually live.
MAX_SPIRAL_ANGLE = 45.0
HIGH_SPIRAL_ANGLE = 40.0

# The Gleason face-width limit for a spiral set, as a fraction of Ao. Tighter
# than the straight limit of a third, because a spiral tooth's contact sweeps
# along the face as well as up the profile: the toe end of too wide a face
# carries load at a spiral angle well away from the one it was designed at.
SPIRAL_FACE_FRACTION = 0.30


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
    if p.backlash < 0:
        r.error("backlash", "cannot be negative")
    if not (-MAX_SPIRAL_ANGLE <= p.spiral_angle <= MAX_SPIRAL_ANGLE):
        r.error(
            "spiral_angle",
            f"must be between -{MAX_SPIRAL_ANGLE} and {MAX_SPIRAL_ANGLE} degrees",
        )
    if p.hand not in ("right", "left"):
        r.error("hand", "must be 'right' or 'left'")
    if p.cutter_radius is not None and p.cutter_radius <= 0:
        r.error("cutter_radius", "must be greater than zero")


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

    # A curved tooth gets the tighter Gleason limit; see SPIRAL_FACE_FRACTION.
    cone_fraction, cone_label = (
        (SPIRAL_FACE_FRACTION, "0.30*Ao") if p.is_curved else (1.0 / 3.0, "Ao/3")
    )
    face_limit = min(cone_fraction * geo.outer_cone_dist, 10.0 * p.module)
    if p.face_width > face_limit:
        result.warn(
            "face_width",
            f"{p.face_width:.2f} mm exceeds the usual limit of "
            f"min({cone_label}, 10*m) = {face_limit:.2f} mm; the teeth get very "
            "small at the inner end",
        )

    # --- the spiral tooth trace --------------------------------------------
    _check_trace(geo, p, result)

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
            "for bevels",
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

    # --- backlash ----------------------------------------------------------
    #
    # Measured against the *outer* circular pitch, because that is where the
    # backlash is quoted: the tooth thickness it comes off is the one on the
    # outer back cone, and every inner section is a uniform scaling of it - so
    # the backlash a section actually carries tapers with k = A/Ao toward the
    # toe, the same way every other length on the tooth does.
    if p.backlash > MAX_BACKLASH_FRACTION * geo.circular_pitch:
        result.warn(
            "backlash",
            f"{p.backlash:.3f} mm is "
            f"{p.backlash / geo.circular_pitch:.1%} of the outer circular pitch "
            f"({geo.circular_pitch:.2f} mm); each member loses half of it off "
            "its tooth thickness",
        )

    # --- does a tooth space actually exist at the root? --------------------
    # Evaluate at the root circle, not the base circle. On a gear with many
    # virtual teeth the root sits *above* the base circle, where the space is
    # comfortably open even though extrapolating down to the base circle would
    # give a negative width.
    for member in (geo.pinion, geo.gear):
        from ..involute import inv, top_land  # keeps the module import-light

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


def _check_trace(geo, p: BevelSetParams, result: ValidationResult) -> None:
    """Whether the spiral tooth trace is one a cutter could actually sweep.

    Silent for a straight bevel gear, which has no trace at all.
    """
    trace = geo.trace
    if trace is None:
        return

    # --- does the arc even span the face? ----------------------------------
    #
    # A circle of radius r_c centred rho from the crown centre only has points
    # between crown radii |rho - r_c| and rho + r_c. Ask for a face outside that
    # band and there is no trace there to follow - the geometry would clamp to
    # the nearest end and quietly hand back a tooth that stops curving, which is
    # the kind of wrong that looks right in a preview. So this is an error.
    if not trace.reaches(geo.inner_cone_dist, geo.outer_cone_dist):
        lo = abs(trace.centre_distance - trace.cutter_radius)
        hi = trace.centre_distance + trace.cutter_radius
        result.error(
            "cutter_radius",
            f"a {trace.cutter_radius:.2f} mm cutter at {p.spiral_angle:g} deg "
            f"sweeps an arc spanning cone distances {lo:.2f} to {hi:.2f} mm, "
            f"which does not cover the face from {geo.inner_cone_dist:.2f} to "
            f"{geo.outer_cone_dist:.2f} mm",
        )
        return

    # --- how hard the spiral angle swings across the face -------------------
    psi_i = math.degrees(abs(trace.spiral_angle_at(geo.inner_cone_dist)))
    psi_o = math.degrees(abs(trace.spiral_angle_at(geo.outer_cone_dist)))
    if psi_o - psi_i > 20.0:
        result.warn(
            "cutter_radius",
            f"the spiral angle runs {psi_i:.1f} deg at the toe to {psi_o:.1f} "
            f"at the heel, a {psi_o - psi_i:.1f} deg swing; a cutter nearer "
            f"Am = {geo.mean_cone_dist:.2f} mm would flatten it",
        )
    if psi_o >= 90.0 - 1e-9:
        result.error(
            "cutter_radius",
            f"the trace stands at {psi_o:.1f} deg to the cone generator at the "
            "heel, which is tangent to the pitch circle - no tooth runs that way",
        )

    if abs(p.spiral_angle) >= HIGH_SPIRAL_ANGLE:
        result.warn(
            "spiral_angle",
            f"{abs(p.spiral_angle):g} deg is a hard spiral; the axial thrust it "
            "throws at the bearings grows with the tangent and 35 deg is the "
            "usual answer",
        )

    # --- how far the tooth travels round the gear --------------------------
    #
    # Reported rather than refused. A small pinion at a normal spiral angle
    # genuinely sweeps more than one angular pitch - the anchor 17-tooth pinion
    # crosses 38.790 degrees against a pitch of 21.176 - and that is what the
    # gear is, not a mistake. What it does cost is loft sections, so the number
    # is worth putting in front of someone before they wait for the build.
    for member in (geo.pinion, geo.gear):
        sweep = abs(
            phase_at_cone_distance(geo, member.name, geo.outer_cone_dist)
            - phase_at_cone_distance(geo, member.name, geo.inner_cone_dist)
        )
        pitches = sweep / member.angular_pitch
        if pitches > 2.0:
            result.warn(
                member.name,
                f"the tooth sweeps {math.degrees(sweep):.1f} deg over the face, "
                f"{pitches:.1f} angular pitches; the loft will need "
                f"{section_count(geo, member.name)} sections and the tooth wraps "
                "a long way round the blank",
            )
