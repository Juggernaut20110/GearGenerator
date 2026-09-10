"""Planar involute tooth profiles. Pure math - no COM, no GUI, no file I/O.

Millimetres and radians throughout, like everything upstream of `gears.sw`.

The frame
---------
One flat plane. The gear is centred on the origin and a tooth **SPACE** is
centred on angle 0, so the profile this module returns is the shape a cutter
removes, not the shape of a tooth. That choice is what lets the SOLIDWORKS layer
cut one space and circular-pattern it `z` times.

Both gear types share this module, and they mean different planes by it:

* a **spur** gear uses it directly - the plane is the transverse plane, and the
  profile it returns is the real tooth form
* a **bevel** gear uses it through Tredgold's approximation - the plane is the
  developed back cone, the profile is a "virtual" spur gear with `z / cos(delta)`
  teeth, and `gears.bevel.geometry.to_cone_3d` bends the result onto the cone

Nothing here knows which caller it has. Every function takes plain radii and
angles, so the only inputs that reach the flank shape are the base radius, the
root and tip radii, `psi0` and the angular half-pitch.

Internal gears
--------------
A ring gear's teeth point **inward**, so its tip is its smallest radius and its
root its largest - the reverse of everything above. The involute is the same
curve of the same base circle, and the one identity that makes it all reusable
is this:

    an internal gear's tooth SPACE has the shape of an external gear's TOOTH

Both narrow as the radius grows, following `psi0 - inv(alpha_r)`, where the
external gear's *space* widens instead. That sign is why `internal_flank_points`
is a second function rather than `flank_points` called cleverly: no substitution
of `psi0` or `half_pitch` turns one form into the other.

`tooth_space_loop` takes an `internal` flag and returns the same named segments
either way. The names describe the tooth's own features rather than radii - the
root is where the fillet sits, the cap is where the cut clears the blank - so
they stay true read from the material inward or outward.

Why `psi0` and `half_pitch` rather than module and tooth count
--------------------------------------------------------------
Because the angular constants are **scale-invariant**: `acos(k*r_b / k*r)` is
`acos(r_b / r)` for any k. A bevel section at any cone distance is a uniform
scaling of the outer one, so one `psi0` and one `half_pitch` serve every section
along the face width. Passing the two angles instead of the quantities they were
derived from is what makes that reuse free.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

Point2 = tuple[float, float]


@dataclass(frozen=True)
class StartOfInvolute:
    """Analytical start-of-involute result for a generated external root.

    ``trochoid_parameter`` is the rolling parameter used by the analytical
    rack envelope in this module.  ``involute_roll_parameter`` is the usual
    involute roll parameter ``tan(alpha_r)``.  Keeping both makes it explicit
    that the two curves were solved as a physical intersection rather than
    joined at a common sampled radius.
    """

    start_of_involute_r: float
    start_of_involute_angle: float
    involute_roll_parameter: float
    trochoid_parameter: float
    undercut: bool
    point: Point2

    @property
    def start_of_involute_d(self) -> float:
        """Start-of-involute diameter, ``d_Ff = 2*r_Ff``."""
        return 2.0 * self.start_of_involute_r


@dataclass(frozen=True)
class RackGeneratedRoot:
    """Analytical transverse envelope of a rounded straight rack corner.

    The curve is kept in analytical form.  ``phi_root`` and
    ``phi_flank_transition`` delimit the generated root from the nominal root
    circle to the rack flank/tip tangency.  The latter is only the standard
    no-undercut transition; undercut gears use :func:`solve_start_of_involute`
    to find the physical intersection with the nominal involute.
    """

    reference_r: float
    r_base: float
    r_root: float
    psi0: float
    half_pitch: float
    cutter_tip_depth: float
    rack_root_radius: float
    alpha: float
    pitch_space_angle: float
    phi_root: float
    phi_flank_transition: float
    v_centre: float
    u_centre: float

    @property
    def generated_root_r(self) -> float:
        """Generated root-circle radius (ISO ``d_fE/2`` in this model)."""
        return self.reference_r - self.cutter_tip_depth

    @property
    def undercut(self) -> bool:
        return undercut_condition(self)

    @property
    def base_space_angle(self) -> float:
        return self.half_pitch - self.psi0

    def _raw_envelope(self, phi: float) -> Point2:
        c, s = math.cos(phi), math.sin(phi)
        a = self.reference_r + self.v_centre
        b = self.u_centre - self.reference_r * phi
        cx = c * a - s * b
        cy = s * a + c * b

        # Derivative of Rot(phi) (a, u_centre - R*phi).
        dcx = -s * a - c * b + s * self.reference_r
        dcy = c * a - s * b - c * self.reference_r
        speed = math.hypot(dcx, dcy)
        if speed <= 1e-14:
            raise ValueError("rack root envelope has a stationary tool corner")

        # The active side is the minus side of this inverted cutter corner.
        return (
            cx + self.rack_root_radius * dcy / speed,
            cy - self.rack_root_radius * dcx / speed,
        )

    def point(self, phi: float) -> Point2:
        """Evaluate the generated root at analytical rolling parameter ``phi``."""
        x, y = self._raw_envelope(phi)
        c, s = math.cos(self.pitch_space_angle), math.sin(self.pitch_space_angle)
        return x * c - y * s, x * s + y * c

    def polar(self, phi: float) -> tuple[float, float]:
        """Return ``(radius, polar angle)`` for the analytical trochoid point."""
        point = self.point(phi)
        return math.hypot(*point), math.atan2(point[1], point[0])

    def nominal_involute_point(self, roll: float) -> Point2:
        """Evaluate the positive external nominal involute at roll ``tan(alpha)``."""
        if roll < -1e-14:
            raise ValueError("involute roll parameter must be non-negative")
        roll = max(0.0, roll)
        radius = self.r_base * math.sqrt(1.0 + roll * roll)
        angle = self.base_space_angle + roll - math.atan(roll)
        return polar(radius, angle)

    def nominal_involute_at_radius(self, radius: float) -> tuple[Point2, float]:
        """Return the nominal involute point and roll parameter at ``radius``."""
        if radius < self.r_base - 1e-12:
            raise ValueError("nominal involute is undefined below the base circle")
        roll = math.sqrt(max(0.0, (radius / self.r_base) ** 2 - 1.0))
        return self.nominal_involute_point(roll), roll

    def sample_to(self, phi_end: float, n: int) -> list[Point2]:
        """Sample the analytical root from the nominal root circle to ``phi_end``."""
        if n < 2:
            raise ValueError("at least two root samples are required")
        points = [
            self.point(
                self.phi_root
                + (phi_end - self.phi_root) * i / (n - 1)
            )
            for i in range(n)
        ]
        # The first point is exactly on the generated root circle.  Removing
        # floating-point drift here does not alter the analytical curve.
        points[0] = polar(
            self.generated_root_r,
            self.pitch_space_angle + self.phi_root,
        )
        return points

# Default sampling density along one involute flank.
FLANK_POINTS = 40

# Smallest top land the tip is allowed to keep, as a multiple of the module.
#
# Past a point the two flanks of a tooth cross: the top land goes negative and
# neighbouring tooth spaces overlap. Callers clamp the tip radius so that never
# happens.
#
# It is not a hypothetical. A Gleason long-addendum bevel pinion with few teeth
# is already close to pointed at its nominal tip - 17 teeth at 2.53:1 leaves only
# 0.36 mm of top land - and pushing the loft section outward along the cone grows
# the tip radius further. The extension region lies outside the blank anyway, so
# clamping it costs nothing.
MIN_TOP_LAND_FACTOR = 0.05

# How far past the tip the tooth-space profile is carried, so that a cut fully
# clears the blank radially. A multiple of the module.
CUT_OVERSHOOT_FACTOR = 0.5


def inv(angle: float) -> float:
    """Involute function: inv(a) = tan(a) - a."""
    return math.tan(angle) - angle


def polar(r: float, phi: float) -> Point2:
    return r * math.cos(phi), r * math.sin(phi)


def top_land(r: float, r_base: float, psi0: float) -> float:
    """Tooth thickness at radius r, along the arc. Negative means pointed."""
    alpha_r = math.acos(min(1.0, r_base / r))
    return 2.0 * (psi0 - inv(alpha_r)) * r


def max_tip_radius(r_base: float, psi0: float, min_land: float) -> float:
    """Largest tip radius still leaving `min_land` of top land.

    `top_land` decreases monotonically once past the base circle, so a plain
    bisection is safe.
    """
    if top_land(r_base, r_base, psi0) <= min_land:
        return r_base

    lo = hi = r_base
    for _ in range(200):
        hi *= 1.05
        if top_land(hi, r_base, psi0) <= min_land:
            break
    for _ in range(80):
        mid = 0.5 * (lo + hi)
        if top_land(mid, r_base, psi0) > min_land:
            lo = mid
        else:
            hi = mid
    return lo


def flank_points(
    r_base: float,
    r_root: float,
    r_tip: float,
    psi0: float,
    half_pitch: float,
    n: int,
) -> list[Point2]:
    """One side of an external gear's tooth space, root to tip, at positive angle.

    The tooth centred on angle 0 has angular half-thickness
        theta_t(r) = psi0 - inv(acos(r_base / r))
    so the space centred on angle 0 has half-width half_pitch - theta_t(r).

    Below the base circle the involute is undefined; a radial line is used
    instead, which is the standard simplification (the true form there is a
    trochoid that depends on the cutter).
    """

    def space_angle(r: float) -> float:
        alpha_r = math.acos(min(1.0, r_base / r))
        return half_pitch - psi0 + inv(alpha_r)

    pts: list[Point2] = []

    # Sample uniformly in the involute roll parameter rather than in radius:
    # it distributes points evenly along the curve instead of bunching them at
    # the tip.
    r_lo = max(r_root, r_base)
    if r_root < r_base:
        pts.append(polar(r_root, space_angle(r_base)))

    t_lo = math.sqrt(max(0.0, (r_lo / r_base) ** 2 - 1.0))
    t_hi = math.sqrt(max(0.0, (r_tip / r_base) ** 2 - 1.0))
    for i in range(n):
        t = t_lo + (t_hi - t_lo) * i / (n - 1)
        r = r_base * math.sqrt(1.0 + t * t)
        pts.append(polar(r, space_angle(r)))

    return pts


def rack_generated_root(
    reference_r: float,
    r_base: float,
    r_root: float,
    psi0: float,
    half_pitch: float,
    cutter_tip_depth: float,
    rack_root_radius: float,
) -> RackGeneratedRoot | None:
    """Return an analytical external straight-rack root representation.

    This is the rolling envelope of the rounded tip of an inverted basic-rack
    tooth.  It deliberately does not decide where the nominal involute starts;
    :func:`solve_start_of_involute` performs that separate ISO 21771-1 step.

    ``cutter_tip_depth`` is measured from the gear reference circle towards
    its centre.  For an external gear generated from the ISO 53 basic rack it
    is the member dedendum, ``m_n (h_fP* - x)``.  The straight tip land then
    envelopes the nominal root circle at ``reference_r - cutter_tip_depth``.
    ``rack_root_radius`` is ``rho_fP`` in the same transverse section.

    The rolling equations are written in a tangent rack frame.  If ``u`` is
    tangent to the gear and ``v`` is outward from its centre, a fixed rack
    point transforms to the gear frame as

        (x, y) = Rot(phi) (reference_r + v, u - reference_r*phi).

    A circle centre follows this motion.  The envelope point is the centre
    offset by the radius along the normal to the centre trajectory.  This is
    a direct envelope construction; no polynomial or spline is used as the
    source geometry.  CAD code samples the returned representation only after
    the start-of-involute solution has been obtained.

    This function deliberately covers the external straight-gear case only.
    ISO 53 defines the rack in the normal section; a helical cutter's
    transverse tip rounding is not the same circle after projection.  The
    spur caller therefore uses this function only for ``beta == 0`` and
    explicitly falls back to its legacy root approximation otherwise.

    ``None`` means that the selected rack geometry cannot form a usable
    analytical representation.
    """
    if (
        reference_r <= 0.0
        or r_base <= 0.0
        or r_root <= 0.0
        or cutter_tip_depth <= 0.0
        or rack_root_radius <= 0.0
    ):
        return None

    # The straight rack flank is u = tan(alpha) * v.  alpha is obtained from
    # the gear's reference and base circles, so the construction uses the
    # same transverse involute that the flank generator uses.
    ratio = r_base / reference_r
    if ratio <= 0.0 or ratio > 1.0 + 1e-12:
        return None
    alpha = math.acos(min(1.0, ratio))
    if alpha <= 1e-12 or alpha >= math.pi / 2.0:
        return None

    cos_alpha = math.cos(alpha)
    tan_alpha = math.tan(alpha)
    sec_alpha = 1.0 / cos_alpha
    rho = rack_root_radius
    depth = cutter_tip_depth

    # The relevant corner is the one whose inverted cutter tooth extends to
    # negative u from this flank.  The centre is rho above the cutter tip
    # line and rho from the straight flank.
    v_centre = -depth + rho
    u_centre = tan_alpha * v_centre - rho * sec_alpha

    # Tangency point of the rack rounding with its straight flank.  In the
    # rack frame the unit normal to u - tan(alpha)v = 0 is
    # (cos(alpha), -sin(alpha)).
    v_touch = v_centre - rho * math.sin(alpha)

    # The line-envelope condition is v = tan(alpha) R phi / (1+tan^2(alpha)).
    phi_transition = v_touch * (1.0 + tan_alpha * tan_alpha) / (
        tan_alpha * reference_r
    )
    # The tip-line envelope touches the rounded corner's bottom point when
    # the rack tangent coordinate is R*phi.
    phi_root = u_centre / reference_r
    if phi_transition >= phi_root:
        return None

    generated_root_r = reference_r - depth
    if abs(generated_root_r - r_root) > 1e-8 * max(1.0, reference_r):
        # The member dimensions and the selected rack would describe two
        # different root circles.  Do not silently produce a discontinuity.
        return None

    # The pitch-point flank is generated at angle zero in this rolling frame.
    # Rotate the complete generated boundary to the requested tooth-space
    # angle; this carries the profile-shifted tooth thickness into the root
    # without inventing a second angular convention.
    pitch_space_angle = (
        half_pitch - psi0 + inv(alpha)
    )
    representation = RackGeneratedRoot(
        reference_r=reference_r,
        r_base=r_base,
        r_root=r_root,
        psi0=psi0,
        half_pitch=half_pitch,
        cutter_tip_depth=cutter_tip_depth,
        rack_root_radius=rack_root_radius,
        alpha=alpha,
        pitch_space_angle=pitch_space_angle,
        phi_root=phi_root,
        phi_flank_transition=phi_transition,
        v_centre=v_centre,
        u_centre=u_centre,
    )
    try:
        root_point = representation.point(phi_root)
        transition_point = representation.point(phi_transition)
    except ValueError:
        return None
    if (
        not math.isfinite(math.hypot(*root_point))
        or math.hypot(*transition_point) <= r_root
        or not math.isfinite(math.hypot(*transition_point))
    ):
        return None
    return representation


def undercut_condition(root: RackGeneratedRoot) -> bool:
    """Return the Clause 10.2 rack undercut condition for ``root``.

    For a straight rack, the transverse and normal pressure angles coincide.
    With ``B = h_fP - x*m_n - rho_fP*(1 - sin(alpha_n))`` represented by
    ``cutter_tip_depth - rack_root_radius*(1 - sin(alpha))``, ISO 21771-1
    Clause 10.2 has undercut when ``(d/2)*sin(alpha)^2 - B < 0``.
    """
    corner_depth = root.cutter_tip_depth - root.rack_root_radius * (
        1.0 - math.sin(root.alpha)
    )
    scale = max(1.0, abs(root.reference_r), abs(corner_depth))
    return (
        root.reference_r * math.sin(root.alpha) ** 2 - corner_depth
        < -1e-14 * scale
    )


def _angular_intersection_residual(
    root: RackGeneratedRoot, phi: float
) -> float | None:
    """Analytical angular residual after eliminating radius with Eq. (275)."""
    radius, angle = root.polar(phi)
    if radius < root.r_base - 1e-11 * max(1.0, root.r_base):
        return None
    _, roll = root.nominal_involute_at_radius(radius)
    involute_angle = root.base_space_angle + roll - math.atan(roll)
    return angle - involute_angle


def _base_circle_boundary(root: RackGeneratedRoot, valid_phi: float, invalid_phi: float) -> float:
    """Bound a local crossing of the trochoid with the nominal base circle."""
    valid_radius = root.polar(valid_phi)[0]
    invalid_radius = root.polar(invalid_phi)[0]
    if valid_radius < root.r_base or invalid_radius >= root.r_base:
        raise ValueError("invalid base-circle boundary bracket")
    lo, hi = valid_phi, invalid_phi
    for _ in range(100):
        mid = 0.5 * (lo + hi)
        if root.polar(mid)[0] >= root.r_base:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


def _base_boundary_residual(root: RackGeneratedRoot, phi: float) -> float:
    """Angular residual at the base-circle limit, where involute roll is zero."""
    return root.polar(phi)[1] - root.base_space_angle


def _bisect_intersection(
    root: RackGeneratedRoot, lo: float, hi: float, flo: float, fhi: float
) -> float:
    """Bounded bisection of the analytical angular intersection residual."""
    if abs(flo) <= 1e-14:
        return lo
    if abs(fhi) <= 1e-14:
        return hi
    for _ in range(120):
        mid = 0.5 * (lo + hi)
        fmid = _angular_intersection_residual(root, mid)
        if fmid is None:
            # This only occurs at the base-circle boundary.  Move toward the
            # valid half of the current bounded interval.
            lo = mid
            flo = fmid if fmid is not None else flo
            continue
        if abs(fmid) <= 1e-14:
            return mid
        if flo * fmid <= 0.0:
            hi, fhi = mid, fmid
        else:
            lo, flo = mid, fmid
    return 0.5 * (lo + hi)


def solve_start_of_involute(
    root: RackGeneratedRoot,
) -> StartOfInvolute | None:
    """Solve the ISO 21771-1 start of involute for an analytical root.

    The radial equality in Clause 10.3 Eq. (275) is eliminated analytically by
    evaluating the involute roll parameter from the trochoid radius.  A bounded
    sign-bracketed solve then enforces Eq. (276), equality of polar angle.  No
    sampled CAD point is used as the solution.
    """
    undercut = undercut_condition(root)
    if not undercut:
        phi = root.phi_flank_transition
        radius, angle = root.polar(phi)
        if radius < root.r_base:
            return None
        _, roll = root.nominal_involute_at_radius(radius)
        return StartOfInvolute(
            start_of_involute_r=radius,
            start_of_involute_angle=angle,
            involute_roll_parameter=roll,
            trochoid_parameter=phi,
            undercut=False,
            point=root.point(phi),
        )

    lo = min(root.phi_root, root.phi_flank_transition)
    hi = max(root.phi_root, root.phi_flank_transition)
    # The values are analytical evaluations; this bounded subdivision only
    # finds a sign bracket and never substitutes for the root solve.
    brackets: list[tuple[float, float, float, float]] = []
    previous: tuple[float, float] | None = None
    for i in range(2049):
        phi = lo + (hi - lo) * i / 2048.0
        value = _angular_intersection_residual(root, phi)
        if value is None:
            if previous is not None:
                boundary = _base_circle_boundary(root, previous[0], phi)
                boundary_value = _base_boundary_residual(root, boundary)
                if previous[1] * boundary_value < 0.0:
                    brackets.append(
                        (previous[0], boundary, previous[1], boundary_value)
                    )
                previous = None
            continue
        if abs(value) <= 1e-13:
            brackets.append((phi, phi, value, value))
        elif previous is not None and previous[1] * value < 0.0:
            brackets.append((previous[0], phi, previous[1], value))
        previous = (phi, value)

    if not brackets:
        return None

    roots = [
        lo if lo == hi else _bisect_intersection(root, lo, hi, flo, fhi)
        for lo, hi, flo, fhi in brackets
    ]
    # If two intersections exist, the root-form transition is the physical
    # one with the smallest radius on the root side of the nominal involute.
    phi = min(roots, key=lambda candidate: root.polar(candidate)[0])
    radius, angle = root.polar(phi)
    _, roll = root.nominal_involute_at_radius(radius)
    return StartOfInvolute(
        start_of_involute_r=radius,
        start_of_involute_angle=angle,
        involute_roll_parameter=roll,
        trochoid_parameter=phi,
        undercut=True,
        point=root.point(phi),
    )


def rack_root_envelope(
    reference_r: float,
    r_base: float,
    r_root: float,
    psi0: float,
    half_pitch: float,
    cutter_tip_depth: float,
    rack_root_radius: float,
    n: int = 24,
) -> tuple[list[Point2], float] | None:
    """Compatibility wrapper returning sampled root points and SOI radius.

    New code should use :func:`rack_generated_root` followed by
    :func:`solve_start_of_involute` so the full analytical result remains
    available.  The wrapper keeps the old tuple-shaped helper usable by
    downstream callers during migration.
    """
    root = rack_generated_root(
        reference_r,
        r_base,
        r_root,
        psi0,
        half_pitch,
        cutter_tip_depth,
        rack_root_radius,
    )
    if root is None:
        return None
    soi = solve_start_of_involute(root)
    if soi is None:
        return None
    return root.sample_to(soi.trochoid_parameter, n), soi.start_of_involute_r


def internal_flank_points(
    r_base: float,
    r_root: float,
    r_tip: float,
    psi0: float,
    n: int,
) -> list[Point2]:
    """One side of an *internal* gear's tooth space, tip to root, positive angle.

    An internal gear's tooth space has the shape of an external gear's **tooth**.
    That is the whole of the difference, and it is worth stating as an identity
    rather than a rule of thumb, because it is what lets one involute serve
    both. Compare the two half-widths:

        external tooth   theta_t(r) = psi0 - inv(alpha_r)      narrows outward
        external space   theta_s(r) = half_pitch - theta_t(r)  widens outward
        internal space   theta_s(r) = psi0 - inv(alpha_r)      narrows outward

    So the sign on `inv` flips, and that is why this cannot be `flank_points`
    called with its arguments rearranged - no substitution of psi0 or half_pitch
    turns one into the other. `psi0` here is the **space** half-width
    extrapolated to the base circle, `e / (2 * r_p) + inv(alpha_t)`, where the
    external gear's psi0 is built from the tooth thickness instead.

    The radial run also reverses. An internal gear's tip is its *innermost*
    radius - the teeth point inward, toward the axis - and its root is furthest
    out, so the space runs from `r_tip` up to `r_root` with r_tip < r_root.

    There is no below-the-base-circle case to handle. The internal root is the
    outermost radius of the whole tooth, so if the tip clears the base circle
    then so does everything above it; a tip that does *not* is a gear with no
    involute flank at all, and `validate` refuses it rather than drawing a
    radial line and pretending.
    """

    def space_angle(r: float) -> float:
        alpha_r = math.acos(min(1.0, r_base / r))
        return psi0 - inv(alpha_r)

    # Sampled uniformly in the roll parameter, same as the external case, and
    # ordered inner to outer so the caller gets tip-to-root - the direction that
    # matches "root to tip" once you remember which way an internal tooth points.
    t_lo = math.sqrt(max(0.0, (max(r_tip, r_base) / r_base) ** 2 - 1.0))
    t_hi = math.sqrt(max(0.0, (r_root / r_base) ** 2 - 1.0))

    pts: list[Point2] = []
    for i in range(n):
        t = t_lo + (t_hi - t_lo) * i / (n - 1)
        r = r_base * math.sqrt(1.0 + t * t)
        pts.append(polar(r, space_angle(r)))
    return pts


def internal_space_width(r: float, r_base: float, psi0: float) -> float:
    """Width of an internal gear's tooth space at radius r, along the arc.

    The mirror of `top_land`: that measures an external gear's tooth, this
    measures an internal gear's space, and they are the same formula because
    they are the same shape. Negative means the space has closed up.
    """
    return top_land(r, r_base, psi0)


def internal_tooth_width(
    r: float, r_base: float, psi0: float, half_pitch: float
) -> float:
    """Width of an internal gear's *tooth* at radius r, along the arc.

    What is left of the pitch after the space is taken out. An internal tooth
    is narrowest at its **tip**, which is its innermost radius - the opposite
    end from an external tooth - so this is what decides whether a ring gear's
    teeth come to a point, and it is checked at the tip radius.
    """
    return 2.0 * (half_pitch - (psi0 - inv(math.acos(min(1.0, r_base / r))))) * r


def min_internal_tip_radius(
    r_base: float, psi0: float, half_pitch: float, min_land: float
) -> float:
    """Smallest tip radius an internal gear can have and still keep a top land.

    The counterpart of `max_tip_radius`, and it bisects the other way round:
    an internal tooth widens as the radius grows, so the constraint is a floor
    rather than a ceiling.
    """
    if internal_tooth_width(r_base, r_base, psi0, half_pitch) > min_land:
        return r_base

    lo, hi = r_base, r_base
    for _ in range(200):
        hi *= 1.05
        if internal_tooth_width(hi, r_base, psi0, half_pitch) > min_land:
            break
    for _ in range(80):
        mid = 0.5 * (lo + hi)
        if internal_tooth_width(mid, r_base, psi0, half_pitch) > min_land:
            hi = mid
        else:
            lo = mid
    return hi


def point_segment_distance(p: Point2, a: Point2, b: Point2) -> tuple[float, Point2]:
    ax, ay = a
    bx, by = b
    px, py = p
    dx, dy = bx - ax, by - ay
    denom = dx * dx + dy * dy
    if denom < 1e-18:
        return math.hypot(px - ax, py - ay), a
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / denom))
    q = (ax + t * dx, ay + t * dy)
    return math.hypot(px - q[0], py - q[1]), q


def distance_to_polyline(p: Point2, poly: list[Point2]) -> tuple[float, Point2, int]:
    best = (float("inf"), poly[0], 0)
    for i in range(len(poly) - 1):
        d, q = point_segment_distance(p, poly[i], poly[i + 1])
        if d < best[0]:
            best = (d, q, i)
    return best


def root_fillet(
    flank: list[Point2],
    r_root: float,
    rho: float,
    arc_points: int = 9,
    internal: bool = False,
) -> tuple[list[Point2], list[Point2]] | None:
    """Fit a circular fillet of radius `rho` tangent to the flank and the root.

    Returns (trimmed_flank, arc_points_root_to_flank), or None if no fillet of
    that size fits inside the space - in which case the caller keeps a sharp
    corner.

    The fillet centre must sit one fillet radius **inside the material** from
    the root circle, so only its angle is unknown. For an external gear the root
    is the innermost radius and the centre goes at `r_root + rho`; for an
    internal gear the root is the outermost and it goes at `r_root - rho`. That
    one sign is the whole difference - distance-to-flank still decreases
    monotonically as the angle sweeps from the space centreline toward the
    flank, so the same bisection serves both.

    `flank` must start at the root end either way. The internal flank generator
    returns tip-to-root, so its caller reverses it before handing it over.
    """
    if rho <= 0.0:
        return None

    phi_flank = math.atan2(flank[0][1], flank[0][0])
    if phi_flank <= 0.0:
        return None

    r_centre = r_root - rho if internal else r_root + rho
    if r_centre <= 0.0:
        return None

    def gap(phi: float) -> float:
        c = polar(r_centre, phi)
        return distance_to_polyline(c, flank)[0] - rho

    lo, hi = 0.0, phi_flank
    if gap(lo) <= 0.0:
        return None  # space too narrow for this fillet radius

    for _ in range(60):
        mid = 0.5 * (lo + hi)
        if gap(mid) > 0.0:
            lo = mid
        else:
            hi = mid
    phi_c = 0.5 * (lo + hi)

    centre = polar(r_centre, phi_c)
    _, touch, seg = distance_to_polyline(centre, flank)

    # Tangent point on the root circle is radially outward from the centre for
    # an internal gear and inward for an external one - which is just to say it
    # is on the root circle at the centre's own angle, either way.
    root_touch = polar(r_root, phi_c)

    a0 = math.atan2(root_touch[1] - centre[1], root_touch[0] - centre[0])
    a1 = math.atan2(touch[1] - centre[1], touch[0] - centre[0])
    # Keep the short way round.
    while a1 - a0 > math.pi:
        a1 -= 2.0 * math.pi
    while a0 - a1 > math.pi:
        a1 += 2.0 * math.pi

    arc = [
        (
            centre[0] + rho * math.cos(a0 + (a1 - a0) * i / (arc_points - 1)),
            centre[1] + rho * math.sin(a0 + (a1 - a0) * i / (arc_points - 1)),
        )
        for i in range(arc_points)
    ]
    return [touch] + flank[seg + 1:], arc


def tooth_space_loop(
    r_base: float,
    r_root: float,
    r_tip: float,
    r_cap: float,
    psi0: float,
    half_pitch: float,
    fillet_rho: float,
    n_flank: int = FLANK_POINTS,
    split_cap: bool = False,
    internal: bool = False,
    rack_root: tuple[list[Point2], float] | None = None,
) -> tuple[dict[str, list[Point2]], list[Point2], bool]:
    """One closed tooth-space boundary: named segments, flat loop, fillet flag.

    The loop runs counter-clockwise: up the negative flank, out past the tip,
    across the cap, back down the positive flank, then round the root. It is
    returned both as named pieces and as one flattened polyline, because the two
    consumers want different things - the SOLIDWORKS layer builds splines and
    arcs separately rather than forcing one spline through a sharp corner, while
    the preview and DXF export want a single closed polyline.

    `split_cap` breaks the cap into `cap_neg` + `cap_pos` meeting at a real
    vertex on the space centreline. A guided loft needs its guide curve to touch
    a point that exists in every profile, and a point that merely lies *near* a
    spline is the classic way such a loft fails - so the helical spur builder
    asks for the vertex to be there by construction.

    `internal` builds a ring gear's space instead. **Every segment keeps its
    name and its job**, and the caller sees the same dictionary; what changes is
    which way "out past the tip" points. An internal gear's teeth point inward,
    so its tip is its smallest radius and its root its largest, and the caller
    passes `r_tip < r_root` with `r_cap` smaller still. The names stay honest
    because they name the tooth's own features, not radii: the root is where the
    fillet is, the cap is where the cut clears the blank, and both are true of
    a ring gear read from the material outward.

    If ``rack_root`` is supplied for an external member, it is the output of
    :func:`rack_root_envelope`: the ``fillet_*`` segments become compatibility
    aliases for the named ``generated_root_*`` segments, while ``root`` stays
    the circular tip-land envelope. Internal callers do not consume this
    argument, which prevents the external rack construction from being
    applied to a ring by accident.
    """
    generated_root: list[Point2] = []
    if internal:
        # Tip to root, so reverse it: `root_fillet` and the loop assembly below
        # both want the flank to start at the root end.
        flank = internal_flank_points(r_base, r_root, r_tip, psi0, n_flank)[::-1]
        fillet = root_fillet(flank, r_root, fillet_rho, internal=True)
    elif rack_root is not None:
        # The rack envelope already contains the root transition.  Generate
        # only the true involute from that transition to the tip and retain
        # the old segment names as compatibility aliases for CAD consumers.
        generated_root, transition_r = rack_root
        if r_tip <= transition_r + 1e-10:
            # The requested tip does not leave room for the selected rack
            # corner.  The caller may still build a useful section with the
            # documented legacy approximation.
            generated_root = []
            flank = flank_points(
                r_base, r_root, r_tip, psi0, half_pitch, n_flank
            )
            fillet = root_fillet(flank, r_root, fillet_rho, internal=False)
        else:
            flank = flank_points(
                r_base, transition_r, r_tip, psi0, half_pitch, n_flank
            )
            if not flank:
                generated_root = []
                fillet = root_fillet(
                    flank, r_root, fillet_rho, internal=False
                )
            else:
                # The analytical solver established this common physical
                # point.  Unify only the two numerically sampled copies; do
                # not move either curve to manufacture continuity.
                if math.dist(generated_root[-1], flank[0]) > 1e-8:
                    generated_root = []
                    fillet = root_fillet(
                        flank, r_root, fillet_rho, internal=False
                    )
                else:
                    generated_root[-1] = flank[0]
                    fillet = None
    else:
        flank = flank_points(r_base, r_root, r_tip, psi0, half_pitch, n_flank)
        fillet = root_fillet(flank, r_root, fillet_rho, internal=False)

    if fillet is not None:
        flank, arc = fillet
    else:
        arc = generated_root

    # Mirror across the x axis for the other side of the space.
    def mirror(pts: list[Point2]) -> list[Point2]:
        return [(x, -y) for x, y in pts]

    phi_root = math.atan2(arc[0][1], arc[0][0]) if arc else math.atan2(
        flank[0][1], flank[0][0]
    )
    phi_tip = math.atan2(flank[-1][1], flank[-1][0])

    root_arc = [polar(r_root, -phi_root + 2.0 * phi_root * i / 8.0) for i in range(9)]
    cap = [polar(r_cap, -phi_tip + 2.0 * phi_tip * i / 4.0) for i in range(5)]

    segments = {
        # Root to flank, matching the direction of travel round the loop; the
        # positive side runs the other way and so is reversed instead.
        "fillet_neg": mirror(arc) if arc else [],
        "generated_root_neg": mirror(generated_root) if generated_root else [],
        "flank_neg": mirror(flank),
        "riser_neg": [mirror(flank)[-1], cap[0]],
        "cap": cap,
        "riser_pos": [cap[-1], flank[-1]],
        "flank_pos": flank[::-1],
        "fillet_pos": arc[::-1] if arc else [],
        "generated_root_pos": generated_root[::-1] if generated_root else [],
        "root": root_arc[::-1],
    }
    order = [
        "fillet_neg", "flank_neg", "riser_neg", "cap",
        "riser_pos", "flank_pos", "fillet_pos", "root",
    ]

    if split_cap:
        # cap has 5 points, so index 2 is exactly on the centreline.
        mid = len(cap) // 2
        segments["cap_neg"] = cap[: mid + 1]
        segments["cap_pos"] = cap[mid:]
        del segments["cap"]
        order[order.index("cap")] = "cap_neg"
        order.insert(order.index("cap_neg") + 1, "cap_pos")

    loop: list[Point2] = []
    for name in order:
        for pt in segments[name]:
            if not loop or math.dist(loop[-1], pt) > 1e-9:
                loop.append(pt)
    if loop and math.dist(loop[0], loop[-1]) < 1e-9:
        loop.pop()

    return segments, loop, bool(arc)
