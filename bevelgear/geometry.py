"""Straight bevel gear geometry. Pure math - no COM, no GUI, no file I/O.

Units are millimetres and radians throughout. The SOLIDWORKS layer converts to
metres at its own boundary; nothing here knows about that.

3D coordinate system
--------------------
The gear axis is +Z and the **pitch apex sits at the origin**. The toothed body
extends toward +Z. Putting the apex at the origin is what makes the inner
section a plain uniform scaling of the outer one (see `SECTION_SCALE` below).

Tooth form
----------
Tredgold's approximation: develop the back cone into a flat "virtual" spur gear
with z / cos(delta) teeth, generate a true involute there, then map that planar
profile back onto the cone. Tooth proportions follow the Gleason
long-and-short-addendum system, which is what GearTrax uses.

A consequence of the Gleason system worth knowing, because it drives the code:
the dedendum angle is atan(dedendum / Ao), which puts the **root cone apex
exactly at the pitch apex**, so root radii scale uniformly. The addendum angle
is instead borrowed from the mate (theta_a1 = theta_f2, giving constant
clearance along the face), which puts the **face cone apex off the origin**, so
tip radii do *not* scale uniformly and must be solved for. That asymmetry is the
single subtlest thing in this module.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from .params import BevelSetParams

# Gleason straight-bevel proportions, as multiples of the outer module.
WORKING_DEPTH_FACTOR = 2.000
WHOLE_DEPTH_FACTOR = 2.188
CLEARANCE_FACTOR = 0.188

# How far past the tip the tooth-space profile is carried, so that a loft cut
# fully clears the blank radially. A multiple of the module.
CUT_OVERSHOOT_FACTOR = 0.5

# How far past each end of the face width the loft sections are pushed.
#
# The outer section would otherwise sit exactly on the back cone - which is a
# real face of the blank - so the cut would end tangent to that face and
# SOLIDWORKS rejects it with "would result in zero-thickness geometry". The
# inner section touches the front face at a single point for the same reason.
# Extending both ends past the material costs nothing: every flank is a conical
# surface through the pitch apex, so the extension is the same surface.
END_OVERSHOOT_FRACTION = 0.05
END_OVERSHOOT_MIN_MM = 0.5


def beyond_back_cone(geo: "SetGeometry", m: "MemberGeometry", pt: "Point3") -> float:
    """How far a point sits behind the back cone, along the pitch cone. mm.

    The back cone stands perpendicular to the pitch cone at Ao, so the signed
    distance from it is just the component of (point - outer pitch point) along
    the pitch cone generator. Positive means outside the blank.
    """
    x, y, z = pt
    sin_d, cos_d = math.sin(m.pitch_angle), math.cos(m.pitch_angle)
    return (math.hypot(x, y) - geo.outer_cone_dist * sin_d) * sin_d + (
        z - geo.outer_cone_dist * cos_d
    ) * cos_d


def end_overshoot(geo: "SetGeometry", member: str, margin: float | None = None) -> float:
    """Smallest overshoot that puts both loft sections clear of the blank, mm.

    A fixed fraction of the face width is not enough on its own: whether the
    inner section clears the flat front face depends on the cone angle and the
    tooth depth. So start from the fraction and grow until the property actually
    holds - measured on the real section points rather than assumed.

    The outer end is the easy one now that the blank ends on the back cone: the
    section and the back cone are the same family of cones, so *every* point of
    the section clears by exactly the overshoot. It is still measured rather
    than asserted, because that is what keeps the two ends honest with each
    other.
    """
    p = geo.params
    m = geo.member(member)
    if margin is None:
        margin = max(0.2, 0.1 * p.module)

    z_front = front_face_z(geo, m)     # the flat front face

    over = max(END_OVERSHOOT_MIN_MM, END_OVERSHOOT_FRACTION * p.face_width)
    for _ in range(40):
        outer = tooth_space_section(geo, member, "outer", overshoot=over)
        inner = tooth_space_section(geo, member, "inner", overshoot=over)
        clear_back = min(beyond_back_cone(geo, m, pt) for pt in outer.loop_3d()) > margin
        clear_front = max(z for _, _, z in inner.loop_3d()) < z_front - margin
        if clear_back and clear_front:
            return over
        over *= 1.25
    return over


def front_face_z(geo: "SetGeometry", m: "MemberGeometry") -> float:
    """Axial position of the blank's flat front face: the inner tip point."""
    return (
        geo.inner_cone_dist / math.cos(m.pitch_angle)
        - m.virtual_tip_r_inner * math.sin(m.pitch_angle)
    )

# Default sampling density along one involute flank.
FLANK_POINTS = 40

# Smallest top land the tip is allowed to keep, as a multiple of the module.
#
# A Gleason long-addendum pinion with few teeth is already close to pointed at
# its nominal tip - 17 teeth at 2.53:1 leaves only 0.36 mm of top land. Pushing
# the loft section outward along the cone grows the tip radius further, and past
# a point the two flanks of a tooth cross: the top land goes negative and
# neighbouring tooth spaces overlap. The tip radius is clamped so that never
# happens. The extension region lies outside the blank anyway, so clamping it
# costs nothing.
MIN_TOP_LAND_FACTOR = 0.05


def inv(angle: float) -> float:
    """Involute function: inv(a) = tan(a) - a."""
    return math.tan(angle) - angle


# ---------------------------------------------------------------------------
# Derived geometry
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class MemberGeometry:
    """Derived geometry for one member (pinion or gear) of the set."""

    name: str
    z: int
    pitch_angle: float          # delta
    pitch_dia: float            # d, at the outer end
    addendum: float             # a
    dedendum: float             # b_f
    addendum_angle: float       # theta_a
    dedendum_angle: float       # theta_f
    face_angle: float           # delta_a = delta + theta_a
    root_angle: float           # delta_f = delta - theta_f

    # Virtual (back cone) spur gear, in the developed plane.
    virtual_teeth: float        # z_v = z / cos(delta)
    virtual_pitch_r: float      # r_p = (d/2) / cos(delta)
    virtual_base_r: float       # r_b = r_p * cos(alpha)
    virtual_tip_r: float        # r_a = r_p + a          (outer end)
    virtual_root_r: float       # r_f = r_p - b_f        (outer end)
    virtual_tip_r_inner: float  # tip radius at the inner end (solved, not scaled)

    outside_dia: float          # d_a = d + 2*a*cos(delta)
    crown_to_apex: float        # axial distance, pitch apex to crown
    mounting_distance: float    # crown to the mate's axis

    # Outer root point: where the back cone stops and the flat back begins.
    # Same back-cone generator as the crown, stepped inward by the dedendum.
    outer_root_radius: float    # R of that point
    root_to_apex: float         # axial distance, pitch apex to that point

    @property
    def pitch_angle_deg(self) -> float:
        return math.degrees(self.pitch_angle)

    @property
    def face_angle_deg(self) -> float:
        return math.degrees(self.face_angle)

    @property
    def root_angle_deg(self) -> float:
        return math.degrees(self.root_angle)

    @property
    def angular_pitch(self) -> float:
        """True angular pitch about the gear axis, radians."""
        return 2.0 * math.pi / self.z


@dataclass(frozen=True)
class SetGeometry:
    """Everything derived from a BevelSetParams."""

    params: BevelSetParams
    outer_cone_dist: float      # Ao
    mean_cone_dist: float       # Am
    inner_cone_dist: float      # Ai
    section_scale: float        # k = Ai / Ao
    working_depth: float
    whole_depth: float
    clearance: float
    circular_pitch: float
    pinion: MemberGeometry
    gear: MemberGeometry

    def member(self, which: str) -> MemberGeometry:
        if which == "pinion":
            return self.pinion
        if which == "gear":
            return self.gear
        raise ValueError(f"unknown member {which!r}; expected 'pinion' or 'gear'")


def _solve_2x2(a11, a12, a21, a22, b1, b2) -> tuple[float, float]:
    det = a11 * a22 - a12 * a21
    if abs(det) < 1e-12:
        raise ValueError("singular 2x2 system")
    return (b1 * a22 - a12 * b2) / det, (a11 * b2 - b1 * a21) / det


def tip_radius_at_cone_distance(
    cone_dist: float,
    delta: float,
    delta_a: float,
    crown_R: float,
    crown_z: float,
) -> float:
    """Developed tip radius of the section at the given cone distance.

    Because the Gleason face cone apex is offset from the pitch apex, the tip
    radius is *not* a uniform scaling of the outer one. Intersect the face cone
    with the cone standing perpendicular to the pitch cone at `cone_dist`, in
    the meridian (R, z) plane.

    Face cone:     (R, z) = crown + t * (-sin(delta_a), -cos(delta_a))
    Section cone:  (R, z) = (r*cos(delta), A/cos(delta) - r*sin(delta))

    which rearranges to
        t*sin(delta_a) + r*cos(delta) = crown_R
        t*cos(delta_a) - r*sin(delta) = crown_z - A/cos(delta)

    At cone_dist == Ao this returns exactly the outer tip radius r_p + a, so
    the same solver serves both ends and any overshoot beyond them.
    """
    section_apex_z = cone_dist / math.cos(delta)
    _, r = _solve_2x2(
        math.sin(delta_a), math.cos(delta),
        math.cos(delta_a), -math.sin(delta),
        crown_R, crown_z - section_apex_z,
    )
    return r


def compute_set(p: BevelSetParams) -> SetGeometry:
    """Derive the full geometry of a bevel gear set."""
    sigma = p.sigma

    # Pitch cone angles for an arbitrary shaft angle. atan2 keeps delta1 in
    # (0, pi) so an internal/crown result surfaces as delta1 >= pi/2 rather than
    # silently wrapping; validate.py rejects that case.
    delta1 = math.atan2(math.sin(sigma), p.ratio + math.cos(sigma))
    delta2 = sigma - delta1

    d1 = p.module * p.z1
    d2 = p.module * p.z2
    outer_cone_dist = d1 / (2.0 * math.sin(delta1))

    inner_cone_dist = outer_cone_dist - p.face_width
    mean_cone_dist = outer_cone_dist - p.face_width / 2.0
    scale = inner_cone_dist / outer_cone_dist

    working_depth = WORKING_DEPTH_FACTOR * p.module
    whole_depth = WHOLE_DEPTH_FACTOR * p.module
    clearance = CLEARANCE_FACTOR * p.module

    # Gleason long-and-short addendum: the gear gets the short one.
    equiv_ratio = (p.z2 * math.cos(delta1)) / (p.z1 * math.cos(delta2))
    a2 = 0.54 * p.module + 0.46 * p.module / equiv_ratio
    a1 = working_depth - a2
    bf1 = whole_depth - a1
    bf2 = whole_depth - a2

    # Dedendum angles put both root cones through the pitch apex; the addendum
    # angle of each member is the dedendum angle of its mate, which is what
    # makes the clearance constant along the face width.
    theta_f1 = math.atan(bf1 / outer_cone_dist)
    theta_f2 = math.atan(bf2 / outer_cone_dist)
    theta_a1, theta_a2 = theta_f2, theta_f1

    members = []
    for name, z, delta, d, a, bf, th_a, th_f, mate_d in (
        ("pinion", p.z1, delta1, d1, a1, bf1, theta_a1, theta_f1, d2),
        ("gear", p.z2, delta2, d2, a2, bf2, theta_a2, theta_f2, d1),
    ):
        cos_d, sin_d = math.cos(delta), math.sin(delta)
        delta_a = delta + th_a
        delta_f = delta - th_f

        # Outer pitch point in the meridian plane, then step along the back cone
        # generator: outward by the addendum to reach the crown, inward by the
        # dedendum to reach the outer root point. Those two points bound the
        # blank's back cone face.
        pitch_R = outer_cone_dist * sin_d
        pitch_z = outer_cone_dist * cos_d
        tip_R = pitch_R + a * cos_d
        tip_z = pitch_z - a * sin_d
        root_R = pitch_R - bf * cos_d
        root_z = pitch_z + bf * sin_d

        virtual_pitch_r = (d / 2.0) / cos_d
        members.append(
            MemberGeometry(
                name=name,
                z=z,
                pitch_angle=delta,
                pitch_dia=d,
                addendum=a,
                dedendum=bf,
                addendum_angle=th_a,
                dedendum_angle=th_f,
                face_angle=delta_a,
                root_angle=delta_f,
                virtual_teeth=z / cos_d,
                virtual_pitch_r=virtual_pitch_r,
                virtual_base_r=virtual_pitch_r * math.cos(p.alpha),
                virtual_tip_r=virtual_pitch_r + a,
                virtual_root_r=virtual_pitch_r - bf,
                virtual_tip_r_inner=tip_radius_at_cone_distance(
                    inner_cone_dist, delta, delta_a, tip_R, tip_z
                ),
                outside_dia=d + 2.0 * a * cos_d,
                crown_to_apex=tip_z,
                mounting_distance=mate_d / 2.0 - a * sin_d,
                outer_root_radius=root_R,
                root_to_apex=root_z,
            )
        )

    return SetGeometry(
        params=p,
        outer_cone_dist=outer_cone_dist,
        mean_cone_dist=mean_cone_dist,
        inner_cone_dist=inner_cone_dist,
        section_scale=scale,
        working_depth=working_depth,
        whole_depth=whole_depth,
        clearance=clearance,
        circular_pitch=math.pi * p.module,
        pinion=members[0],
        gear=members[1],
    )


# ---------------------------------------------------------------------------
# Tooth space profile, in the developed (virtual spur gear) plane
# ---------------------------------------------------------------------------

Point2 = tuple[float, float]
Point3 = tuple[float, float, float]


def _polar(r: float, phi: float) -> Point2:
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


def _flank_points(
    r_base: float,
    r_root: float,
    r_tip: float,
    psi0: float,
    half_pitch: float,
    n: int,
) -> list[Point2]:
    """One side of a tooth space, root to tip, at positive angle.

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
        pts.append(_polar(r_root, space_angle(r_base)))

    t_lo = math.sqrt(max(0.0, (r_lo / r_base) ** 2 - 1.0))
    t_hi = math.sqrt(max(0.0, (r_tip / r_base) ** 2 - 1.0))
    for i in range(n):
        t = t_lo + (t_hi - t_lo) * i / (n - 1)
        r = r_base * math.sqrt(1.0 + t * t)
        pts.append(_polar(r, space_angle(r)))

    return pts


def _point_segment_distance(p: Point2, a: Point2, b: Point2) -> tuple[float, Point2]:
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


def _distance_to_polyline(p: Point2, poly: list[Point2]) -> tuple[float, Point2, int]:
    best = (float("inf"), poly[0], 0)
    for i in range(len(poly) - 1):
        d, q = _point_segment_distance(p, poly[i], poly[i + 1])
        if d < best[0]:
            best = (d, q, i)
    return best


def _root_fillet(
    flank: list[Point2], r_root: float, rho: float, arc_points: int = 9
) -> tuple[list[Point2], list[Point2]] | None:
    """Fit a circular fillet of radius `rho` tangent to the flank and the root.

    Returns (trimmed_flank, arc_points_root_to_flank), or None if no fillet of
    that size fits inside the space - in which case the caller keeps a sharp
    corner.

    The fillet centre must sit at radius r_root + rho (tangency with the root
    circle), so only its angle is unknown. Distance-to-flank decreases
    monotonically as that angle sweeps from the space centreline toward the
    flank, so a bisection is both safe and simple.
    """
    if rho <= 0.0:
        return None

    phi_flank = math.atan2(flank[0][1], flank[0][0])
    if phi_flank <= 0.0:
        return None

    def gap(phi: float) -> float:
        c = _polar(r_root + rho, phi)
        return _distance_to_polyline(c, flank)[0] - rho

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

    centre = _polar(r_root + rho, phi_c)
    _, touch, seg = _distance_to_polyline(centre, flank)

    # Tangent point on the root circle is radially inward from the centre.
    root_touch = _polar(r_root, phi_c)

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


@dataclass(frozen=True)
class ToothSpaceSection:
    """One end section of a single tooth space.

    `segments` keeps the boundary broken into named pieces so the SOLIDWORKS
    layer can build splines and arcs separately rather than forcing one spline
    through sharp corners. `loop_2d` is the same boundary flattened into a
    closed polyline, which is what the preview and DXF export want.
    """

    member: str
    end: str                    # "outer" | "inner"
    cone_apex_z: float          # axial position of this section's cone apex
    pitch_angle: float
    r_root: float
    r_tip: float
    r_cap: float
    filleted: bool
    segments: dict[str, list[Point2]] = field(default_factory=dict)
    loop_2d: list[Point2] = field(default_factory=list)

    def loop_3d(self) -> list[Point3]:
        return [
            to_cone_3d(x, y, self.pitch_angle, self.cone_apex_z) for x, y in self.loop_2d
        ]

    def segments_3d(self) -> dict[str, list[Point3]]:
        return {
            name: [
                to_cone_3d(x, y, self.pitch_angle, self.cone_apex_z) for x, y in pts
            ]
            for name, pts in self.segments.items()
        }


def to_cone_3d(x: float, y: float, delta: float, cone_apex_z: float) -> Point3:
    """Map a point of the developed virtual spur gear onto the real cone.

    A point at developed radius r and developed angle phi lands at

        R     = r * cos(delta)              distance from the gear axis
        theta = phi / cos(delta)            true angle about the axis
        z     = cone_apex_z - r * sin(delta)

    The 1/cos(delta) on the angle is what turns z_v teeth in the flat
    development into z teeth around the real gear. Arc length is preserved:
    R * theta == r * phi.
    """
    r = math.hypot(x, y)
    phi = math.atan2(y, x)
    R = r * math.cos(delta)
    theta = phi / math.cos(delta)
    return R * math.cos(theta), R * math.sin(theta), cone_apex_z - r * math.sin(delta)


def tooth_space_section(
    geo: SetGeometry,
    member: str,
    end: str = "outer",
    n_flank: int = FLANK_POINTS,
    overshoot: float = 0.0,
) -> ToothSpaceSection:
    """Build one end section of a tooth space, centred on angle 0.

    The loop runs counter-clockwise: up the negative flank, out past the tip,
    across the cap, back down the positive flank, then round the root.

    `overshoot` pushes the section past the end of the face width, in mm - out
    beyond Ao for the outer end, in below Ai for the inner. Pure geometry uses
    0; the SOLIDWORKS builder passes `end_overshoot(geo)` so the loft cut does
    not finish tangent to a face of the blank.
    """
    if end not in ("outer", "inner"):
        raise ValueError(f"end must be 'outer' or 'inner', got {end!r}")

    p = geo.params
    m = geo.member(member)

    if end == "outer":
        cone_dist = geo.outer_cone_dist + overshoot
    else:
        # Never let the overshoot walk the section through the pitch apex.
        cone_dist = max(
            geo.inner_cone_dist - overshoot, 0.05 * geo.outer_cone_dist
        )
    k = cone_dist / geo.outer_cone_dist

    # Root radii and the involute base scale uniformly about the pitch apex,
    # because the Gleason root cone apexes there. The tip radius does not, so
    # it is solved against the face cone - see the module docstring.
    r_base = k * m.virtual_base_r
    r_root = k * m.virtual_root_r

    # Angular constants are scale-invariant: acos(k*r_b / k*r) == acos(r_b / r),
    # so the same psi0 and half_pitch serve every section.
    tooth_thickness = math.pi * p.module / 2.0 - p.backlash / 2.0
    psi0 = tooth_thickness / (2.0 * m.virtual_pitch_r) + inv(p.alpha)
    half_pitch = math.pi / m.virtual_teeth

    # Clamp the tip so the tooth never goes pointed - see MIN_TOP_LAND_FACTOR.
    r_tip = tip_radius_at_cone_distance(
        cone_dist, m.pitch_angle, m.face_angle, m.outside_dia / 2.0, m.crown_to_apex
    )
    r_tip = min(
        r_tip, max_tip_radius(r_base, psi0, MIN_TOP_LAND_FACTOR * p.module * k)
    )
    r_cap = r_tip + CUT_OVERSHOOT_FACTOR * p.module * max(k, 0.2)

    cone_apex_z = cone_dist / math.cos(m.pitch_angle)

    flank = _flank_points(r_base, r_root, r_tip, psi0, half_pitch, n_flank)

    fillet = _root_fillet(flank, r_root, p.fillet_factor * p.module * k)
    if fillet is not None:
        flank, arc = fillet
    else:
        arc = []

    # Mirror across the x axis for the other side of the space.
    def mirror(pts: list[Point2]) -> list[Point2]:
        return [(x, -y) for x, y in pts]

    phi_root = math.atan2(arc[0][1], arc[0][0]) if arc else math.atan2(
        flank[0][1], flank[0][0]
    )
    phi_tip = math.atan2(flank[-1][1], flank[-1][0])

    root_arc = [
        _polar(r_root, -phi_root + 2.0 * phi_root * i / 8.0) for i in range(9)
    ]
    cap = [_polar(r_cap, -phi_tip + 2.0 * phi_tip * i / 4.0) for i in range(5)]

    segments = {
        # Root to flank, matching the direction of travel round the loop; the
        # positive side runs the other way and so is reversed instead.
        "fillet_neg": mirror(arc) if arc else [],
        "flank_neg": mirror(flank),
        "riser_neg": [mirror(flank)[-1], cap[0]],
        "cap": cap,
        "riser_pos": [cap[-1], flank[-1]],
        "flank_pos": flank[::-1],
        "fillet_pos": arc[::-1] if arc else [],
        "root": root_arc[::-1],
    }

    loop: list[Point2] = []
    for name in (
        "fillet_neg", "flank_neg", "riser_neg", "cap",
        "riser_pos", "flank_pos", "fillet_pos", "root",
    ):
        for pt in segments[name]:
            if not loop or math.dist(loop[-1], pt) > 1e-9:
                loop.append(pt)
    if loop and math.dist(loop[0], loop[-1]) < 1e-9:
        loop.pop()

    return ToothSpaceSection(
        member=member,
        end=end,
        cone_apex_z=cone_apex_z,
        pitch_angle=m.pitch_angle,
        r_root=r_root,
        r_tip=r_tip,
        r_cap=r_cap,
        filleted=bool(arc),
        segments=segments,
        loop_2d=loop,
    )


# ---------------------------------------------------------------------------
# Blank
# ---------------------------------------------------------------------------


def blank_outline(geo: SetGeometry, member: str) -> list[Point2]:
    """Meridian half-section of the gear blank, as (R, z) with the apex at 0.

    Revolving this about the Z axis gives the un-toothed body. The profile runs:
    bore at the front face, out across the front face, up the face cone to the
    crown, **back down the back cone** to the outer root point, then flat home
    along the bore, with an optional hub boss behind.

    Why the back cone rather than a plane
    -------------------------------------
    The back cone is perpendicular to the pitch cone at Ao, so *both* members of
    a pair have their back cones perpendicular to the same pitch generator, in
    the same meridian plane - the two back cones share that generator and are
    tangent along it. The consequence is the one that makes a meshed pair look
    right: the outer tip of every tooth lands exactly on the **mate's** back
    cone, flush, with nothing sticking out.

    Truncating the large end with a plane perpendicular to the axis instead
    leaves each member's teeth hanging past the back of the mate by the working
    depth resolved onto the mate's axis - on the anchor set, the pinion teeth
    overhang the gear by 2*m*cos(delta1) = 3.72 mm and the gear teeth overhang
    the pinion by 2*m*sin(delta1) = 1.47 mm. That is the overhang this profile
    removes.

    The back cone only runs over the tooth depth - from the crown at
    `outside_dia/2` down to the outer root point - and behind that the blank is
    flat. `hub_thickness` is measured from that flat back, which is what the
    parameter has always claimed ("backing behind the outer root point").

    SOLIDWORKS note: the outer loft section lies *on* this back cone, so a cut
    ending there would be tangent to a real face and get rejected as
    zero-thickness geometry. `end_overshoot` pushes it clear, and it clears by a
    uniform distance because section and back cone are concentric - see there.
    """
    p = geo.params
    m = geo.member(member)
    cos_d = math.cos(m.pitch_angle)

    r_bore = p.bore / 2.0
    min_wall = max(p.module, 1.0)

    tip_R = m.outside_dia / 2.0          # crown, the widest point of the blank
    z_crown = m.crown_to_apex
    root_R = m.outer_root_radius         # where the back cone stops
    z_back = m.root_to_apex              # the flat back sits here
    inner_R = m.virtual_tip_r_inner * cos_d
    z_front = front_face_z(geo, m)

    outline = [
        (r_bore, z_front),
        (inner_R, z_front),
        (tip_R, z_crown),
        (root_R, z_back),
    ]

    if p.hub_thickness > 0.0:
        # Keep the hub inside the root cone so the tooth cut never grazes it.
        # `root_R` is the root cone's radius at exactly this plane.
        hub_R = max(
            r_bore + min_wall,
            min(r_bore + 2.0 * min_wall, 0.7 * root_R),
        )
        outline.append((hub_R, z_back))
        outline.append((hub_R, z_back + p.hub_thickness))
        outline.append((r_bore, z_back + p.hub_thickness))
    else:
        outline.append((r_bore, z_back))

    return outline
