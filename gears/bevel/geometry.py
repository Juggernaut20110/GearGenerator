"""Bevel gear geometry, straight and spiral. Pure math - no COM, no GUI, no I/O.

Units are millimetres and radians throughout. The SOLIDWORKS layer converts to
metres at its own boundary; nothing here knows about that.

3D coordinate system
--------------------
The gear axis is +Z and the **pitch apex sits at the origin**. The toothed body
extends toward +Z. Putting the apex at the origin is what makes the inner
section a plain uniform scaling of the outer one (see `SECTION_SCALE` below).

The tooth trace
---------------
A straight bevel tooth runs along a cone generator; a spiral one follows a
circular arc laid in the generating crown gear's plane. That arc is `CrownTrace`,
and it enters the geometry as a single **phase** rotation on each section: the
section keeps its shape and its cone distance and turns about the gear axis by
however far the trace has curved by that point along the face.

Straight teeth are not a special case anywhere in this module. They are the
absence of a trace, which makes every phase exactly zero and every section a
plain scaling of the outer one, and the same code path builds both.

Two cone constructions are in play and they are not the same one, which is the
thing to keep straight. Tredgold's back cone is about the **profile** - it turns
z teeth into z/cos(delta) virtual ones and gives the flank its shape. The crown
gear is about the **trace** - it says where along the face that profile sits.
`to_cone_3d` applies them in that order and says why.

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

from ..involute import (
    CUT_OVERSHOOT_FACTOR,
    FLANK_POINTS,
    MIN_TOP_LAND_FACTOR,
    inv,
    max_tip_radius,
    polar,
    tooth_space_loop,
    top_land,
)
from .params import BevelSetParams

# Gleason straight-bevel proportions, as multiples of the outer module.
WORKING_DEPTH_FACTOR = 2.000
WHOLE_DEPTH_FACTOR = 2.188
CLEARANCE_FACTOR = 0.188

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

# How far a lofted flank may fall inside the true swept surface, in mm.
#
# The same tolerance and the same argument as the helical spur builder's, which
# is where this was measured: a loft carries each profile point from one section
# to the next along a straight chord, and the trace between them is an arc, so
# the swept surface sits inside the real one by the chord's sagitta -
# r * (1 - cos(delta / 2)) at radius r over a rotation of delta. A guide curve
# pins the one point it runs through and leaves the rest interpolated, so it
# helps and does not fix it.
#
# A straight bevel gear has no rotation between sections at all and still takes
# exactly two, so this costs nothing until there is a spiral to follow.
MAX_SECTION_SAGITTA_MM = 0.02


# ---------------------------------------------------------------------------
# The spiral tooth trace
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class CrownTrace:
    """The tooth trace, as an arc in the generating crown gear's plane.

    A face-milling cutter of radius `r_c` sweeps a circular arc across the
    crown gear - the imaginary 90-degree bevel gear both members are generated
    against. Each real member's trace is that one arc mapped onto its own pitch
    cone, and this class holds the arc.

    The arc's centre sits at distance `rho` from the crown centre. For a point P
    on the arc at crown radius A, the triangle (crown centre O, arc centre C,
    point P) has sides rho, r_c and A, so everything falls out of the cosine
    rule:

        spiral angle    sin psi(A) = (A^2 + r_c^2 - rho^2) / (2*A*r_c)
        trace angle   theta_c(A) = acos((rho^2 + A^2 - r_c^2) / (2*rho*A))

    The spiral angle is the angle between the trace and the cone generator,
    which is 90 degrees minus the angle OPC - and the tangent at P is
    perpendicular to CP, which is what turns that into the sine above.

    Solving the first for `rho` at the mean cone distance is what places the arc:

        rho^2 = Am^2 + r_c^2 - 2*Am*r_c*sin(psi_m)

    Why one arc serves both members
    -------------------------------
    A point of the crown plane at radius A and angle theta_c maps onto a pitch
    cone of angle delta at cone distance A and **true** angle
    theta = theta_c / sin(delta) - which is just the statement that developing
    the cone into a plane is what the crown gear is.

    So the arc length swept along the pitch circle at cone distance A is

        R * theta = (A * sin delta) * (theta_c / sin delta) = A * theta_c

    the same for both members, at every cone distance, whatever their pitch
    angles. The two traces therefore coincide along the common pitch generator
    by construction - which is the meshing condition - and no sign anywhere has
    to be chosen to make that happen.

    It is also where the opposite hands come from. Nothing flips the gear's
    trace; mapping one arc through two different pitch angles is the whole of
    it. The pinion's small delta divides by a small sine and sweeps far - 38.790
    degrees over the face of the anchor 17-tooth pinion, against 15.336 on its
    43-tooth mate - and looking down each member's own axis in the assembled
    pair, those two sweeps read as opposite hands.

    **Which sign is the right hand has not been measured.** The arithmetic is
    symmetric, so nothing here can settle it: it needs someone to build the
    anchor pinion and look down its axis in SOLIDWORKS. Until then `hand` names
    a direction consistently without any claim about which one a catalogue would
    call right - and the pair meshes either way, because both members take their
    hand from the same sign.
    """

    cutter_radius: float        # r_c
    centre_distance: float      # rho, crown centre to arc centre
    mean_cone_dist: float       # Am, where the mean spiral angle is quoted
    sign: float                 # +1 right hand, -1 left, on the PINION

    @classmethod
    def for_set(
        cls, psi_m: float, cutter_radius: float, mean_cone_dist: float
    ) -> "CrownTrace":
        """Place the arc that delivers `psi_m` at the mean cone distance."""
        rho_sq = (
            mean_cone_dist ** 2
            + cutter_radius ** 2
            - 2.0 * mean_cone_dist * cutter_radius * math.sin(abs(psi_m))
        )
        return cls(
            cutter_radius=cutter_radius,
            centre_distance=math.sqrt(max(0.0, rho_sq)),
            mean_cone_dist=mean_cone_dist,
            sign=-1.0 if psi_m < 0.0 else 1.0,
        )

    def spiral_angle_at(self, cone_dist: float) -> float:
        """Spiral angle at a cone distance, radians. Signed by hand.

        Varies along the face, which is the whole difference between a real
        face-milled trace and the constant-angle idealisation. Measured on the
        anchor set at 35 degrees mean: 30.074 at the toe, 40.599 at the heel.
        """
        r_c = self.cutter_radius
        s = (cone_dist ** 2 + r_c ** 2 - self.centre_distance ** 2) / (
            2.0 * cone_dist * r_c
        )
        return self.sign * math.asin(max(-1.0, min(1.0, s)))

    def theta_at(self, cone_dist: float) -> float:
        """Angle of the trace in the crown plane, radians, measured from Am.

        Zero at the mean cone distance by construction, so the mean section is
        the clocking reference for the whole face - the bevel analogue of the
        spur gear's "the front face sits at z = 0".
        """
        return self._theta_raw(cone_dist) - self._theta_raw(self.mean_cone_dist)

    def _theta_raw(self, cone_dist: float) -> float:
        rho, r_c = self.centre_distance, self.cutter_radius
        if rho <= 0.0 or cone_dist <= 0.0:
            return 0.0
        c = (rho ** 2 + cone_dist ** 2 - r_c ** 2) / (2.0 * rho * cone_dist)
        return math.acos(max(-1.0, min(1.0, c)))

    def reaches(self, inner: float, outer: float) -> bool:
        """Whether the arc actually spans a face running from `inner` to `outer`.

        An arc only exists between crown radii |rho - r_c| and rho + r_c. Outside
        that band there is no point of the circle at that radius at all, and
        `theta_at` would clamp to the nearest end and quietly hand back a trace
        that stops following the cutter. The validator turns this into an error.
        """
        lo = abs(self.centre_distance - self.cutter_radius)
        hi = self.centre_distance + self.cutter_radius
        return lo <= inner and outer <= hi


def phase_at_cone_distance(
    geo: "SetGeometry", member: str, cone_dist: float
) -> float:
    """How far the section at `cone_dist` is rotated about the gear axis, radians.

    The crown-plane trace angle divided by sin(delta) - see `CrownTrace`. Zero at
    the mean cone distance, and zero everywhere for a straight bevel gear, which
    is why nothing downstream needs to ask which kind it is building.
    """
    if geo.trace is None:
        return 0.0
    m = geo.member(member)
    sin_d = math.sin(m.pitch_angle)
    if abs(sin_d) < 1e-12:
        return 0.0
    return geo.trace.sign * geo.trace.theta_at(cone_dist) / sin_d


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


def blank_reach_past_back_cone(geo: "SetGeometry", member: str) -> float:
    """How far the blank sticks out past the back cone, where the cut can see it.

    Zero when the blank ends on the back cone. With a root rim it is the rim's
    far corner that reaches furthest - `min_root_thickness * cos(delta)`, since
    the near corner sits on the cone - but this measures the outline rather than
    assuming which vertex wins, so it stays right if the outline changes shape.

    Only the blank at or outside the outer root radius counts. The hub reaches
    much further past the cone than the rim ever does - 2.4 mm against 0.5 on
    the anchor pinion, because a steep back cone runs away from a tall boss near
    the axis - but it sits radially *inside* the whole outer section (the
    property `test_loft_sections_clear_the_blank` pins down), so the loft never
    meets it and there is nothing to clear.
    """
    m = geo.member(member)
    return max(
        (
            beyond_back_cone(geo, m, (R, 0.0, z))
            for R, z in blank_outline(geo, member)
            if R >= m.outer_root_radius - 1e-9
        ),
        default=0.0,
    )


def end_overshoot(geo: "SetGeometry", member: str, margin: float | None = None) -> float:
    """Smallest overshoot that puts both loft sections clear of the blank, mm.

    A fixed fraction of the face width is not enough on its own: whether the
    inner section clears the flat front face depends on the cone angle and the
    tooth depth. So start from the fraction and grow until the property actually
    holds - measured on the real section points rather than assumed.

    The outer end used to be the easy one, back when the blank ended on the back
    cone: the section and the cone are the same family, so every point of the
    section cleared by exactly the overshoot. The root rim breaks that. It puts
    material `min_root_thickness * cos(delta)` *behind* the back cone, and a
    section that only clears the cone would leave that rim uncut - a ring of
    material bridging the tooth spaces at the heel. So the outer end is measured
    against the blank, not against the cone.

    It is not a hypothetical: on the anchor set the pinion settles at 1.505 mm
    of overshoot against a rim that reaches 0.930 mm per mm of thickness, so a
    1.5 mm rim would have overrun it. The gear, at 7.176 mm against 0.368, has
    room to spare.
    """
    p = geo.params
    m = geo.member(member)
    if margin is None:
        margin = max(0.2, 0.1 * p.module)

    z_front = front_face_z(geo, m)     # the flat front face
    reach = blank_reach_past_back_cone(geo, member)

    over = max(END_OVERSHOOT_MIN_MM, END_OVERSHOOT_FRACTION * p.face_width)
    for _ in range(40):
        outer = tooth_space_section(geo, member, "outer", overshoot=over)
        inner = tooth_space_section(geo, member, "inner", overshoot=over)
        clear_back = (
            min(beyond_back_cone(geo, m, pt) for pt in outer.loop_3d())
            > reach + margin
        )
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

    # The tooth trace, or None for a straight bevel gear. None rather than a
    # degenerate arc on purpose: a straight tooth is not an arc of infinite
    # radius as far as this code is concerned, it is the absence of a trace, and
    # every phase it would contribute is exactly zero rather than nearly so.
    trace: CrownTrace | None = None

    @property
    def face_contact_ratio(self) -> float:
        """How much of a pitch the contact advances along the face. Zero if straight.

        `b * tan(psi_m) / p_m` - the face advance at the mean cone distance over
        the circular pitch there. This is what a spiral buys: it overlaps the
        handover from one tooth to the next, the same way a helix does on a spur
        gear, and it is why a spiral bevel pair is quieter than a straight one at
        the same tooth count. 1.818 on the anchor set at 35 degrees.
        """
        if self.trace is None:
            return 0.0
        p_m = self.circular_pitch * self.mean_cone_dist / self.outer_cone_dist
        if p_m <= 0.0:
            return 0.0
        psi_m = self.trace.spiral_angle_at(self.mean_cone_dist)
        return abs(self.params.face_width * math.tan(psi_m) / p_m)

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

    # One arc, shared by both members - see CrownTrace for why that is the whole
    # of the meshing condition rather than half of it.
    trace = (
        CrownTrace.for_set(
            p.psi_m,
            p.cutter_radius if p.cutter_radius is not None else mean_cone_dist,
            mean_cone_dist,
        )
        if p.is_curved
        else None
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
        trace=trace,
    )


# ---------------------------------------------------------------------------
# Tooth space profile, in the developed (virtual spur gear) plane
# ---------------------------------------------------------------------------

Point2 = tuple[float, float]
Point3 = tuple[float, float, float]


@dataclass(frozen=True)
class ToothSpaceSection:
    """One end section of a single tooth space.

    `segments` keeps the boundary broken into named pieces so the SOLIDWORKS
    layer can build splines and arcs separately rather than forcing one spline
    through sharp corners. `loop_2d` is the same boundary flattened into a
    closed polyline, which is what the preview and DXF export want.
    """

    member: str
    end: str                    # "outer" | "inner" | "mid" for anything between
    cone_apex_z: float          # axial position of this section's cone apex
    pitch_angle: float
    r_root: float
    r_tip: float
    r_cap: float
    filleted: bool
    cone_dist: float = 0.0      # where along the pitch cone this section sits
    phase: float = 0.0          # rotation the trace has reached here, radians
    segments: dict[str, list[Point2]] = field(default_factory=dict)
    loop_2d: list[Point2] = field(default_factory=list)

    def loop_3d(self) -> list[Point3]:
        return [
            to_cone_3d(x, y, self.pitch_angle, self.cone_apex_z, self.phase)
            for x, y in self.loop_2d
        ]

    def segments_3d(self) -> dict[str, list[Point3]]:
        return {
            name: [
                to_cone_3d(x, y, self.pitch_angle, self.cone_apex_z, self.phase)
                for x, y in pts
            ]
            for name, pts in self.segments.items()
        }


def to_cone_3d(
    x: float, y: float, delta: float, cone_apex_z: float, phase: float = 0.0
) -> Point3:
    """Map a point of the developed virtual spur gear onto the real cone.

    A point at developed radius r and developed angle phi lands at

        R     = r * cos(delta)              distance from the gear axis
        theta = phi / cos(delta) + phase    true angle about the axis
        z     = cone_apex_z - r * sin(delta)

    The 1/cos(delta) on the angle is what turns z_v teeth in the flat
    development into z teeth around the real gear. Arc length is preserved:
    R * theta == r * phi.

    `phase` is where the spiral enters, and it enters here and nowhere else: the
    section itself is the same shape it always was, sitting at the same cone
    distance, turned about the gear axis by however far the tooth trace has
    curved by this point along the face. Zero for a straight bevel gear, so the
    argument's default is the whole of the straight case.

    Note which mapping the phase is added *after*. The 1/cos(delta) belongs to
    Tredgold's back-cone development and is about the tooth's profile; the phase
    belongs to the crown-gear trace and is about where along the face that
    profile sits. They are two different cone constructions and adding the phase
    before the division would silently divide it by cos(delta) as well.
    """
    r = math.hypot(x, y)
    phi = math.atan2(y, x)
    R = r * math.cos(delta)
    theta = phi / math.cos(delta) + phase
    return R * math.cos(theta), R * math.sin(theta), cone_apex_z - r * math.sin(delta)


def tooth_space_section(
    geo: SetGeometry,
    member: str,
    end: str = "outer",
    n_flank: int = FLANK_POINTS,
    overshoot: float = 0.0,
    cone_dist: float | None = None,
    split_cap: bool = False,
) -> ToothSpaceSection:
    """Build one section of a tooth space, centred on angle 0 at the mean.

    The loop runs counter-clockwise: up the negative flank, out past the tip,
    across the cap, back down the positive flank, then round the root.

    `overshoot` pushes the section past the end of the face width, in mm - out
    beyond Ao for the outer end, in below Ai for the inner. Pure geometry uses
    0; the SOLIDWORKS builder passes `end_overshoot(geo)` so the loft cut does
    not finish tangent to a face of the blank.

    `cone_dist` names the position outright and overrides `end` and `overshoot`
    together. A straight bevel gear only ever needs the two ends; a spiral one
    needs as many sections in between as `section_cone_distances` asks for, and
    those have no end to be named after.
    """
    if cone_dist is None:
        if end not in ("outer", "inner"):
            raise ValueError(f"end must be 'outer' or 'inner', got {end!r}")
        if end == "outer":
            cone_dist = geo.outer_cone_dist + overshoot
        else:
            # Never let the overshoot walk the section through the pitch apex.
            cone_dist = max(
                geo.inner_cone_dist - overshoot, 0.05 * geo.outer_cone_dist
            )
    else:
        cone_dist = max(cone_dist, 0.05 * geo.outer_cone_dist)
        end = "mid"

    p = geo.params
    m = geo.member(member)
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

    # The fillet radius scales with the section, like everything else that is a
    # length rather than an angle.
    segments, loop, filleted = tooth_space_loop(
        r_base, r_root, r_tip, r_cap, psi0, half_pitch,
        p.fillet_factor * p.module * k, n_flank, split_cap=split_cap,
    )

    return ToothSpaceSection(
        member=member,
        end=end,
        cone_apex_z=cone_apex_z,
        pitch_angle=m.pitch_angle,
        r_root=r_root,
        r_tip=r_tip,
        r_cap=r_cap,
        filleted=filleted,
        cone_dist=cone_dist,
        phase=phase_at_cone_distance(geo, member, cone_dist),
        segments=segments,
        loop_2d=loop,
    )


def section_span(geo: SetGeometry, member: str) -> tuple[float, float]:
    """The cone distances the loft has to run between, heel first.

    Both ends pushed past the blank by `end_overshoot`, so the cut never
    finishes tangent to a real face. This is the span the section count has to
    be solved over - **not** the face width, which is the shorter interval
    between them.
    """
    overshoot = end_overshoot(geo, member)
    return (
        geo.outer_cone_dist + overshoot,
        max(geo.inner_cone_dist - overshoot, 0.05 * geo.outer_cone_dist),
    )


def _worst_sagitta(
    geo: SetGeometry, member: str, distances: list[float]
) -> float:
    """How far the loft's worst interval falls inside the true swept surface, mm.

    The chord between two sections rotated `delta` apart sits
    `r * (1 - cos(delta / 2))` inside the arc it should follow, at radius r.
    Evaluated at the tip radius about the gear axis - the furthest any point of
    the cut gets from the axis it is turning about, and so the worst case.

    **Only intervals that reach the blank are counted.** Both ends of the run
    are pushed well past the material by `end_overshoot` - 7.18 mm at the
    gear's toe against a 13.87 mm face - and a chord out there bounds a piece of
    cut that removes nothing, so its accuracy buys nothing either. Counting it
    anyway is not merely wasteful: the trace turns fastest at small cone
    distances, so the interval furthest past the toe is always the worst one,
    and sizing the whole loft by it doubles the section count to chase a
    tolerance in fresh air.
    """
    r_tip_axis = geo.member(member).outside_dia / 2.0
    face_lo, face_hi = geo.inner_cone_dist, geo.outer_cone_dist
    phases = [phase_at_cone_distance(geo, member, A) for A in distances]

    worst = 0.0
    for (a, b), (lo, hi) in zip(zip(distances, distances[1:]), zip(phases, phases[1:])):
        if max(a, b) < face_lo or min(a, b) > face_hi:
            continue        # entirely outside the blank; cuts nothing
        worst = max(worst, r_tip_axis * (1.0 - math.cos(abs(hi - lo) / 2.0)))
    return worst


def section_cone_distances(
    geo: SetGeometry,
    member: str,
    max_sagitta: float = MAX_SECTION_SAGITTA_MM,
) -> list[float]:
    """Where along the pitch cone the loft sections sit, heel first.

    Evenly spaced across `section_span`, with as many as it takes to keep every
    interval's chord inside `max_sagitta` of the true swept surface. A straight
    bevel gear gets exactly the two ends, which is the pair the builder has
    always used - so its output does not move.

    **The count is measured, not solved.** The obvious closed form - total sweep
    over the largest step the tolerance allows - is wrong here for two reasons,
    and the first one cost a test:

    * the sections span more than the face width. Both ends are pushed out by
      `end_overshoot`, so solving the count over `Ao - Ai` sizes it for a
      shorter run than the loft actually makes.
    * the rotation is **not linear** in cone distance the way a helix's is in z.
      The trace's angle changes fastest at small cone distances, so evenly
      spaced sections rotate unevenly and the interval nearest the toe is always
      the worst one. An average-sized count leaves that interval over tolerance.

    Growing the count until the property holds on the sections themselves settles
    both, and it is the same tactic `end_overshoot` uses a few functions up: this
    module would rather measure a thing than assume it.

    Measured on the anchor spiral set at 35 degrees: **11 sections each**. The
    two landing on the same number is a coincidence of two effects cancelling,
    and worth knowing about because the naive expectation is wrong. The pinion
    sweeps far more - 38.790 degrees against the gear's 15.336 - but the gear's
    tip stands much further from the axis it is turning about, 43.735 mm against
    18.860, and the sagitta is proportional to that radius. A straight bevel set
    takes two either way.
    """
    a_hi, a_lo = section_span(geo, member)

    def spread(n: int) -> list[float]:
        return [a_hi + (a_lo - a_hi) * i / (n - 1) for i in range(n)]

    if geo.trace is None or max_sagitta <= 0.0:
        return spread(2)

    n = 2
    for _ in range(200):
        distances = spread(n)
        if _worst_sagitta(geo, member, distances) <= max_sagitta:
            return distances
        n += 1
    return spread(n)


def section_count(
    geo: SetGeometry,
    member: str,
    max_sagitta: float = MAX_SECTION_SAGITTA_MM,
) -> int:
    """How many sections the loft needs to follow the trace closely enough.

    Two for a straight bevel gear, which has no rotation between its sections at
    all; 11 for each member of the anchor spiral set. See
    `section_cone_distances` for why the number is arrived at by measurement
    rather than by a closed form.
    """
    return len(section_cone_distances(geo, member, max_sagitta))


def guide_spiral(
    geo: SetGeometry,
    member: str,
    a_hi: float,
    a_lo: float,
    points: int = 61,
) -> list[Point3]:
    """The loft guide curve: the path of the cap's centreline vertex.

    The same construction as the helical spur builder's `guide_helix`, and it is
    there for the same reason: a guided loft needs its guide to touch a point
    that *exists* in every profile, and a point that merely lies near a spline is
    the classic way such a loft fails. The cap of the tooth-space section is
    symmetric about the space centreline, so the point at developed angle 0
    exists in every section, and `split_cap` puts a real vertex there rather
    than leaving it somewhere along a spline.

    Unlike the spur case this is not a helix - the cap radius scales with the
    cone distance while the phase follows the arc - so it is sampled and handed
    over as a spline, which is what the builder does with every other curve.
    """
    return [
        _cap_vertex(geo, member, a_hi + (a_lo - a_hi) * i / (points - 1))
        for i in range(points)
    ]


def _cap_vertex(geo: SetGeometry, member: str, cone_dist: float) -> Point3:
    """Where the cap's centreline vertex sits at one cone distance."""
    section = tooth_space_section(geo, member, cone_dist=cone_dist)
    return to_cone_3d(
        section.r_cap, 0.0, section.pitch_angle, section.cone_apex_z, section.phase
    )


# ---------------------------------------------------------------------------
# Blank
# ---------------------------------------------------------------------------


def blank_outline(geo: SetGeometry, member: str) -> list[Point2]:
    """Meridian half-section of the gear blank, as (R, z) with the apex at 0.

    Revolving this about the Z axis gives the un-toothed body. The profile runs:
    bore at the front face, out across the front face, up the face cone to the
    crown, **back down the back cone** to the outer root point, straight back
    along the root rim, then flat home along the bore, with an optional hub boss
    behind.

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

    The root rim
    ------------
    The outer root point is where the tooth root emerges on the back cone, so
    running the flat back straight through it leaves *zero* material under the
    root at the heel: a wedge of included angle 90 - root_angle, tapering to a
    knife edge. On a 17/43 set that is 70 degrees on the pinion and 25 on the
    gear, where the rim stays under 1 mm for the last 2.1 mm of radius. It gets
    worse as the ratio climbs, because the gear's root cone lies down towards
    its own back face.

    `min_root_thickness` inserts a short cylinder at the outer root radius
    before the flat back, so the rim is at least that thick everywhere under the
    teeth. A cylinder specifically: extending the back cone further, or offsetting
    a cone parallel to the root cone, leaves the two surfaces still meeting at a
    point and only opens the wedge (to 43 degrees on the anchor gear) rather than
    removing it. The cylinder meets the back cone at 158 degrees instead.

    It grows the blank *away* from the mate - the rim sits behind the back cone,
    which is the face the mate's teeth land on - so it costs no meshing
    clearance. What it does cost is loft overshoot; see `end_overshoot`.

    SOLIDWORKS note: the outer loft section lies *on* this back cone, so a cut
    ending there would be tangent to a real face and get rejected as
    zero-thickness geometry. `end_overshoot` pushes it clear - past the rim as
    well as past the cone, which is no longer the same distance.
    """
    p = geo.params
    m = geo.member(member)
    cos_d = math.cos(m.pitch_angle)

    r_bore = p.bore / 2.0
    min_wall = max(p.module, 1.0)

    tip_R = m.outside_dia / 2.0          # crown, the widest point of the blank
    z_crown = m.crown_to_apex
    root_R = m.outer_root_radius         # where the back cone stops
    z_root = m.root_to_apex              # the outer root point
    z_back = z_root + max(0.0, p.min_root_thickness)   # the flat back sits here
    inner_R = m.virtual_tip_r_inner * cos_d
    z_front = front_face_z(geo, m)

    outline = [
        (r_bore, z_front),
        (inner_R, z_front),
        (tip_R, z_crown),
        (root_R, z_root),
    ]
    if z_back > z_root:
        outline.append((root_R, z_back))

    if p.hub_thickness > 0.0:
        # Keep the hub inside the root cone so the tooth cut never grazes it.
        # `root_R` is the root cone's radius at the outer root point; at the
        # flat back, a root rim further along the cone, it is wider still - so
        # measuring against `root_R` errs on the safe side.
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
