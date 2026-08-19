"""Involute spur gear geometry, straight and helical. Pure math - no COM, no GUI.

Units are millimetres and radians throughout. The SOLIDWORKS layer converts to
metres at its own boundary; nothing here knows about that.

3D coordinate system
--------------------
The gear axis is +Z and the **front face sits at z = 0**, with the body running
to z = face_width. A tooth **space** is centred on +X *at z = 0*, so z = 0 is the
clocking reference plane for the whole pair - the plane a mating pinion and gear
are phased against. Every other transverse section is that one rotated by the
twist that has accumulated by its own z.

Straight teeth are not a special case anywhere in this module. They are
helix_angle = 0, which makes the twist zero and every section identical; the
same code path builds both.

Tooth form
----------
A true involute in the **transverse** plane, from `gears.involute`. The inputs
that reach the flank are the transverse module and pressure angle, so a helical
gear differs from a straight one only in that those two are larger:

    m_t     = m_n / cos(beta)
    alpha_t = atan(tan(alpha_n) / cos(beta))

Proportions are the plain ISO ones on the **normal** module - addendum 1.0 m_n,
dedendum 1.25 m_n - because that is what a cutter of a given normal module
produces. Note the asymmetry this creates and do not tidy it away: the depths
are normal quantities, the radii they are measured from are transverse ones.

The twist, and why the gear is wound the other way
--------------------------------------------------
Over the face width a helical tooth sweeps

    twist = face_width * tan(beta) / pitch_r

about the axis. `beta` is signed by hand, so the twist is too, and the two
members of a pair carry opposite signs. That is not a convention - it is forced.
The gear is placed on its parallel axis by a pure translation with no flip, so
if both parts were cut the same way round their teeth would cross instead of
meshing.
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
    tooth_space_loop,
)
from .params import SpurSetParams

Point2 = tuple[float, float]
Point3 = tuple[float, float, float]

# ISO standard proportions, as multiples of the NORMAL module.
ADDENDUM_FACTOR = 1.00
DEDENDUM_FACTOR = 1.25
WHOLE_DEPTH_FACTOR = ADDENDUM_FACTOR + DEDENDUM_FACTOR

# How far past each end of the face width the loft sections are pushed.
#
# Same reason as the bevel builder's: a cut that finishes tangent to a real face
# of the blank is rejected as zero-thickness geometry, and both ends of a spur
# blank are real faces. Unlike the bevel case this needs no search - the faces
# are planes perpendicular to the axis, so any positive overshoot clears them.
END_OVERSHOOT_FRACTION = 0.05
END_OVERSHOOT_MIN_MM = 0.5


# ---------------------------------------------------------------------------
# Derived geometry
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class SpurMemberGeometry:
    """Derived geometry for one member (pinion or gear) of the set."""

    name: str
    z: int
    beta: float                 # signed helix angle, radians; this member's hand
    pitch_r: float
    base_r: float
    tip_r: float
    root_r: float
    addendum: float
    dedendum: float
    virtual_teeth: float        # z / cos(beta)**3, the equivalent spur gear
    twist: float                # total rotation over the face width, radians
    psi0: float                 # angular half-thickness constant at the base
    half_pitch: float           # pi / z

    @property
    def hand(self) -> str:
        if abs(self.beta) < 1e-12:
            return "none"
        return "right" if self.beta > 0.0 else "left"

    @property
    def helix_angle_deg(self) -> float:
        return math.degrees(self.beta)

    @property
    def twist_deg(self) -> float:
        return math.degrees(self.twist)

    @property
    def angular_pitch(self) -> float:
        return 2.0 * math.pi / self.z

    @property
    def outside_dia(self) -> float:
        return 2.0 * self.tip_r


@dataclass(frozen=True)
class SpurSetGeometry:
    params: SpurSetParams
    centre_distance: float
    transverse_module: float
    transverse_pressure_angle: float
    circular_pitch: float           # transverse, at the pitch circle
    axial_pitch: float              # inf for straight teeth
    whole_depth: float
    transverse_contact_ratio: float
    axial_contact_ratio: float
    pinion: SpurMemberGeometry
    gear: SpurMemberGeometry

    @property
    def total_contact_ratio(self) -> float:
        return self.transverse_contact_ratio + self.axial_contact_ratio

    def member(self, which: str) -> SpurMemberGeometry:
        if which == "pinion":
            return self.pinion
        if which == "gear":
            return self.gear
        raise ValueError(f"member must be 'pinion' or 'gear', got {which!r}")


def compute_set(p: SpurSetParams) -> SpurSetGeometry:
    """Everything derived from the inputs, for both members at once."""
    m_n = p.module
    m_t = p.transverse_module
    alpha_t = p.alpha_t

    addendum = ADDENDUM_FACTOR * m_n
    dedendum = DEDENDUM_FACTOR * m_n

    # Circular tooth thickness at the pitch circle, measured in the transverse
    # plane. Backlash is taken off the tooth, which is the convention that keeps
    # the centre distance nominal.
    tooth_thickness = math.pi * m_t / 2.0 - p.backlash / 2.0

    members = []
    for name, z, beta in (
        ("pinion", p.z1, p.beta),
        # Opposite hand - see the module docstring. Nothing else differs.
        ("gear", p.z2, -p.beta),
    ):
        pitch_r = m_t * z / 2.0
        base_r = pitch_r * math.cos(alpha_t)
        tip_r = pitch_r + addendum
        root_r = pitch_r - dedendum

        # psi0 is the angular half-thickness of the tooth extrapolated back to
        # the base circle; half_pitch is half the angular pitch. Together they
        # are all the flank generator needs, and both are scale-free.
        psi0 = tooth_thickness / (2.0 * pitch_r) + inv(alpha_t)
        half_pitch = math.pi / z

        members.append(
            SpurMemberGeometry(
                name=name,
                z=z,
                beta=beta,
                pitch_r=pitch_r,
                base_r=base_r,
                tip_r=tip_r,
                root_r=root_r,
                addendum=addendum,
                dedendum=dedendum,
                virtual_teeth=z / math.cos(beta) ** 3,
                twist=p.face_width * math.tan(beta) / pitch_r,
                psi0=psi0,
                half_pitch=half_pitch,
            )
        )

    pinion, gear = members
    centre_distance = p.centre_distance
    sin_beta = abs(math.sin(p.beta))

    return SpurSetGeometry(
        params=p,
        centre_distance=centre_distance,
        transverse_module=m_t,
        transverse_pressure_angle=alpha_t,
        circular_pitch=math.pi * m_t,
        axial_pitch=(math.pi * m_n / sin_beta) if sin_beta > 1e-12 else math.inf,
        whole_depth=WHOLE_DEPTH_FACTOR * m_n,
        transverse_contact_ratio=transverse_contact_ratio(
            pinion, gear, centre_distance, alpha_t, math.pi * m_t
        ),
        axial_contact_ratio=(
            p.face_width * sin_beta / (math.pi * m_n) if sin_beta > 1e-12 else 0.0
        ),
        pinion=pinion,
        gear=gear,
    )


def transverse_contact_ratio(
    pinion: SpurMemberGeometry,
    gear: SpurMemberGeometry,
    centre_distance: float,
    alpha_t: float,
    circular_pitch: float,
) -> float:
    """How many tooth pairs are in contact on average, in the transverse plane.

    The standard length-of-action over base pitch. The length of action is the
    part of the line of action lying between the two tip circles:

        g = sqrt(ra1^2 - rb1^2) + sqrt(ra2^2 - rb2^2) - a * sin(alpha_t)

    and the base pitch it is divided by is `p_t * cos(alpha_t)`. Below 1.0 the
    pair loses contact between teeth and cannot transmit continuous motion,
    which is why the validator treats that as an error rather than a warning.

    The `max(0, ...)` guards a tip circle that has fallen inside its own base
    circle - possible with a badly undercut pinion - where the square root is of
    a negative number and the geometry is meaningless anyway.
    """
    def branch(m: SpurMemberGeometry) -> float:
        return math.sqrt(max(0.0, m.tip_r ** 2 - m.base_r ** 2))

    length_of_action = branch(pinion) + branch(gear) - centre_distance * math.sin(
        alpha_t
    )
    base_pitch = circular_pitch * math.cos(alpha_t)
    return max(0.0, length_of_action / base_pitch)


def undercut_limit(alpha_t: float, beta: float) -> float:
    """Fewest teeth a standard rack cutter can cut without undercutting.

    `2 * cos(beta) / sin(alpha_t)^2` - 17.1 at 20 degrees and straight, which is
    why a 17-tooth pinion sits right on the line. A helix raises the transverse
    pressure angle, so a helical gear can carry fewer teeth before undercutting
    than a straight one of the same normal pressure angle.

    Only reported, never designed around: with no profile shift in the parameter
    set there is nothing the geometry can do about it, and the honest answer is
    to say so. The generated root below the base circle is a radial line, not the
    trochoid a real cutter leaves, so the model does not show the undercut even
    when it happens.
    """
    return 2.0 * math.cos(beta) / math.sin(alpha_t) ** 2


# ---------------------------------------------------------------------------
# Tooth space profile, in the transverse plane
# ---------------------------------------------------------------------------


def end_overshoot(geo: SpurSetGeometry) -> float:
    """How far past each end face the cut profile is pushed, mm."""
    return max(END_OVERSHOOT_MIN_MM, END_OVERSHOOT_FRACTION * geo.params.face_width)


@dataclass(frozen=True)
class ToothSpaceSection:
    """One transverse section of a single tooth space.

    `segments` keeps the boundary broken into named pieces so the SOLIDWORKS
    layer can build splines and arcs separately rather than forcing one spline
    through sharp corners. `loop_2d` is the same boundary flattened into a
    closed polyline, which is what the preview and DXF export want.

    `z` is the axial position and `phase` the rotation the twist has accumulated
    by it - zero at z = 0 by definition, since that is the clocking reference.
    """

    member: str
    z: float
    phase: float
    r_root: float
    r_tip: float
    r_cap: float
    filleted: bool
    segments: dict[str, list[Point2]] = field(default_factory=dict)
    loop_2d: list[Point2] = field(default_factory=list)

    def loop_3d(self) -> list[Point3]:
        return [to_axial_3d(x, y, self.phase, self.z) for x, y in self.loop_2d]

    def segments_3d(self) -> dict[str, list[Point3]]:
        return {
            name: [to_axial_3d(x, y, self.phase, self.z) for x, y in pts]
            for name, pts in self.segments.items()
        }


def to_axial_3d(x: float, y: float, phase: float, z: float) -> Point3:
    """Lift a transverse profile point to its place on the real gear.

    Rotate by the phase the twist has reached at this z, then set the height.
    For straight teeth the phase is zero and this is a plain lift.
    """
    c, s = math.cos(phase), math.sin(phase)
    return x * c - y * s, x * s + y * c, z


def phase_at(geo: SpurSetGeometry, member: str, z: float) -> float:
    """How far the section at height `z` has twisted, radians.

    Linear in z, which is exactly what a helix is. Measured from z = 0, so it is
    negative below the front face and beyond the total twist above the back one -
    both of which happen, because the cut profile is pushed past each end.
    """
    b = geo.params.face_width
    if b <= 0.0:
        return 0.0
    return geo.member(member).twist * z / b


def tooth_space_section(
    geo: SpurSetGeometry,
    member: str,
    z: float = 0.0,
    n_flank: int = FLANK_POINTS,
    split_cap: bool = False,
) -> ToothSpaceSection:
    """Build one transverse section of a tooth space, centred on angle 0 at z=0.

    The loop runs counter-clockwise: up the negative flank, out past the tip,
    across the cap, back down the positive flank, then round the root.

    Every section of a spur gear is the same shape - only its phase differs -
    because nothing about the profile depends on z. That is the whole difference
    from the bevel case, where each section is a different size and the radii
    have to be rebuilt for every one.
    """
    p = geo.params
    m = geo.member(member)

    # Clamp the tip so the tooth never goes pointed. Rarely binds for a spur
    # gear at standard proportions - it takes a very small tooth count - but it
    # costs nothing and it is the same guard the bevel side relies on.
    r_tip = min(
        m.tip_r,
        max_tip_radius(m.base_r, m.psi0, MIN_TOP_LAND_FACTOR * p.module),
    )
    r_cap = r_tip + CUT_OVERSHOOT_FACTOR * p.module

    segments, loop, filleted = tooth_space_loop(
        m.base_r, m.root_r, r_tip, r_cap, m.psi0, m.half_pitch,
        p.fillet_factor * p.module, n_flank, split_cap=split_cap,
    )

    return ToothSpaceSection(
        member=member,
        z=z,
        phase=phase_at(geo, member, z),
        r_root=m.root_r,
        r_tip=r_tip,
        r_cap=r_cap,
        filleted=filleted,
        segments=segments,
        loop_2d=loop,
    )


def guide_helix(
    geo: SpurSetGeometry,
    member: str,
    z_lo: float,
    z_hi: float,
    points: int = 61,
) -> list[Point3]:
    """The loft guide curve: the helix traced by the cap's centreline vertex.

    A guided loft needs its guide to touch a point that *exists* in every
    profile. The cap of the tooth-space section is symmetric about the space
    centreline, so the point at (r_cap, angle 0) in section coordinates traces a
    pure helix of radius r_cap as the section twists - and `split_cap` puts a
    real vertex there rather than leaving it somewhere along a spline.

    Sampled rather than handed to `InsertHelix`, for the same reason the tooth
    sections are: the builder already knows how to write a 3D sketch through
    points, and a sampled curve needs no agreement with SOLIDWORKS about pitch,
    start angle or hand.
    """
    section = tooth_space_section(geo, member, z=0.0)
    r_cap = section.r_cap
    return [
        to_axial_3d(r_cap, 0.0, phase_at(geo, member, z), z)
        for z in (
            z_lo + (z_hi - z_lo) * i / (points - 1) for i in range(points)
        )
    ]


# ---------------------------------------------------------------------------
# Blank
# ---------------------------------------------------------------------------


def blank_outline(geo: SpurSetGeometry, member: str) -> list[Point2]:
    """Meridian half-section of the gear blank, as (R, z) with the front face at 0.

    Revolving this about the Z axis gives the un-toothed body. The profile runs:
    bore at the front face, out across the front face to the tip radius, back
    along the outside to the back face, in across the back face, then home along
    the bore - with an optional hub boss standing behind the back face.

    A plain stepped cylinder, and deliberately so. The bevel blank ends on the
    back cone and needs a root rim under the teeth at the heel; a spur blank ends
    on planes perpendicular to the axis, both faces carry the full tooth depth
    uniformly, and there is no wedge of vanishing material to protect.
    """
    p = geo.params
    m = geo.member(member)

    r_bore = p.bore / 2.0
    r_tip = m.tip_r
    b = p.face_width
    r_hub = max(r_bore, min(m.root_r, r_bore + 2.0 * p.module))

    outline: list[Point2] = [
        (r_bore, 0.0),
        (r_tip, 0.0),
        (r_tip, b),
    ]

    if p.hub_thickness > 0.0 and r_hub > r_bore + 1e-9:
        outline += [
            (r_hub, b),
            (r_hub, b + p.hub_thickness),
            (r_bore, b + p.hub_thickness),
        ]
    else:
        outline.append((r_bore, b))

    return outline
