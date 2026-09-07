"""Pure Python Gleason/ISO-style hypoid geometry.

The macro solver follows the Method 1 pitch-cone relationships: the two
operating cones have distinct axes, the pinion and wheel spiral angles differ
by the hypoid offset angle, and the common-normal offset is solved at the mean
contact point.  Tooth sections use the repository's involute/Tredgold core;
the local cone frames are later placed on skew axes by :mod:`hypoid.mesh`.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from .. import involute
from ..bevel.geometry import CrownTrace, to_cone_3d
from ..involute import tooth_space_loop
from .params import HypoidSetParams

Point2 = tuple[float, float]
Point3 = tuple[float, float, float]

WORKING_DEPTH_FACTOR = 2.0
CLEARANCE_FACTOR = 0.125
MAX_ITERATIONS = 80
SOLVER_TOLERANCE = 2e-10


@dataclass(frozen=True)
class HypoidMemberGeometry:
    name: str
    z: int
    pitch_angle: float
    pitch_radius: float
    cone_distance: float
    outer_cone_distance: float
    mean_spiral_angle: float
    addendum: float
    dedendum: float
    face_angle: float
    root_angle: float
    virtual_teeth: float
    virtual_pitch_r: float
    virtual_base_r: float
    virtual_tip_r: float
    virtual_root_r: float
    normal_tooth_thickness: float
    transverse_tooth_thickness: float
    tip_r: float
    root_r: float
    outside_dia: float
    outer_root_radius: float

    @property
    def pitch_angle_deg(self) -> float:
        return math.degrees(self.pitch_angle)

    @property
    def mean_spiral_angle_deg(self) -> float:
        return math.degrees(self.mean_spiral_angle)

    @property
    def angular_pitch(self) -> float:
        return 2.0 * math.pi / self.z


@dataclass(frozen=True)
class HypoidSection:
    member: str
    cone_dist: float
    phase: float
    pitch_angle: float
    cone_apex_z: float
    r_root: float
    r_tip: float
    r_cap: float
    segments: dict[str, list[Point2]]
    loop_2d: list[Point2]

    def loop_3d(self) -> list[Point3]:
        return [
            to_cone_3d(x, y, self.pitch_angle, self.cone_apex_z, self.phase)
            for x, y in self.loop_2d
        ]

    def segments_3d(self) -> dict[str, list[Point3]]:
        return {
            key: [
                to_cone_3d(x, y, self.pitch_angle, self.cone_apex_z, self.phase)
                for x, y in points
            ]
            for key, points in self.segments.items()
        }


@dataclass(frozen=True)
class HypoidSetGeometry:
    params: HypoidSetParams
    outer_cone_dist: float
    mean_cone_dist: float
    inner_cone_dist: float
    mean_normal_module: float
    offset_angle: float
    pitch_plane_offset: float
    pinion: HypoidMemberGeometry
    gear: HypoidMemberGeometry

    @property
    def offset(self) -> float:
        return self.params.offset

    @property
    def shaft_angle_deg(self) -> float:
        return self.params.shaft_angle

    @property
    def offset_angle_deg(self) -> float:
        return math.degrees(self.offset_angle)

    @property
    def face_contact_ratio(self) -> float:
        pitch = math.pi * self.mean_normal_module
        return abs(self.params.face_width * math.tan(self.pinion.mean_spiral_angle) / pitch)

    @property
    def circular_pitch(self) -> float:
        return math.pi * self.mean_normal_module

    @property
    def working_depth(self) -> float:
        return WORKING_DEPTH_FACTOR * self.mean_normal_module

    @property
    def clearance(self) -> float:
        return CLEARANCE_FACTOR * self.mean_normal_module

    @property
    def whole_depth(self) -> float:
        return self.working_depth + self.clearance

    def member(self, which: str) -> HypoidMemberGeometry:
        if which == "pinion":
            return self.pinion
        if which == "gear":
            return self.gear
        raise ValueError(f"unknown member {which!r}; expected 'pinion' or 'gear'")


def contact_azimuths(geo: HypoidSetGeometry) -> tuple[float, float]:
    """Local radial angles of the common mean pitch-surface normal."""
    p = geo.params
    d1, d2, sigma = (
        geo.pinion.pitch_angle,
        geo.gear.pitch_angle,
        p.sigma,
    )
    sin_sigma = math.sin(sigma)
    if abs(sin_sigma) < 1e-12:
        raise ValueError("hypoid contact azimuth is singular at zero shaft angle")

    nz = -math.sin(d1)
    nx = (math.sin(d2) + math.sin(d1) * math.cos(sigma)) / sin_sigma
    ny2 = 1.0 - nx * nx - nz * nz
    if ny2 < -1e-10:
        raise ValueError("hypoid pitch-surface normals do not share a contact direction")
    ny = math.copysign(math.sqrt(max(0.0, ny2)), p.offset or 1.0)

    theta1 = math.atan2(ny / math.cos(d1), nx / math.cos(d1))
    axis2 = (math.sin(sigma), 0.0, math.cos(sigma))
    radial_x2 = (math.cos(sigma), 0.0, -math.sin(sigma))
    normal = (nx, ny, nz)
    radial2 = tuple(
        (-normal[i] + math.sin(d2) * axis2[i]) / math.cos(d2)
        for i in range(3)
    )
    theta2 = math.atan2(
        radial2[1], sum(radial2[i] * radial_x2[i] for i in range(3))
    )
    return theta1, theta2


def _solve_linear3(matrix, vector):
    """Small Gaussian solver used instead of bringing in a numeric dependency."""
    a = [list(row) + [value] for row, value in zip(matrix, vector)]
    for i in range(3):
        pivot = max(range(i, 3), key=lambda row: abs(a[row][i]))
        if abs(a[pivot][i]) < 1e-14:
            raise ValueError("singular hypoid pitch-cone system")
        a[i], a[pivot] = a[pivot], a[i]
        scale = a[i][i]
        for j in range(i, 4):
            a[i][j] /= scale
        for row in range(3):
            if row == i:
                continue
            scale = a[row][i]
            for j in range(i, 4):
                a[row][j] -= scale * a[i][j]
    return [a[i][3] for i in range(3)]


def _pitch_solution(p: HypoidSetParams) -> tuple[float, float, float, float, float]:
    """Solve the three Method 1 macro relations.

    The public offset is the shaft-to-shaft common-normal distance.  Method 1
    reports the pitch-plane offset, which is slightly larger; the conversion is
    solved together with the two pitch angles and the pinion offset angle.
    """
    sigma = p.sigma
    if abs(p.offset) < 1e-12:
        d1 = math.atan2(math.sin(sigma), p.ratio + math.cos(sigma))
        d2 = sigma - d1
        r2 = (p.wheel_outer_radius / math.sin(d2) - p.face_width / 2.0) * math.sin(d2)
        return d1, d2, 0.0, r2 / p.ratio, r2

    sign = 1.0 if p.offset > 0.0 else -1.0
    target = abs(p.offset)
    beta1 = abs(p.psi1)
    # The initial estimate is the ordinary hypoid offset angle.  The wheel
    # pitch radius is evaluated at the middle of the face width.
    d0 = math.atan2(math.sin(sigma), p.ratio + math.cos(sigma))
    d2 = max(0.2, sigma - d0)
    eps = math.asin(min(0.95, target / max(p.wheel_outer_radius, target + 1e-9)))
    d1 = d0
    x = [d1, d2, eps]

    def residual(v):
        d1, d2, eps = v
        if not (0.05 < d1 < math.pi - 0.05 and 0.05 < d2 < math.pi - 0.05):
            return (1e3, 1e3, 1e3)
        re2 = p.wheel_outer_radius / math.sin(d2)
        rm2 = re2 - p.face_width / 2.0
        r2 = rm2 * math.sin(d2)
        beta2 = beta1 - eps
        r1 = r2 * math.cos(beta2) / (p.ratio * max(math.cos(beta1), 1e-9))
        # Skew-cone relation, common-normal offset, and pitch-plane offset.
        f1 = math.cos(d1) * math.cos(d2) * math.cos(eps) - math.sin(d1) * math.sin(d2) - math.cos(sigma)
        f2 = (r1 * math.cos(d2) + r2 * math.cos(d1)) * math.sin(eps) / max(math.sin(sigma), 1e-9) - target
        f3 = rm2 * math.sin(eps) - target * (1.0 + 0.005 * abs(math.sin(sigma)))
        return f1, f2, f3

    for _ in range(MAX_ITERATIONS):
        f = residual(x)
        if max(abs(v) for v in f) < SOLVER_TOLERANCE:
            break
        h = 1e-6
        columns = []
        for j in range(3):
            trial = list(x)
            trial[j] += h
            g = residual(trial)
            columns.append([(g[i] - f[i]) / h for i in range(3)])
        jacobian = [[columns[col][row] for col in range(3)] for row in range(3)]
        try:
            step = _solve_linear3(jacobian, [-v for v in f])
        except ValueError as exc:
            raise ValueError("hypoid pitch-cone iteration became singular") from exc
        length = math.sqrt(sum(v * v for v in step))
        if length > 0.15:
            step = [v * 0.15 / length for v in step]
        x = [x[i] + step[i] for i in range(3)]
    else:
        raise ValueError("hypoid Method 1 pitch-cone iteration did not converge")

    d1, d2, eps = x
    if not (0.0 < d1 < math.pi / 2 and 0.0 < d2 < math.pi / 2):
        raise ValueError("hypoid pitch angles are outside the external-pair range")
    re2 = p.wheel_outer_radius / math.sin(d2)
    rm2 = re2 - p.face_width / 2.0
    r2 = rm2 * math.sin(d2)
    beta2 = beta1 - eps
    r1 = r2 * math.cos(beta2) / (p.ratio * max(math.cos(beta1), 1e-9))
    return d1, d2, sign * eps, r1, r2


def _member(name, z, delta, radius, cone_distance, spiral, p, tooth_module, mate=False):
    # The 0.54/0.46 split is the Gleason long/short addendum convention.  The
    # published Method 1 profile-shift factor nudges the split; depth and
    # clearance remain explicit advanced inputs rather than magic constants.
    split = (0.54 if not mate else 0.46) + 0.02 * p.profile_shift
    addendum = split * tooth_module
    dedendum = p.depth_factor * tooth_module + p.clearance_factor * tooth_module - addendum
    virtual_pitch = radius / max(math.cos(delta), 1e-9)
    base = virtual_pitch * math.cos(p.alpha)
    tip = virtual_pitch + addendum
    root = max(base * 1.001, virtual_pitch - dedendum)
    # Method 1's thickness factor redistributes one normal circular pitch
    # between the members; it must not be added to both.  Backlash is likewise
    # split here so it is applied once across the pair.  The Tredgold profile is
    # transverse to each member, hence the final 1/cos(beta) conversion.
    thickness_share = 0.5 * (1.0 - p.thickness_factor if mate else 1.0 + p.thickness_factor)
    normal_thickness = math.pi * tooth_module * thickness_share - p.backlash / 2.0
    transverse_thickness = normal_thickness / max(abs(math.cos(spiral)), 1e-9)
    half_pitch = math.pi / (z / max(math.cos(delta), 1e-9))
    # Outer diameter is represented in the local cone section.  The published
    # macro dimensions, rather than this presentation value, remain authoritative.
    tip_r = tip * math.cos(delta)
    root_r = root * math.cos(delta)
    outer_scale = (cone_distance + p.face_width / 2.0) / cone_distance
    outer_tip_r = tip_r * outer_scale
    outer_root_r = root_r * outer_scale
    return HypoidMemberGeometry(
        name=name, z=z, pitch_angle=delta, pitch_radius=radius,
        cone_distance=cone_distance, outer_cone_distance=cone_distance + p.face_width / 2.0,
        mean_spiral_angle=spiral, addendum=addendum, dedendum=dedendum,
        face_angle=delta + math.atan2(addendum, max(cone_distance, 1e-9)),
        root_angle=max(0.01, delta - math.atan2(dedendum, max(cone_distance, 1e-9))),
        virtual_teeth=z / max(math.cos(delta), 1e-9), virtual_pitch_r=virtual_pitch,
        virtual_base_r=base, virtual_tip_r=tip, virtual_root_r=root,
        normal_tooth_thickness=normal_thickness,
        transverse_tooth_thickness=transverse_thickness,
        tip_r=tip_r, root_r=root_r, outside_dia=2.0 * outer_tip_r,
        outer_root_radius=outer_root_r,
    )


def compute_set(p: HypoidSetParams) -> HypoidSetGeometry:
    d1, d2, eps, r1, r2 = _pitch_solution(p)
    beta1 = p.psi1
    # Offset changes the two spiral-angle magnitudes; it does not change which
    # hand was requested.  Subtracting signed epsilon directly made a left-hand
    # or negative-offset wheel jump to the wrong side of 50 degrees.
    beta2 = math.copysign(max(1e-9, abs(beta1) - abs(eps)), beta1)
    R1 = r1 / max(math.sin(d1), 1e-9)
    R2 = r2 / max(math.sin(d2), 1e-9)
    inner1 = max(1e-6, R1 - p.face_width / 2.0)
    inner2 = max(1e-6, R2 - p.face_width / 2.0)
    m_n = 2.0 * r2 * math.cos(beta2) / p.z2
    pinion = _member("pinion", p.z1, d1, r1, R1, beta1, p, m_n, False)
    gear = _member("gear", p.z2, d2, r2, R2, beta2, p, m_n, True)
    return HypoidSetGeometry(
        params=p, outer_cone_dist=max(R1, R2) + p.face_width / 2.0,
        mean_cone_dist=0.5 * (R1 + R2), inner_cone_dist=min(inner1, inner2),
        mean_normal_module=m_n, offset_angle=eps,
        pitch_plane_offset=abs(R2 * math.sin(eps)), pinion=pinion, gear=gear,
    )


def _gear_section_phase_tangent(geo: HypoidSetGeometry) -> float:
    """Wheel phase tangent that shares the pinion trace at mean contact.

    ``mean_spiral_angle`` remains the Method 1 manufacturing dimension.  The
    Tredgold/conical tooth approximation needs a slightly different wheel
    section phase: resolve the pinion trace tangent into the wheel's generator
    and circumferential directions in the assembled skew-axis frame.  This
    removes the first-order trace mismatch that otherwise makes the lofts cross
    through one another despite agreeing at the mean pitch point.
    """
    a, b, p = geo.pinion, geo.gear, geo.params
    theta1, theta2 = contact_azimuths(geo)

    def generator(delta: float, theta: float) -> tuple[float, float, float]:
        return (
            math.sin(delta) * math.cos(theta),
            math.sin(delta) * math.sin(theta),
            math.cos(delta),
        )

    def tangent(theta: float) -> tuple[float, float, float]:
        return -math.sin(theta), math.cos(theta), 0.0

    def wheel_frame(vector) -> tuple[float, float, float]:
        c, s = math.cos(p.sigma), math.sin(p.sigma)
        x, y, z = vector
        return c * x + s * z, y, -s * x + c * z

    g1, e1 = generator(a.pitch_angle, theta1), tangent(theta1)
    pinion_trace = tuple(
        g1[i] + math.sin(a.pitch_angle) * math.tan(a.mean_spiral_angle) * e1[i]
        for i in range(3)
    )
    wheel_generator = wheel_frame(generator(b.pitch_angle, theta2))
    # The wheel phase has the opposite sign, hence -e_theta is the second
    # basis direction used here.
    wheel_tangent = tuple(-value for value in wheel_frame(tangent(theta2)))

    aa = sum(value * value for value in wheel_generator)
    bb = sum(value * value for value in wheel_tangent)
    ab = sum(wheel_generator[i] * wheel_tangent[i] for i in range(3))
    av = sum(wheel_generator[i] * pinion_trace[i] for i in range(3))
    bv = sum(wheel_tangent[i] * pinion_trace[i] for i in range(3))
    determinant = aa * bb - ab * ab
    if abs(determinant) < 1e-12:
        raise ValueError("hypoid contact trace is singular")
    generator_scale = (av * bb - bv * ab) / determinant
    tangent_scale = (bv * aa - av * ab) / determinant
    if abs(generator_scale) < 1e-12 or abs(math.sin(b.pitch_angle)) < 1e-12:
        raise ValueError("hypoid wheel contact trace has no finite section phase")
    return tangent_scale / generator_scale / math.sin(b.pitch_angle)


def _phase_tangent(member: HypoidMemberGeometry, geo: HypoidSetGeometry) -> float:
    return (
        _gear_section_phase_tangent(geo)
        if member.name == "gear"
        else math.tan(member.mean_spiral_angle)
    )


def _cutter_trace(
    member: HypoidMemberGeometry, geo: HypoidSetGeometry
) -> CrownTrace | None:
    """Return the local circular cutter trace used for a member's phase.

    The hypoid contact calculation supplies the first-order phase tangent at
    the mean cone distance.  A finite cutter radius supplies the curvature
    away from that point, just as it does for a bevel pair.  The wheel uses
    the tangent resolved by _gear_section_phase_tangent so the two traces
    still agree at mean contact.
    """
    radius = geo.params.cutter_radius
    if radius is None:
        return None
    if radius <= 0.0:
        raise ValueError("cutter radius must be greater than zero")
    return CrownTrace.for_set(
        math.atan(abs(_phase_tangent(member, geo))),
        radius,
        member.cone_distance,
    )


def _phase(member: HypoidMemberGeometry, cone_dist: float, geo: HypoidSetGeometry) -> float:
    # The phase is zero at the mean cone distance and rolls in opposite senses
    # on the pair.  With a finite cutter, use the circular arc's exact change
    # in trace angle rather than its tangent at the mean.
    span = cone_dist - member.cone_distance
    mate_sign = -1.0 if member.name == "gear" else 1.0
    phase_tangent = _phase_tangent(member, geo)
    trace = _cutter_trace(member, geo)
    if trace is None:
        phase_curve = span * phase_tangent / max(cone_dist, 1e-9)
    else:
        if abs(geo.params.spiral_angle) > 1e-12:
            trace_sign = math.copysign(1.0, phase_tangent)
        else:
            # At zero mean spiral the tangent has no sign.  The hand still
            # chooses which side of the circular cutter arc the Zerol trace
            # bends toward.
            trace_sign = 1.0 if geo.params.hand == "right" else -1.0
        phase_curve = -trace_sign * trace.theta_at(cone_dist)
    return mate_sign * phase_curve


def tooth_space_section(geo: HypoidSetGeometry, member: str, cone_dist: float | None = None,
                        split_cap: bool = False) -> HypoidSection:
    m = geo.member(member)
    if cone_dist is None:
        cone_dist = m.cone_distance
    scale = cone_dist / max(m.cone_distance, 1e-9)
    base = m.virtual_base_r * scale
    root = m.virtual_root_r * scale
    tip = m.virtual_tip_r * scale
    cap = tip + involute.CUT_OVERSHOOT_FACTOR * geo.params.module
    psi0 = (
        m.transverse_tooth_thickness / max(2.0 * m.virtual_pitch_r, 1e-9)
        + involute.inv(geo.params.alpha)
    )
    half = math.pi / m.virtual_teeth
    segments, loop, _ = tooth_space_loop(
        base, root, tip, cap, psi0, half,
        max(0.0, geo.params.min_root_thickness * 0.4),
        split_cap=split_cap,
    )
    phase = _phase(m, cone_dist, geo)
    return HypoidSection(
        member=member, cone_dist=cone_dist, phase=phase,
        # ``to_cone_3d`` maps a Tredgold back-cone section.  Its local apex is
        # A/cos(delta), not the pitch-cone apex at zero; using zero translated
        # every tooth section completely away from the revolved blank, so a
        # SOLIDWORKS loft had no material to cut.
        pitch_angle=m.pitch_angle,
        cone_apex_z=cone_dist / max(math.cos(m.pitch_angle), 1e-9),
        r_root=root, r_tip=tip, r_cap=cap, segments=segments, loop_2d=loop,
    )


def section_cone_distances(geo: HypoidSetGeometry, member: str, count: int = 8) -> list[float]:
    m = geo.member(member)
    p = geo.params
    outline = blank_outline(geo, member)
    z_front = outline[0][1]
    i_back = 4 if p.min_root_thickness > 0.0 else 3
    z_back = outline[i_back][1]
    margin = max(0.1, 0.1 * p.module)
    step = max(p.module, p.face_width / 8.0)

    # A cut loft is most reliable when both terminal profiles are wholly in
    # free space.  The nominal mean +/- half-face positions are still inside
    # the sloping face/back cones, which made InsertCutBlend reject the closed
    # volume.  Walk each end just far enough to clear the actual blank planes.
    lo = max(0.05 * m.cone_distance, m.cone_distance - p.face_width / 2.0)
    for _ in range(100):
        inner = tooth_space_section(geo, member, lo)
        if max(z for _, _, z in inner.loop_3d()) < z_front - margin:
            break
        lo = max(0.05 * m.cone_distance, lo - step)
    else:
        raise ValueError(f"could not clear the {member} hypoid blank front face")

    hi = m.cone_distance + p.face_width / 2.0
    for _ in range(100):
        outer = tooth_space_section(geo, member, hi)
        if min(z for _, _, z in outer.loop_3d()) > z_back + margin:
            break
        hi += step
    else:
        raise ValueError(f"could not clear the {member} hypoid blank back face")

    return [lo + (hi - lo) * i / max(count - 1, 1) for i in range(max(2, count))]


def blank_outline(geo: HypoidSetGeometry, member: str) -> list[tuple[float, float]]:
    m = geo.member(member)
    p = geo.params
    bore = p.bore / 2.0 if member == "pinion" else max(0.5, p.bore / 2.0)
    outer = m.cone_distance + p.face_width / 2.0
    inner = max(0.05 * m.cone_distance, m.cone_distance - p.face_width / 2.0)
    cos_d = math.cos(m.pitch_angle)
    sin_d = math.sin(m.pitch_angle)

    def mapped(radius: float, cone_dist: float) -> tuple[float, float]:
        """Physical R/z of a developed section radius on its back cone."""
        developed = radius * cone_dist / m.cone_distance
        return developed * cos_d, cone_dist / cos_d - developed * sin_d

    inner_tip, z_front = mapped(m.virtual_tip_r, inner)
    outer_tip, z_crown = mapped(m.virtual_tip_r, outer)
    outer_root, z_root = mapped(m.virtual_root_r, outer)
    z_back = z_root + max(0.0, p.min_root_thickness)
    outline = [(bore, z_front), (inner_tip, z_front),
               (outer_tip, z_crown), (outer_root, z_root)]
    if p.min_root_thickness > 0.0:
        outline.append((outer_root, z_back))
    if p.hub_thickness > 0.0:
        hub_r = min(outer_root * 0.7, bore + 2.0 * max(p.module, 1.0))
        outline.extend([(hub_r, z_back), (hub_r, z_back + p.hub_thickness),
                        (bore, z_back + p.hub_thickness)])
    else:
        outline.append((bore, z_back))
    return outline


def section_count(geo: HypoidSetGeometry, member: str) -> int:
    m = geo.member(member)
    sag = max(0.01, 0.02 * geo.params.module)
    inner = max(0.05 * m.cone_distance, m.cone_distance - geo.params.face_width / 2.0)
    outer = m.cone_distance + geo.params.face_width / 2.0
    twist = abs(_phase(m, outer, geo) - _phase(m, inner, geo))
    step = 2.0 * math.acos(max(-1.0, min(1.0, 1.0 - sag / max(m.tip_r, 1e-9))))
    return max(2, math.ceil(twist / max(step, 1e-9)) + 1)
