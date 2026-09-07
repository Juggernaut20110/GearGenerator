"""Pure Python Gleason/ISO-style hypoid geometry.

The macro solver follows the Method 1 pitch-cone relationships: the two
operating cones have distinct axes, the pinion and wheel spiral angles differ
by the hypoid offset angle, and the common-normal offset is solved at the mean
contact point.  Tooth sections use the repository's involute/Tredgold core;
the local cone frames are later placed on skew axes by :mod:`hypoid.mesh`.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, replace

from .. import involute
from ..bevel.geometry import CrownTrace, to_cone_3d
from ..involute import tooth_space_loop
from .params import HypoidSetParams

Point2 = tuple[float, float]
Point3 = tuple[float, float, float]

MAX_ITERATIONS = 80
# ISO 23509 Method 1 formula 11 uses this factor only for the preliminary
# wheel-angle estimate; the final result is set by the curvature closure.
METHOD1_INITIAL_FACTOR = 1.2
METHOD1_CURVATURE_TOLERANCE = 1e-9


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
    working_depth: float
    clearance: float
    whole_depth: float
    addendum_angle: float
    dedendum_angle: float
    face_angle: float
    root_angle: float
    face_width: float
    outer_face_width: float
    inner_face_width: float
    inner_cone_distance: float
    pitch_apex_z: float
    mean_pitch_z: float
    face_apex_z: float
    root_apex_z: float
    inner_tip_z: float
    outer_tip_z: float
    inner_root_z: float
    outer_root_z: float
    outer_pitch_diameter: float
    inner_pitch_diameter: float
    outer_tip_diameter: float
    inner_tip_diameter: float
    outer_addendum: float
    inner_addendum: float
    outer_dedendum: float
    inner_dedendum: float
    outer_whole_depth: float
    inner_whole_depth: float
    mean_tip_radius: float
    mean_root_radius: float
    inner_tip_radius: float
    outer_tip_radius: float
    inner_root_radius: float
    outer_root_diameter: float
    inner_root_diameter: float
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
    tredgold_outer_root_radius: float

    @property
    def pitch_angle_deg(self) -> float:
        return math.degrees(self.pitch_angle)

    @property
    def mean_spiral_angle_deg(self) -> float:
        return math.degrees(self.mean_spiral_angle)

    @property
    def face_angle_deg(self) -> float:
        return math.degrees(self.face_angle)

    @property
    def root_angle_deg(self) -> float:
        return math.degrees(self.root_angle)

    @property
    def angular_pitch(self) -> float:
        return 2.0 * math.pi / self.z


@dataclass(frozen=True)
class HypoidMethod1Geometry:
    """Published Method 1 pitch-cone iteration results.

    The ISO equations use positive magnitudes for the offset geometry.  The
    public result restores the sign of the requested offset to the offset
    angles and pitch-plane offset, while the pitch angles and curvature
    quantities remain invariant under an offset sign reversal.
    """

    gear_ratio: float
    desired_pinion_spiral_angle: float
    shaft_angle_departure: float
    approximate_wheel_pitch_angle: float
    approximate_wheel_mean_radius: float
    approximate_pinion_offset_angle: float
    approximate_dimension_factor: float
    approximate_pinion_mean_radius: float
    wheel_offset_angle_axial: float
    intermediate_pinion_offset_angle_axial: float
    intermediate_pinion_pitch_angle: float
    intermediate_pinion_offset_angle_pitch: float
    intermediate_pinion_spiral_angle: float
    dimension_factor_increment: float
    pinion_mean_radius_increment: float
    pinion_offset_angle_axial: float
    pinion_offset_angle_pitch: float
    pinion_spiral_angle: float
    wheel_spiral_angle: float
    pinion_pitch_angle: float
    wheel_pitch_angle: float
    pinion_mean_radius: float
    wheel_mean_radius: float
    pinion_mean_cone_distance: float
    wheel_mean_cone_distance: float
    pitch_plane_offset: float
    limit_pressure_angle: float
    limit_radius_of_curvature: float | None
    mean_tooth_curvature: float | None
    curvature_residual: float | None
    iterations: int
    # Method 1 blank closure values.  They live with the pitch solution so
    # callers can audit the complete calculation without reconstructing the
    # hidden intermediate geometry used by compute_set().
    wheel_face_width_factor: float | None = None
    wheel_outer_face_width: float | None = None
    wheel_inner_face_width: float | None = None
    pinion_outer_face_width: float | None = None
    pinion_inner_face_width: float | None = None
    crossing_to_wheel_mean_z: float | None = None
    crossing_to_pinion_mean_z: float | None = None
    wheel_pitch_apex_z: float | None = None
    pinion_pitch_apex_z: float | None = None
    wheel_face_apex_z: float | None = None
    wheel_root_apex_z: float | None = None
    pinion_face_apex_z: float | None = None
    pinion_root_apex_z: float | None = None
    pinion_root_plane_offset_angle: float | None = None
    pinion_face_plane_offset_angle: float | None = None
    pinion_face_width_auxiliary_angle: float | None = None


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
    basic_addendum_factor: float
    basic_dedendum_factor: float
    profile_shift_coefficient: float
    mean_working_depth: float
    mean_clearance: float
    mean_whole_depth: float
    offset_angle: float
    pitch_plane_offset: float
    method1: HypoidMethod1Geometry
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
        return self.mean_working_depth

    @property
    def clearance(self) -> float:
        return self.mean_clearance

    @property
    def whole_depth(self) -> float:
        return self.mean_whole_depth

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


def _checked_asin(value: float, label: str) -> float:
    """Return asin(value), rejecting geometrically impossible values.

    The solver never clamps an out-of-range design into a result.
    """
    if not math.isfinite(value) or abs(value) > 1.0:
        raise ValueError(f"hypoid Method 1 {label} is outside asin domain: {value!r}")
    return math.asin(value)


def _checked_external_angle(numerator: float, denominator: float, label: str) -> float:
    angle = math.atan2(numerator, denominator)
    if not 0.0 < angle < math.pi / 2.0:
        raise ValueError(
            f"hypoid Method 1 {label} is outside the external-pair range: "
            f"{math.degrees(angle):.6g} degrees"
        )
    return angle


def _checked_tangent_angle(tangent: float, label: str) -> float:
    if not math.isfinite(tangent):
        raise ValueError(f"hypoid Method 1 {label} is not finite")
    return _checked_external_angle(tangent, 1.0, label)


@dataclass(frozen=True)
class _Method1Trial:
    wheel_offset_angle_axial: float
    intermediate_pinion_offset_angle_axial: float
    intermediate_pinion_pitch_angle: float
    intermediate_pinion_offset_angle_pitch: float
    intermediate_pinion_spiral_angle: float
    dimension_factor_increment: float
    pinion_mean_radius_increment: float
    pinion_offset_angle_axial: float
    pinion_offset_angle_pitch: float
    pinion_spiral_angle: float
    wheel_spiral_angle: float
    pinion_pitch_angle: float
    wheel_pitch_angle: float
    pinion_mean_radius: float
    wheel_mean_radius: float
    pinion_mean_cone_distance: float
    wheel_mean_cone_distance: float
    limit_pressure_angle: float
    limit_radius_of_curvature: float


def _method1_trial(
    p: HypoidSetParams,
    *,
    offset: float,
    delta_sigma: float,
    desired_beta: float,
    approximate_dimension_factor: float,
    approximate_pinion_radius: float,
    wheel_mean_radius: float,
    eta: float,
) -> _Method1Trial:
    """Evaluate ISO 23509 Method 1 formulas 16 through 33 at ``eta``."""
    ratio = p.ratio

    intermediate_offset = _checked_asin(
        (offset - approximate_pinion_radius * math.sin(eta)) / wheel_mean_radius,
        "intermediate pinion axial offset angle",
    )
    intermediate_pitch = _checked_tangent_angle(
        math.sin(eta)
        / (math.tan(intermediate_offset) * math.cos(delta_sigma))
        + math.tan(delta_sigma) * math.cos(eta),
        "intermediate pinion pitch angle",
    )
    intermediate_pitch_offset = _checked_asin(
        math.sin(intermediate_offset) * math.cos(delta_sigma)
        / math.cos(intermediate_pitch),
        "intermediate pinion pitch-plane offset angle",
    )
    intermediate_beta = math.atan2(
        approximate_dimension_factor - math.cos(intermediate_pitch_offset),
        math.sin(intermediate_pitch_offset),
    )
    dimension_increment = math.sin(intermediate_pitch_offset) * (
        math.tan(desired_beta) - math.tan(intermediate_beta)
    )
    radius_increment = wheel_mean_radius * dimension_increment / ratio
    pinion_offset = _checked_asin(
        math.sin(intermediate_offset)
        - radius_increment / wheel_mean_radius * math.sin(eta),
        "pinion axial offset angle",
    )
    pinion_pitch = _checked_tangent_angle(
        math.sin(eta)
        / (math.tan(pinion_offset) * math.cos(delta_sigma))
        + math.tan(delta_sigma) * math.cos(eta),
        "pinion pitch angle",
    )
    pinion_pitch_offset = _checked_asin(
        math.sin(pinion_offset) * math.cos(delta_sigma)
        / math.cos(pinion_pitch),
        "pinion pitch-plane offset angle",
    )
    pinion_beta = math.atan2(
        approximate_dimension_factor + dimension_increment
        - math.cos(pinion_pitch_offset),
        math.sin(pinion_pitch_offset),
    )
    wheel_beta = pinion_beta - pinion_pitch_offset
    wheel_pitch = _checked_tangent_angle(
        math.sin(pinion_offset)
        / (math.tan(eta) * math.cos(delta_sigma))
        + math.cos(pinion_offset) * math.tan(delta_sigma),
        "wheel pitch angle",
    )
    pinion_radius = approximate_pinion_radius + radius_increment
    if pinion_radius <= 0.0:
        raise ValueError("hypoid Method 1 pinion mean pitch radius is not positive")
    pinion_cone = pinion_radius / math.sin(pinion_pitch)
    wheel_cone = wheel_mean_radius / math.sin(wheel_pitch)
    limit_pressure = math.atan(
        -math.tan(pinion_pitch) * math.tan(wheel_pitch)
        * (
            pinion_cone * math.sin(pinion_beta)
            - wheel_cone * math.sin(wheel_beta)
        )
        / (
            math.cos(pinion_pitch_offset)
            * (
                pinion_cone * math.tan(pinion_pitch)
                + wheel_cone * math.tan(wheel_pitch)
            )
        )
    )
    denominator = (
        -math.tan(limit_pressure)
        * (
            math.tan(pinion_beta) / (pinion_cone * math.tan(pinion_pitch))
            + math.tan(wheel_beta) / (wheel_cone * math.tan(wheel_pitch))
        )
        + 1.0 / (pinion_cone * math.cos(pinion_beta))
        - 1.0 / (wheel_cone * math.cos(wheel_beta))
    )
    if denominator == 0.0:
        raise ValueError("hypoid Method 1 curvature equation is singular")
    limit_radius = (
        (1.0 / math.cos(limit_pressure))
        * (math.tan(pinion_beta) - math.tan(wheel_beta))
        / denominator
    )
    if not math.isfinite(limit_radius) or limit_radius <= 0.0:
        raise ValueError("hypoid Method 1 limit radius of curvature is invalid")
    return _Method1Trial(
        wheel_offset_angle_axial=eta,
        intermediate_pinion_offset_angle_axial=intermediate_offset,
        intermediate_pinion_pitch_angle=intermediate_pitch,
        intermediate_pinion_offset_angle_pitch=intermediate_pitch_offset,
        intermediate_pinion_spiral_angle=intermediate_beta,
        dimension_factor_increment=dimension_increment,
        pinion_mean_radius_increment=radius_increment,
        pinion_offset_angle_axial=pinion_offset,
        pinion_offset_angle_pitch=pinion_pitch_offset,
        pinion_spiral_angle=pinion_beta,
        wheel_spiral_angle=wheel_beta,
        pinion_pitch_angle=pinion_pitch,
        wheel_pitch_angle=wheel_pitch,
        pinion_mean_radius=pinion_radius,
        wheel_mean_radius=wheel_mean_radius,
        pinion_mean_cone_distance=pinion_cone,
        wheel_mean_cone_distance=wheel_cone,
        limit_pressure_angle=limit_pressure,
        limit_radius_of_curvature=limit_radius,
    )


def _pitch_solution(p: HypoidSetParams) -> HypoidMethod1Geometry:
    """Solve the ISO 23509 Method 1 pitch-cone and curvature closure.

    For a non-zero offset this is the Method 1 face-milling construction:
    the preliminary pitch design is iterated through the axial offset and
    spiral-angle equations until the limit lengthwise curvature equals the
    cutter radius.  No empirical pitch-plane correction is applied.
    """
    sigma = p.sigma
    if not 0.0 < sigma < math.pi:
        raise ValueError("shaft angle must be between 0 and 180 degrees")
    ratio = p.ratio
    sign = 1.0 if p.offset >= 0.0 else -1.0
    desired_beta = abs(p.psi1)
    delta_sigma = sigma - math.pi / 2.0

    if abs(p.offset) < 1e-12:
        d1 = math.atan2(math.sin(sigma), ratio + math.cos(sigma))
        d2 = sigma - d1
        if not 0.0 < d1 < math.pi / 2.0 or not 0.0 < d2 < math.pi / 2.0:
            raise ValueError("zero-offset pitch cones are outside the external-pair range")
        R2 = p.wheel_outer_radius / math.sin(d2) - p.face_width / 2.0
        if R2 <= 0.0:
            raise ValueError("zero-offset mean cone distance is not positive")
        r2 = R2 * math.sin(d2)
        r1 = r2 / ratio
        beta = sign * desired_beta if p.psi1 >= 0.0 else -desired_beta
        return HypoidMethod1Geometry(
            gear_ratio=ratio,
            desired_pinion_spiral_angle=p.psi1,
            shaft_angle_departure=delta_sigma,
            approximate_wheel_pitch_angle=d2,
            approximate_wheel_mean_radius=r2,
            approximate_pinion_offset_angle=0.0,
            approximate_dimension_factor=1.0,
            approximate_pinion_mean_radius=r1,
            wheel_offset_angle_axial=0.0,
            intermediate_pinion_offset_angle_axial=0.0,
            intermediate_pinion_pitch_angle=d1,
            intermediate_pinion_offset_angle_pitch=0.0,
            intermediate_pinion_spiral_angle=abs(beta),
            dimension_factor_increment=0.0,
            pinion_mean_radius_increment=0.0,
            pinion_offset_angle_axial=0.0,
            pinion_offset_angle_pitch=0.0,
            pinion_spiral_angle=abs(beta),
            wheel_spiral_angle=abs(beta),
            pinion_pitch_angle=d1,
            wheel_pitch_angle=d2,
            pinion_mean_radius=r1,
            wheel_mean_radius=r2,
            pinion_mean_cone_distance=R2,
            wheel_mean_cone_distance=R2,
            pitch_plane_offset=0.0,
            limit_pressure_angle=0.0,
            limit_radius_of_curvature=None,
            mean_tooth_curvature=None,
            curvature_residual=None,
            iterations=0,
        )

    if p.cutter_radius is None:
        raise ValueError("non-zero-offset Method 1 geometry requires cutter_radius")
    if p.cutter_radius <= 0.0:
        raise ValueError("cutter radius must be greater than zero")

    offset = abs(p.offset)
    approximate_wheel_angle = _checked_external_angle(
        ratio * math.cos(delta_sigma),
        METHOD1_INITIAL_FACTOR * (1.0 - ratio * math.sin(delta_sigma)),
        "approximate wheel pitch angle",
    )
    approximate_wheel_radius = (
        p.wheel_outer_diameter
        - p.face_width * math.sin(approximate_wheel_angle)
    ) / 2.0
    if approximate_wheel_radius <= 0.0:
        raise ValueError("approximate wheel mean pitch radius is not positive")
    approximate_offset = _checked_asin(
        offset * math.sin(approximate_wheel_angle) / approximate_wheel_radius,
        "approximate pitch-plane offset angle",
    )
    approximate_dimension = (
        math.tan(desired_beta) * math.sin(approximate_offset)
        + math.cos(approximate_offset)
    )
    approximate_pinion_radius = approximate_wheel_radius * approximate_dimension / ratio
    if approximate_pinion_radius <= 0.0:
        raise ValueError("approximate pinion mean pitch radius is not positive")
    eta = _checked_external_angle(
        offset,
        approximate_wheel_radius
        * (
            math.tan(approximate_wheel_angle) * math.cos(delta_sigma)
            - math.sin(delta_sigma)
        )
        + approximate_pinion_radius,
        "initial wheel axial offset angle",
    )

    trial = None
    for iteration in range(1, MAX_ITERATIONS + 1):
        trial = _method1_trial(
            p,
            offset=offset,
            delta_sigma=delta_sigma,
            desired_beta=desired_beta,
            approximate_dimension_factor=approximate_dimension,
            approximate_pinion_radius=approximate_pinion_radius,
            wheel_mean_radius=approximate_wheel_radius,
            eta=eta,
        )
        residual = trial.limit_radius_of_curvature - p.cutter_radius
        if abs(residual) <= METHOD1_CURVATURE_TOLERANCE:
            break
        h = 1e-6
        try:
            trial_h = _method1_trial(
                p,
                offset=offset,
                delta_sigma=delta_sigma,
                desired_beta=desired_beta,
                approximate_dimension_factor=approximate_dimension,
                approximate_pinion_radius=approximate_pinion_radius,
                wheel_mean_radius=approximate_wheel_radius,
                eta=eta + h,
            )
        except ValueError as exc:
            raise ValueError(
                "hypoid Method 1 curvature iteration left the valid geometry domain"
            ) from exc
        derivative = (
            trial_h.limit_radius_of_curvature
            - trial.limit_radius_of_curvature
        ) / h
        if not math.isfinite(derivative) or abs(derivative) < 1e-12:
            raise ValueError("hypoid Method 1 curvature iteration became singular")
        next_eta = eta - residual / derivative
        if not 0.0 < next_eta < math.pi / 2.0:
            raise ValueError("hypoid Method 1 curvature iteration did not converge")
        eta = next_eta
    else:
        raise ValueError("hypoid Method 1 curvature iteration did not converge")

    assert trial is not None
    if abs(trial.limit_radius_of_curvature - p.cutter_radius) > METHOD1_CURVATURE_TOLERANCE:
        raise ValueError("hypoid Method 1 curvature closure did not converge")
    signed = lambda value: sign * value
    return HypoidMethod1Geometry(
        gear_ratio=ratio,
        desired_pinion_spiral_angle=p.psi1,
        shaft_angle_departure=delta_sigma,
        approximate_wheel_pitch_angle=approximate_wheel_angle,
        approximate_wheel_mean_radius=approximate_wheel_radius,
        approximate_pinion_offset_angle=signed(approximate_offset),
        approximate_dimension_factor=approximate_dimension,
        approximate_pinion_mean_radius=approximate_pinion_radius,
        wheel_offset_angle_axial=signed(trial.wheel_offset_angle_axial),
        intermediate_pinion_offset_angle_axial=signed(
            trial.intermediate_pinion_offset_angle_axial
        ),
        intermediate_pinion_pitch_angle=trial.intermediate_pinion_pitch_angle,
        intermediate_pinion_offset_angle_pitch=signed(
            trial.intermediate_pinion_offset_angle_pitch
        ),
        intermediate_pinion_spiral_angle=trial.intermediate_pinion_spiral_angle,
        dimension_factor_increment=trial.dimension_factor_increment,
        pinion_mean_radius_increment=trial.pinion_mean_radius_increment,
        pinion_offset_angle_axial=signed(trial.pinion_offset_angle_axial),
        pinion_offset_angle_pitch=signed(trial.pinion_offset_angle_pitch),
        pinion_spiral_angle=trial.pinion_spiral_angle,
        wheel_spiral_angle=trial.wheel_spiral_angle,
        pinion_pitch_angle=trial.pinion_pitch_angle,
        wheel_pitch_angle=trial.wheel_pitch_angle,
        pinion_mean_radius=trial.pinion_mean_radius,
        wheel_mean_radius=trial.wheel_mean_radius,
        pinion_mean_cone_distance=trial.pinion_mean_cone_distance,
        wheel_mean_cone_distance=trial.wheel_mean_cone_distance,
        pitch_plane_offset=signed(
            trial.wheel_mean_cone_distance
            * math.sin(trial.pinion_offset_angle_pitch)
        ),
        limit_pressure_angle=trial.limit_pressure_angle,
        limit_radius_of_curvature=trial.limit_radius_of_curvature,
        mean_tooth_curvature=p.cutter_radius,
        curvature_residual=trial.limit_radius_of_curvature - p.cutter_radius,
        iterations=iteration,
    )


@dataclass(frozen=True)
class _Method1Depth:
    """Method 1 tooth-depth factors and dimensions at the calculation point."""

    basic_addendum_factor: float
    basic_dedendum_factor: float
    profile_shift_coefficient: float
    working_depth: float
    clearance: float
    whole_depth: float
    pinion_addendum: float
    pinion_dedendum: float
    gear_addendum: float
    gear_dedendum: float


def _method1_depth(p: HypoidSetParams, mean_normal_module: float) -> _Method1Depth:
    """Convert the type-II tooth data and apply ISO 23509 formulas 132-139."""
    if p.depth_factor <= 0.0:
        raise ValueError("Method 1 depth factor must be greater than zero")
    if p.clearance_factor < 0.0:
        raise ValueError("Method 1 clearance factor cannot be negative")
    if not 0.0 < p.gear_mean_addendum_factor < 1.0:
        raise ValueError("Method 1 gear mean addendum factor must be between zero and one")

    # ISO 23509 Table 4: type-II (kd, kc, c_ham) to type-I (k_hap, k_hfp,
    # x_hm1).  The ``2*kc`` term is important: kc is half the type-I
    # clearance convention used by the Method 1 reference.
    k_hap = 0.5 * p.depth_factor
    k_hfp = 0.5 * p.depth_factor + 2.0 * p.clearance_factor
    x_hm1 = p.depth_factor * (0.5 - p.gear_mean_addendum_factor)
    working_depth = 2.0 * k_hap * mean_normal_module
    clearance = (k_hfp - k_hap) * mean_normal_module
    whole_depth = (k_hap + k_hfp) * mean_normal_module
    pinion_addendum = mean_normal_module * (k_hap + x_hm1)
    pinion_dedendum = mean_normal_module * (k_hfp - x_hm1)
    gear_addendum = mean_normal_module * (k_hap - x_hm1)
    gear_dedendum = mean_normal_module * (k_hfp + x_hm1)
    values = (
        working_depth, clearance, whole_depth, pinion_addendum,
        pinion_dedendum, gear_addendum, gear_dedendum,
    )
    if any(not math.isfinite(value) or value <= 0.0 for value in values):
        raise ValueError("Method 1 tooth-depth inputs produce non-positive dimensions")
    return _Method1Depth(
        basic_addendum_factor=k_hap,
        basic_dedendum_factor=k_hfp,
        profile_shift_coefficient=x_hm1,
        working_depth=working_depth,
        clearance=clearance,
        whole_depth=whole_depth,
        pinion_addendum=pinion_addendum,
        pinion_dedendum=pinion_dedendum,
        gear_addendum=gear_addendum,
        gear_dedendum=gear_dedendum,
    )


def _member(
    name,
    z,
    delta,
    radius,
    cone_distance,
    spiral,
    p,
    tooth_module,
    *,
    addendum,
    dedendum,
    working_depth,
    clearance,
    whole_depth,
    addendum_angle,
    dedendum_angle,
    inner_cone_distance,
    outer_face_width,
    inner_face_width,
    pitch_apex_z,
    mean_pitch_z,
    face_apex_z,
    root_apex_z,
):
    """Build one member from the ISO Method 1 blank dimensions.

    ``tip_r`` and ``root_r`` deliberately remain the Tredgold back-cone
    section radii used by the involute approximation.  The physical Method 1
    blank radii are the explicit ``*_tip_radius`` and ``*_root_radius``
    fields below; they are not forced to agree with the developed section.
    """
    cos_delta = math.cos(delta)
    sin_delta = math.sin(delta)
    if not 0.0 < delta < math.pi / 2.0 or abs(cos_delta) < 1e-12:
        raise ValueError(f"{name} Method 1 pitch angle is outside the external-pair range")
    if cone_distance <= 0.0 or inner_cone_distance <= 0.0:
        raise ValueError(f"{name} Method 1 cone distance is not positive")
    if outer_face_width <= 0.0 or inner_face_width <= 0.0:
        raise ValueError(f"{name} Method 1 face boundary is not positive")
    outer_cone_distance = cone_distance + outer_face_width
    if not math.isfinite(addendum_angle) or not math.isfinite(dedendum_angle):
        raise ValueError(f"{name} Method 1 tooth angles are not finite")
    face_angle = delta + addendum_angle
    root_angle = delta - dedendum_angle
    if not 0.0 < face_angle < math.pi / 2.0:
        raise ValueError(f"{name} Method 1 face angle is outside the external-pair range")
    if not 0.0 < root_angle < math.pi / 2.0:
        raise ValueError(f"{name} Method 1 root angle is outside the external-pair range")

    outer_pitch_diameter = 2.0 * outer_cone_distance * sin_delta
    inner_pitch_diameter = 2.0 * inner_cone_distance * sin_delta
    outer_addendum = addendum + outer_face_width * math.tan(addendum_angle)
    inner_addendum = addendum - inner_face_width * math.tan(addendum_angle)
    outer_dedendum = dedendum + outer_face_width * math.tan(dedendum_angle)
    inner_dedendum = dedendum - inner_face_width * math.tan(dedendum_angle)
    depths = (outer_addendum, inner_addendum, outer_dedendum, inner_dedendum)
    if any(not math.isfinite(value) or value <= 0.0 for value in depths):
        raise ValueError(f"{name} Method 1 face boundaries produce non-positive tooth depth")

    mean_tip_radius = radius + addendum * cos_delta
    mean_root_radius = radius - dedendum * cos_delta
    inner_tip_radius = inner_pitch_diameter / 2.0 + inner_addendum * cos_delta
    outer_tip_radius = outer_pitch_diameter / 2.0 + outer_addendum * cos_delta
    inner_root_radius = inner_pitch_diameter / 2.0 - inner_dedendum * cos_delta
    outer_root_radius = outer_pitch_diameter / 2.0 - outer_dedendum * cos_delta
    radii = (
        mean_tip_radius, mean_root_radius, inner_tip_radius, outer_tip_radius,
        inner_root_radius, outer_root_radius,
    )
    if any(not math.isfinite(value) or value <= 0.0 for value in radii):
        raise ValueError(f"{name} Method 1 blank radius is not positive")

    # The axial positions below are the ISO Method 1 meridian layout.  The
    # face and root apexes are retained separately from the Tredgold section
    # apex, because the latter is only an involute-section approximation.
    inner_tip_z = (
        mean_pitch_z - inner_face_width * cos_delta
        - inner_addendum * sin_delta
    )
    outer_tip_z = (
        mean_pitch_z + outer_face_width * cos_delta
        - outer_addendum * sin_delta
    )
    inner_root_z = (
        mean_pitch_z - inner_face_width * cos_delta
        + inner_dedendum * sin_delta
    )
    outer_root_z = (
        mean_pitch_z + outer_face_width * cos_delta
        + outer_dedendum * sin_delta
    )

    virtual_pitch = radius / cos_delta
    virtual_base = virtual_pitch * math.cos(p.alpha)
    virtual_tip = virtual_pitch + addendum
    virtual_root = virtual_pitch - dedendum
    if virtual_root <= 0.0:
        raise ValueError(f"{name} Method 1 Tredgold root radius is not positive")
    tip_r = virtual_tip * cos_delta
    root_r = virtual_root * cos_delta

    # Method 1's thickness factor redistributes one normal circular pitch
    # between the members; backlash is split once across the pair.  This is
    # still the repository's approximate Tredgold tooth-section thickness,
    # not a generated drive/coast flank calculation.
    thickness_share = 0.5 * (
        1.0 - p.thickness_factor if name == "gear" else 1.0 + p.thickness_factor
    )
    normal_thickness = math.pi * tooth_module * thickness_share - p.backlash / 2.0
    if normal_thickness <= 0.0:
        raise ValueError(f"{name} tooth thickness is not positive")
    spiral_cos = abs(math.cos(spiral))
    if spiral_cos < 1e-12:
        raise ValueError(f"{name} transverse tooth thickness is singular")
    transverse_thickness = normal_thickness / spiral_cos

    return HypoidMemberGeometry(
        name=name,
        z=z,
        pitch_angle=delta,
        pitch_radius=radius,
        cone_distance=cone_distance,
        outer_cone_distance=outer_cone_distance,
        mean_spiral_angle=spiral,
        addendum=addendum,
        dedendum=dedendum,
        working_depth=working_depth,
        clearance=clearance,
        whole_depth=whole_depth,
        addendum_angle=addendum_angle,
        dedendum_angle=dedendum_angle,
        face_angle=face_angle,
        root_angle=root_angle,
        face_width=outer_face_width + inner_face_width,
        outer_face_width=outer_face_width,
        inner_face_width=inner_face_width,
        inner_cone_distance=inner_cone_distance,
        pitch_apex_z=pitch_apex_z,
        mean_pitch_z=mean_pitch_z,
        face_apex_z=face_apex_z,
        root_apex_z=root_apex_z,
        inner_tip_z=inner_tip_z,
        outer_tip_z=outer_tip_z,
        inner_root_z=inner_root_z,
        outer_root_z=outer_root_z,
        outer_pitch_diameter=outer_pitch_diameter,
        inner_pitch_diameter=inner_pitch_diameter,
        outer_tip_diameter=2.0 * outer_tip_radius,
        inner_tip_diameter=2.0 * inner_tip_radius,
        outer_addendum=outer_addendum,
        inner_addendum=inner_addendum,
        outer_dedendum=outer_dedendum,
        inner_dedendum=inner_dedendum,
        outer_whole_depth=outer_addendum + outer_dedendum,
        inner_whole_depth=inner_addendum + inner_dedendum,
        mean_tip_radius=mean_tip_radius,
        mean_root_radius=mean_root_radius,
        inner_tip_radius=inner_tip_radius,
        outer_tip_radius=outer_tip_radius,
        inner_root_radius=inner_root_radius,
        outer_root_radius=outer_root_radius,
        outer_root_diameter=2.0 * outer_root_radius,
        inner_root_diameter=2.0 * inner_root_radius,
        virtual_teeth=z / cos_delta,
        virtual_pitch_r=virtual_pitch,
        virtual_base_r=virtual_base,
        virtual_tip_r=virtual_tip,
        virtual_root_r=virtual_root,
        normal_tooth_thickness=normal_thickness,
        transverse_tooth_thickness=transverse_thickness,
        tip_r=tip_r,
        root_r=root_r,
        outside_dia=2.0 * outer_tip_radius,
        tredgold_outer_root_radius=(
            virtual_root * outer_cone_distance / cone_distance * cos_delta
        ),
    )


def compute_set(p: HypoidSetParams) -> HypoidSetGeometry:
    method1 = _pitch_solution(p)
    d1 = method1.pinion_pitch_angle
    d2 = method1.wheel_pitch_angle
    spiral_sign = 1.0 if p.psi1 >= 0.0 else -1.0
    beta1 = spiral_sign * method1.pinion_spiral_angle
    beta2 = spiral_sign * method1.wheel_spiral_angle
    r1 = method1.pinion_mean_radius
    r2 = method1.wheel_mean_radius
    R1 = method1.pinion_mean_cone_distance
    R2 = method1.wheel_mean_cone_distance
    m_n = 2.0 * r2 * math.cos(beta2) / p.z2
    depth = _method1_depth(p, m_n)

    # ISO 23509 formulas 122-128.  The wheel outer transverse diameter is an
    # input, so the Method 1 calculation point is not generally at b2/2.
    sin_d2 = math.sin(d2)
    if sin_d2 <= 0.0:
        raise ValueError("Method 1 wheel pitch angle has no positive sine")
    Re2 = p.wheel_outer_radius / sin_d2
    be2 = Re2 - R2
    bi2 = p.face_width - be2
    if be2 <= 0.0 or bi2 <= 0.0:
        raise ValueError(
            "Method 1 wheel outer diameter and face width do not contain the "
            "wheel calculation point"
        )
    Ri2 = R2 - bi2
    if Ri2 <= 0.0:
        raise ValueError("Method 1 wheel inner cone distance is not positive")
    cbe2 = be2 / p.face_width

    # ISO 23509 formulas 129-131 locate the calculation point and the two
    # pitch-cone apices relative to the crossing point.  The axial offset is
    # used as a magnitude here: reversing the signed hypoid offset mirrors the
    # blank, but cannot change its dimensions.
    delta_sigma = method1.shaft_angle_departure
    zeta_m = abs(method1.pinion_offset_angle_axial)
    dm1 = 2.0 * R1 * math.sin(d1)
    dm2 = 2.0 * R2 * math.sin(d2)
    tzm2 = (
        dm1 * math.sin(d2) / (2.0 * math.cos(d1))
        - 0.5 * math.cos(zeta_m) * math.tan(delta_sigma)
        * (dm2 + dm1 * math.cos(d2) / math.cos(d1))
    )
    tzm1 = dm2 / 2.0 * math.cos(zeta_m) * math.cos(delta_sigma)
    tzm1 -= tzm2 * math.sin(delta_sigma)
    tz1 = R1 * math.cos(d1) - tzm1
    tz2 = R2 * math.cos(d2) - tzm2

    theta_a2 = math.radians(p.gear_addendum_angle)
    theta_f2 = math.radians(p.gear_dedendum_angle)
    delta_a2 = d2 + theta_a2
    delta_f2 = d2 - theta_f2
    if not 0.0 < delta_a2 < math.pi / 2.0:
        raise ValueError("Method 1 wheel face angle is outside the external-pair range")
    if not 0.0 < delta_f2 < math.pi / 2.0:
        raise ValueError("Method 1 wheel root angle is outside the external-pair range")

    offset = abs(p.offset)
    den_root = R2 * math.cos(theta_f2) - tz2 * math.cos(delta_f2)
    den_face = R2 * math.cos(theta_a2) - tz2 * math.cos(delta_a2)
    if abs(den_root) < 1e-12 or abs(den_face) < 1e-12:
        raise ValueError("Method 1 root/face angle calculation is singular")
    phi_r = math.atan2(
        offset * math.tan(delta_sigma) * math.cos(theta_f2), den_root
    )
    phi_o = math.atan2(
        offset * math.tan(delta_sigma) * math.cos(theta_a2), den_face
    )
    zeta_r = _checked_asin(
        offset * math.cos(phi_r) * math.sin(delta_f2) / den_root,
        "pinion root-plane offset angle",
    ) - phi_r
    zeta_o = _checked_asin(
        offset * math.cos(phi_o) * math.sin(delta_a2) / den_face,
        "pinion face-plane offset angle",
    ) - phi_o
    delta_a1 = _checked_asin(
        math.sin(delta_sigma) * math.sin(delta_f2)
        + math.cos(delta_sigma) * math.cos(delta_f2) * math.cos(zeta_r),
        "pinion face angle",
    )
    delta_f1 = _checked_asin(
        math.sin(delta_sigma) * math.sin(delta_a2)
        + math.cos(delta_sigma) * math.cos(delta_a2) * math.cos(zeta_o),
        "pinion root angle",
    )
    theta_a1 = delta_a1 - d1
    theta_f1 = d1 - delta_f1

    # ISO 23509 formulas 150-153.  These are stored as apex locations rather
    # than folded into a single face-angle approximation.
    if abs(math.sin(delta_a1)) < 1e-12 or abs(math.sin(delta_f1)) < 1e-12:
        raise ValueError("Method 1 pinion apex calculation is singular")
    tzF2 = tz2 - (
        R2 * math.sin(theta_a2) - depth.gear_addendum * math.cos(theta_a2)
    ) / math.sin(delta_a2)
    tzR2 = tz2 + (
        R2 * math.sin(theta_f2) - depth.gear_dedendum * math.cos(theta_f2)
    ) / math.sin(delta_f2)
    tzF1 = (
        offset * math.sin(zeta_r) * math.cos(delta_f2)
        - tzR2 * math.sin(delta_f2) - depth.clearance
    ) / math.sin(delta_a1)
    tzR1 = (
        offset * math.sin(zeta_o) * math.cos(delta_a2)
        - tzF2 * math.sin(delta_a2) - depth.clearance
    ) / math.sin(delta_f1)

    # ISO 23509 formulas 159-166.  Method 1's pinion calculation point is
    # generally not in the middle of the pinion face.
    denominator_lambda = (
        p.ratio * math.cos(d1) + math.cos(d2) * math.cos(abs(method1.pinion_offset_angle_pitch))
    )
    lambda_prime = math.atan2(
        math.sin(abs(method1.pinion_offset_angle_pitch)) * math.cos(d2),
        denominator_lambda,
    )
    cos_lambda = math.cos(abs(method1.pinion_offset_angle_pitch) - lambda_prime)
    if abs(cos_lambda) < 1e-12:
        raise ValueError("Method 1 pinion face-width closure is singular")
    breri1 = p.face_width * math.cos(lambda_prime) / cos_lambda
    delta_bx1 = depth.working_depth * math.sin(abs(zeta_r)) * (1.0 - 1.0 / p.ratio)
    cos_theta_a1 = math.cos(theta_a1)
    cos_delta_a1 = math.cos(delta_a1)
    if abs(cos_theta_a1) < 1e-12 or abs(cos_delta_a1) < 1e-12:
        raise ValueError("Method 1 pinion face-width closure is singular")
    delta_gxe = (
        cbe2 * breri1 * cos_delta_a1 / cos_theta_a1
        + delta_bx1
        - (depth.gear_dedendum - depth.clearance) * math.sin(d1)
    )
    delta_gxi = (
        (1.0 - cbe2) * breri1 * cos_delta_a1 / cos_theta_a1
        + delta_bx1
        + (depth.gear_dedendum - depth.clearance) * math.sin(d1)
    )
    be1 = (
        delta_gxe + depth.pinion_addendum * math.sin(d1)
    ) * cos_theta_a1 / cos_delta_a1
    denominator_bi1 = math.cos(d1) - math.tan(theta_a1) * math.sin(d1)
    if abs(denominator_bi1) < 1e-12:
        raise ValueError("Method 1 pinion inner face-width closure is singular")
    bi1 = (
        delta_gxi - depth.pinion_addendum * math.sin(d1)
    ) / denominator_bi1
    if be1 <= 0.0 or bi1 <= 0.0:
        raise ValueError("Method 1 pinion face-width closure is not positive")
    Ri1 = R1 - bi1
    if Ri1 <= 0.0:
        raise ValueError("Method 1 pinion inner cone distance is not positive")

    method1 = replace(
        method1,
        wheel_face_width_factor=cbe2,
        wheel_outer_face_width=be2,
        wheel_inner_face_width=bi2,
        pinion_outer_face_width=be1,
        pinion_inner_face_width=bi1,
        crossing_to_wheel_mean_z=tzm2,
        crossing_to_pinion_mean_z=tzm1,
        wheel_pitch_apex_z=tz2,
        pinion_pitch_apex_z=tz1,
        wheel_face_apex_z=tzF2,
        wheel_root_apex_z=tzR2,
        pinion_face_apex_z=tzF1,
        pinion_root_apex_z=tzR1,
        pinion_root_plane_offset_angle=zeta_r,
        pinion_face_plane_offset_angle=zeta_o,
        pinion_face_width_auxiliary_angle=lambda_prime,
    )

    pinion = _member(
        "pinion", p.z1, d1, r1, R1, beta1, p, m_n,
        addendum=depth.pinion_addendum,
        dedendum=depth.pinion_dedendum,
        working_depth=depth.working_depth,
        clearance=depth.clearance,
        whole_depth=depth.whole_depth,
        addendum_angle=theta_a1,
        dedendum_angle=theta_f1,
        inner_cone_distance=Ri1,
        outer_face_width=be1,
        inner_face_width=bi1,
        pitch_apex_z=tz1,
        mean_pitch_z=tzm1,
        face_apex_z=tzF1,
        root_apex_z=tzR1,
    )
    gear = _member(
        "gear", p.z2, d2, r2, R2, beta2, p, m_n,
        addendum=depth.gear_addendum,
        dedendum=depth.gear_dedendum,
        working_depth=depth.working_depth,
        clearance=depth.clearance,
        whole_depth=depth.whole_depth,
        addendum_angle=theta_a2,
        dedendum_angle=theta_f2,
        inner_cone_distance=Ri2,
        outer_face_width=be2,
        inner_face_width=bi2,
        pitch_apex_z=tz2,
        mean_pitch_z=tzm2,
        face_apex_z=tzF2,
        root_apex_z=tzR2,
    )
    return HypoidSetGeometry(
        params=p,
        outer_cone_dist=gear.outer_cone_distance,
        mean_cone_dist=gear.cone_distance,
        inner_cone_dist=gear.inner_cone_distance,
        mean_normal_module=m_n,
        basic_addendum_factor=depth.basic_addendum_factor,
        basic_dedendum_factor=depth.basic_dedendum_factor,
        profile_shift_coefficient=depth.profile_shift_coefficient,
        mean_working_depth=depth.working_depth,
        mean_clearance=depth.clearance,
        mean_whole_depth=depth.whole_depth,
        offset_angle=method1.pinion_offset_angle_pitch,
        pitch_plane_offset=method1.pitch_plane_offset, method1=method1,
        pinion=pinion, gear=gear,
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
    # free space.  Walk from the actual Method 1 boundaries just far enough to
    # clear the blank planes; the pinion and wheel no longer share these
    # distances.
    lo = m.inner_cone_distance
    for _ in range(100):
        inner = tooth_space_section(geo, member, lo)
        if max(z for _, _, z in inner.loop_3d()) < z_front - margin:
            break
        lo -= step
        if lo <= 0.0:
            raise ValueError(f"could not clear the {member} hypoid blank front face")
    else:
        raise ValueError(f"could not clear the {member} hypoid blank front face")

    hi = m.outer_cone_distance
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
    # These are physical Method 1 points.  The Tredgold developed radii used
    # for the approximate tooth section are intentionally not used to size the
    # revolved blank.
    z_front = m.inner_tip_z
    z_crown = m.outer_tip_z
    z_root = m.outer_root_z
    z_back = z_root + max(0.0, p.min_root_thickness)
    outline = [
        (bore, z_front),
        (m.inner_tip_radius, z_front),
        (m.outer_tip_radius, z_crown),
        (m.outer_root_radius, z_root),
    ]
    if p.min_root_thickness > 0.0:
        outline.append((m.outer_root_radius, z_back))
    if p.hub_thickness > 0.0:
        hub_r = min(m.outer_root_radius * 0.7, bore + 2.0 * max(p.module, 1.0))
        outline.extend([(hub_r, z_back), (hub_r, z_back + p.hub_thickness),
                        (bore, z_back + p.hub_thickness)])
    else:
        outline.append((bore, z_back))
    return outline


def section_count(geo: HypoidSetGeometry, member: str) -> int:
    m = geo.member(member)
    sag = max(0.01, 0.02 * geo.params.module)
    inner = m.inner_cone_distance
    outer = m.outer_cone_distance
    twist = abs(_phase(m, outer, geo) - _phase(m, inner, geo))
    step = 2.0 * math.acos(max(-1.0, min(1.0, 1.0 - sag / max(m.tip_r, 1e-9))))
    return max(2, math.ceil(twist / max(step, 1e-9)) + 1)
