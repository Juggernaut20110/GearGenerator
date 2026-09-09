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
from .params import HypoidSetParams

Point2 = tuple[float, float]
Point3 = tuple[float, float, float]

METHOD1_MAX_ITERATIONS = 80
# ISO 23509 formula 11 uses this factor only for the preliminary wheel-angle
# estimate.  It is not a calibration constant; the final pitch solution is
# determined by the Method 1 curvature closure below.
METHOD1_PRELIMINARY_WHEEL_FACTOR = 1.2
METHOD1_CURVATURE_TOLERANCE_MM = 1e-9
METHOD1_DERIVATIVE_STEP_RAD = 1e-6
# Numerical tolerance for the one-dimensional phase integral used to transport
# the Method 1 pinion trace through a non-zero hypoid offset.  This is an
# integration accuracy, not a geometry/calibration factor.
PHASE_INTEGRATION_TOLERANCE_RAD = 1e-11
PHASE_INTEGRATION_MAX_DEPTH = 20


@dataclass(frozen=True)
class HypoidMemberGeometry:
    """Calculated geometry for one hypoid member.

    All distances and radii are in millimetres.  Angles are stored in radians
    and exposed in degrees by the ``*_deg`` properties.  ``face_width`` is the
    Method 1 calculated member facewidth: the pinion value is ``b_reri1`` and
    the wheel value is the input ``b2``.  ``face_width_along_pitch_cone`` is a
    different quantity for an offset hypoid: it is the physical pitch-cone
    distance span ``b_e + b_i`` between the outer and inner tooth boundaries.
    ``outer_face_width`` and ``inner_face_width`` are the corresponding
    distances from the calculation point to those physical boundaries.

    ``tooth_face_width`` is retained as a compatibility alias for
    ``face_width_along_pitch_cone``.  It must not be confused with the Method
    1 member facewidth on an offset pinion.  The ``tredgold_*`` fields describe
    only the developed back-cone involute approximation used by the current
    tooth-section builder.
    """

    name: str
    z: int
    pitch_angle: float
    pitch_radius: float
    cone_distance: float
    outer_cone_distance: float
    tooth_face_inner_cone_distance: float
    tooth_face_outer_cone_distance: float
    mean_spiral_angle: float
    inner_spiral_angle: float
    outer_spiral_angle: float
    generated_drive_normal_pressure_angle: float
    generated_coast_normal_pressure_angle: float
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
    face_width_along_pitch_cone: float
    # Compatibility alias.  New code should use face_width_along_pitch_cone.
    tooth_face_width: float
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
    outer_root_radius: float
    outer_root_diameter: float
    inner_root_diameter: float
    virtual_teeth: float
    virtual_pitch_r: float
    # Nominal mean-section transverse-equivalent base radius.  The generated
    # drive and coast flanks have independent bases; HypoidSection records those
    # local values explicitly.
    virtual_base_r: float
    virtual_tip_r: float
    virtual_root_r: float
    thickness_modification_coefficient: float
    mean_normal_tooth_thickness: float
    mean_transverse_tooth_thickness: float
    tredgold_tip_radius: float
    tredgold_mean_root_radius: float
    tredgold_outer_root_radius: float

    @property
    def tip_r(self) -> float:
        """Compatibility alias for the Tredgold developed tip radius (mm)."""
        return self.tredgold_tip_radius

    @property
    def root_r(self) -> float:
        """Compatibility alias for the Tredgold developed mean root radius."""
        return self.tredgold_mean_root_radius

    @property
    def outside_dia(self) -> float:
        """Compatibility alias for the physical outer tip diameter (mm)."""
        return self.outer_tip_diameter

    @property
    def physical_pitch_cone_face_width(self) -> float:
        """Compatibility-friendly name for the physical ``b_e + b_i`` span."""
        return self.face_width_along_pitch_cone

    @property
    def pitch_angle_deg(self) -> float:
        return math.degrees(self.pitch_angle)

    @property
    def mean_spiral_angle_deg(self) -> float:
        return math.degrees(self.mean_spiral_angle)

    @property
    def generated_drive_normal_pressure_angle_deg(self) -> float:
        return math.degrees(self.generated_drive_normal_pressure_angle)

    @property
    def generated_coast_normal_pressure_angle_deg(self) -> float:
        return math.degrees(self.generated_coast_normal_pressure_angle)

    @property
    def generated_drive_transverse_pressure_angle(self) -> float:
        """Mean-section transverse equivalent of the generated drive angle."""
        return normal_to_transverse_pressure_angle(
            self.generated_drive_normal_pressure_angle,
            self.mean_spiral_angle,
        )

    @property
    def generated_coast_transverse_pressure_angle(self) -> float:
        """Mean-section transverse equivalent of the generated coast angle."""
        return normal_to_transverse_pressure_angle(
            self.generated_coast_normal_pressure_angle,
            self.mean_spiral_angle,
        )

    @property
    def generated_drive_transverse_pressure_angle_deg(self) -> float:
        return math.degrees(self.generated_drive_transverse_pressure_angle)

    @property
    def generated_coast_transverse_pressure_angle_deg(self) -> float:
        return math.degrees(self.generated_coast_transverse_pressure_angle)

    @property
    def face_angle_deg(self) -> float:
        return math.degrees(self.face_angle)

    @property
    def root_angle_deg(self) -> float:
        return math.degrees(self.root_angle)

    @property
    def angular_pitch(self) -> float:
        return 2.0 * math.pi / self.z

    @property
    def mean_transverse_module(self) -> float:
        """Mean transverse module at this member's Method 1 point (mm)."""
        return 2.0 * self.pitch_radius / self.z

    @property
    def outer_transverse_module(self) -> float:
        """Outer transverse module at this member's outer pitch cone (mm)."""
        return self.outer_pitch_diameter / self.z

    @property
    def normal_tooth_thickness(self) -> float:
        """Compatibility alias for the ISO mean-normal thickness."""
        return self.mean_normal_tooth_thickness

    @property
    def transverse_tooth_thickness(self) -> float:
        """Compatibility alias for the ISO mean-transverse thickness."""
        return self.mean_transverse_tooth_thickness

    @property
    def x_sm(self) -> float:
        """ISO thickness-modification coefficient, including backlash."""
        return self.thickness_modification_coefficient


@dataclass(frozen=True)
class HypoidThicknessGeometry:
    """ISO 23509 tooth-thickness calculation at the mean point.

    ``outer_transverse_backlash`` is the public input convention.  The two
    mean backlash values are derived at the calculation point; they are not
    additional clearances applied to the members.  ``backlash_thickness_...``
    is the one common correction used to obtain x_sm1 and x_sm2.  Lengths are
    millimetres; the pressure angle is radians; thickness modification values
    are dimensionless.
    """

    mean_normal_pressure_angle: float
    theoretical_thickness_modification: float
    backlash_thickness_modification: float
    pinion_thickness_modification: float
    gear_thickness_modification: float
    outer_transverse_backlash: float
    mean_transverse_backlash: float
    mean_normal_backlash: float

    @property
    def mean_normal_pressure_angle_deg(self) -> float:
        return math.degrees(self.mean_normal_pressure_angle)


@dataclass(frozen=True)
class HypoidMethod1Geometry:
    """Published Method 1 pitch-cone iteration results.

    The ISO equations use positive magnitudes for the offset geometry.  The
    public result restores the sign of the requested offset to the offset
    angles and pitch-plane offset, while the pitch angles and curvature
    quantities remain invariant under an offset sign reversal.  Distances and
    curvature radii are millimetres; angles are radians; factors and ratios
    are dimensionless.  ``preliminary_*`` fields are the formula-11 seed
    values only and must not be mistaken for the converged result fields.
    """

    gear_ratio: float
    desired_pinion_spiral_angle: float
    shaft_angle_departure: float
    preliminary_wheel_pitch_angle: float
    preliminary_wheel_mean_radius: float
    preliminary_pinion_offset_angle: float
    preliminary_dimension_factor: float
    preliminary_pinion_mean_radius: float
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
    generated_drive_normal_pressure_angle: float
    generated_coast_normal_pressure_angle: float
    limit_radius_of_curvature: float | None
    mean_tooth_curvature: float | None
    curvature_residual: float | None
    iterations: int
    # Method 1 blank closure values.  They live with the pitch solution so
    # callers can audit the complete calculation without reconstructing the
    # hidden intermediate geometry used by compute_set().  In particular,
    # pinion_face_width is b_reri1 (formula 160), while the physical pinion
    # pitch-cone span is pinion_outer_face_width + pinion_inner_face_width
    # (formula 166).
    wheel_face_width_factor: float | None = None
    wheel_outer_face_width: float | None = None
    wheel_inner_face_width: float | None = None
    pinion_face_width: float | None = None
    pinion_face_width_increment_along_axis: float | None = None
    pinion_outer_face_width: float | None = None
    pinion_inner_face_width: float | None = None
    pinion_boundary_wheel_outer_cone_distance: float | None = None
    pinion_boundary_wheel_inner_cone_distance: float | None = None
    pinion_inner_spiral_angle: float | None = None
    pinion_outer_spiral_angle: float | None = None
    wheel_inner_spiral_angle: float | None = None
    wheel_outer_spiral_angle: float | None = None
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

    @property
    def generated_drive_normal_pressure_angle_deg(self) -> float:
        return math.degrees(self.generated_drive_normal_pressure_angle)

    @property
    def generated_coast_normal_pressure_angle_deg(self) -> float:
        return math.degrees(self.generated_coast_normal_pressure_angle)


@dataclass(frozen=True)
class HypoidSection:
    """One sampled approximate Tredgold tooth-space section.

    ``cone_dist`` and all radii are millimetres.  ``phase``, ``pitch_angle``
    and ``cone_apex_z`` use radians/mm respectively.  The drive and coast
    flanks are independent curves.  ``loop_2d`` is the developed back-cone
    (transverse-equivalent) plane; its local pressure angles and tooth thickness
    are converted from the Method 1 normal values using the local spiral angle.
    This section is not a generated cutter envelope or a fully conjugate hypoid
    surface.
    """

    member: str
    cone_dist: float
    phase: float
    pitch_angle: float
    cone_apex_z: float
    r_root: float
    r_tip: float
    r_cap: float
    spiral_angle: float
    normal_tooth_thickness: float
    transverse_tooth_thickness: float
    drive_transverse_pressure_angle: float
    coast_transverse_pressure_angle: float
    drive_base_radius: float
    coast_base_radius: float
    root_fillet_radius: float
    filleted: bool
    drive_flank: list[Point2]
    coast_flank: list[Point2]
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
class HypoidSectionBounds:
    """Actual Method 1 face limits and separate loft-only extensions.

    ``tooth_face_*`` are physical pitch-cone boundaries.  ``loft_*`` are
    artificial terminal sections used only to let a SOLIDWORKS cut pass clear
    of the revolved blank.  The calculation point is retained explicitly; it
    is not inferred as the midpoint of either interval.  Every distance and
    overshoot is in millimetres.
    """

    calculation_point: float
    tooth_face_inner: float
    tooth_face_outer: float
    loft_inner: float
    loft_outer: float

    @property
    def tooth_face_width(self) -> float:
        return self.tooth_face_outer - self.tooth_face_inner

    @property
    def inner_overshoot(self) -> float:
        return self.tooth_face_inner - self.loft_inner

    @property
    def outer_overshoot(self) -> float:
        return self.loft_outer - self.tooth_face_outer

    def is_loft_extension(self, cone_dist: float, tolerance: float = 1e-9) -> bool:
        return (
            cone_dist < self.tooth_face_inner - tolerance
            or cone_dist > self.tooth_face_outer + tolerance
        )


@dataclass(frozen=True)
class HypoidSetGeometry:
    """Complete calculated hypoid geometry and its approximation metadata.

    The Method 1 pitch, depth, thickness and blank values are stored
    separately from the Tredgold section quantities on each member.  All
    lengths are millimetres, angles are radians unless a property says ``deg``,
    and dimensionless factors are explicitly named as coefficients/factors.
    ``section_cone_bounds`` is the source of the physical face limits and the
    separate SOLIDWORKS-only loft overshoots.
    """

    params: HypoidSetParams
    outer_cone_dist: float
    mean_cone_dist: float
    inner_cone_dist: float
    mean_normal_module: float
    basic_addendum_factor: float
    basic_dedendum_factor: float
    method1_profile_shift_coefficient: float
    mean_working_depth: float
    mean_clearance: float
    mean_whole_depth: float
    offset_angle: float
    pitch_plane_offset: float
    method1: HypoidMethod1Geometry
    thickness: HypoidThicknessGeometry
    pinion: HypoidMemberGeometry
    gear: HypoidMemberGeometry

    @property
    def offset(self) -> float:
        return self.params.offset

    @property
    def profile_shift_coefficient(self) -> float:
        """Compatibility alias for the Method 1 type-I ``x_hm1`` value."""
        return self.method1_profile_shift_coefficient

    @property
    def shaft_angle_deg(self) -> float:
        return self.params.shaft_angle

    @property
    def offset_angle_deg(self) -> float:
        return math.degrees(self.offset_angle)

    @property
    def wheel_outer_transverse_module(self) -> float:
        """The Method 1 input module ``m_et2`` (the wheel outer module)."""
        return self.gear.outer_transverse_module

    @property
    def wheel_mean_transverse_module(self) -> float:
        """Wheel transverse module at its Method 1 mean calculation point."""
        return self.gear.mean_transverse_module

    @property
    def wheel_face_width(self) -> float:
        """Wheel net facewidth ``b2`` / physical pitch-cone face span (mm)."""
        return self.gear.face_width

    @property
    def face_overlap_ratio_estimate(self) -> float:
        """Wheel-side ISO/AGMA-style face-overlap estimate ``epsilon_beta``.

        ISO 23509 Annex B.7 gives this relationship for spiral-bevel design
        selection.  Method 1 is a hypoid calculation, so this is deliberately
        reported as an estimate rather than an operating flank contact ratio.
        The wheel quantities are kept together because the public Method 1
        module and facewidth are wheel quantities::

            epsilon_beta = Re2 * b2 * tan(beta_m2)
                            / (pi * Rm2 * m_et2)

        ``Re2`` and ``Rm2`` are the wheel outer and mean cone distances,
        ``b2`` is the physical wheel pitch-cone span, ``beta_m2`` is the wheel
        mean spiral angle, and ``m_et2`` is the wheel outer transverse module.
        Taking the magnitude makes hand reversal a mirror operation only.
        """
        re2 = self.gear.outer_cone_distance
        rm2 = self.gear.cone_distance
        b2 = self.wheel_face_width
        m_et2 = self.wheel_outer_transverse_module
        if min(re2, rm2, b2, m_et2) <= 0.0:
            return 0.0
        return abs(
            re2 * b2 * math.tan(self.gear.mean_spiral_angle)
            / (math.pi * rm2 * m_et2)
        )

    @property
    def face_contact_ratio(self) -> float:
        """Compatibility alias for :attr:`face_overlap_ratio_estimate`.

        This name is retained for callers of earlier releases.  The Method 1
        reports use the explicit ``face overlap ratio estimate`` label because
        the Tredgold sections do not calculate a true operating contact ratio.
        """
        return self.face_overlap_ratio_estimate

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

    @property
    def outer_transverse_backlash(self) -> float:
        return self.thickness.outer_transverse_backlash

    @property
    def mean_transverse_backlash(self) -> float:
        return self.thickness.mean_transverse_backlash

    @property
    def mean_normal_backlash(self) -> float:
        return self.thickness.mean_normal_backlash

    def member(self, which: str) -> HypoidMemberGeometry:
        if which == "pinion":
            return self.pinion
        if which == "gear":
            return self.gear
        raise ValueError(f"unknown member {which!r}; expected 'pinion' or 'gear'")


def contact_azimuths(geo: HypoidSetGeometry) -> tuple[float, float]:
    """Return radial angles for the common mean pitch-surface normal.

    This solves the *macro* pitch-cone contact geometry, not the longitudinal
    tooth trace.  With ``a1`` and ``a2`` the two shaft axes, and ``e1``/``e2``
    the local radial directions at the returned angles, the oppositely
    oriented cone normals are

        n = cos(delta1) e1 - sin(delta1) a1
          = -cos(delta2) e2 + sin(delta2) a2.

    ``n`` is therefore the common pitch-surface normal.  A tooth trace tangent
    is another tangent-plane direction; its circumferential/generator ratio
    is set by that member's Method 1 spiral angle and is not required to equal
    the other member's trace tangent.  The actual contact line and relative
    sliding direction require the flank surfaces and motion; neither is
    inferred by equating these trace tangents.

    The sign of the selected normal's transverse component follows the signed
    hypoid offset.  This chooses the mirrored member placement while leaving
    the Method 1 dimensions unchanged.
    """
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


def _generated_normal_pressure_angles(
    p: HypoidSetParams, limit_pressure_angle: float
) -> tuple[float, float]:
    """Return the Method 1 generated drive and coast normal angles.

    ISO's generated pressure-angle pair is obtained by applying the signed
    limit pressure angle to the nominal drive/coast design angle.  The anchor
    has a negative limit angle, hence its drive flank is the smaller angle.
    """
    drive = p.alpha + limit_pressure_angle
    coast = p.alpha - limit_pressure_angle
    if not 0.0 < drive < math.pi / 2.0:
        raise ValueError("Method 1 generated drive pressure angle is invalid")
    if not 0.0 < coast < math.pi / 2.0:
        raise ValueError("Method 1 generated coast pressure angle is invalid")
    return drive, coast


def normal_to_transverse_pressure_angle(
    normal_pressure_angle: float, spiral_angle: float
) -> float:
    """Convert a normal pressure angle to its transverse equivalent.

    Method 1 supplies pressure angles in the normal section, perpendicular to
    the local tooth trace.  The involute built in the developed back-cone
    section is transverse-equivalent, so its pressure angle is

        tan(alpha_t) = tan(alpha_n) / abs(cos(beta)).

    ``beta`` is signed by hand, but the conversion depends only on its
    magnitude.  The signed normal angle is retained in the return value so the
    helper remains useful for diagnostic calculations; generated hypoid angles
    are positive magnitudes.
    """
    if not math.isfinite(normal_pressure_angle) or not (
        -math.pi / 2.0 < normal_pressure_angle < math.pi / 2.0
    ):
        raise ValueError("normal pressure angle must lie between -90 and 90 degrees")
    cosine = abs(math.cos(spiral_angle))
    if not math.isfinite(cosine) or cosine < 1e-12:
        raise ValueError("transverse pressure angle is singular at a 90 degree spiral")
    return math.atan2(math.tan(normal_pressure_angle), cosine)


@dataclass(frozen=True)
class _Method1Trial:
    """One Method 1 curvature-closure trial.

    Angles are radians, cone distances and radii are millimetres, and the
    dimension/radius increments are millimetres or dimensionless exactly as
    indicated by their names.
    """

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
    preliminary_dimension_factor: float,
    preliminary_pinion_radius: float,
    wheel_mean_radius: float,
    eta: float,
) -> _Method1Trial:
    """Evaluate ISO 23509 Method 1 formulas 16 through 33 at ``eta``."""
    ratio = p.ratio

    intermediate_offset = _checked_asin(
        (offset - preliminary_pinion_radius * math.sin(eta)) / wheel_mean_radius,
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
        preliminary_dimension_factor - math.cos(intermediate_pitch_offset),
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
        preliminary_dimension_factor + dimension_increment
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
    pinion_radius = preliminary_pinion_radius + radius_increment
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
        generated_drive, generated_coast = _generated_normal_pressure_angles(p, 0.0)
        return HypoidMethod1Geometry(
            gear_ratio=ratio,
            desired_pinion_spiral_angle=p.psi1,
            shaft_angle_departure=delta_sigma,
            preliminary_wheel_pitch_angle=d2,
            preliminary_wheel_mean_radius=r2,
            preliminary_pinion_offset_angle=0.0,
            preliminary_dimension_factor=1.0,
            preliminary_pinion_mean_radius=r1,
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
            generated_drive_normal_pressure_angle=generated_drive,
            generated_coast_normal_pressure_angle=generated_coast,
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
    preliminary_wheel_angle = _checked_external_angle(
        ratio * math.cos(delta_sigma),
        METHOD1_PRELIMINARY_WHEEL_FACTOR
        * (1.0 - ratio * math.sin(delta_sigma)),
        "preliminary wheel pitch angle",
    )
    preliminary_wheel_radius = (
        p.wheel_outer_diameter
        - p.face_width * math.sin(preliminary_wheel_angle)
    ) / 2.0
    if preliminary_wheel_radius <= 0.0:
        raise ValueError("preliminary wheel mean pitch radius is not positive")
    preliminary_offset = _checked_asin(
        offset * math.sin(preliminary_wheel_angle) / preliminary_wheel_radius,
        "preliminary pitch-plane offset angle",
    )
    preliminary_dimension = (
        math.tan(desired_beta) * math.sin(preliminary_offset)
        + math.cos(preliminary_offset)
    )
    preliminary_pinion_radius = (
        preliminary_wheel_radius * preliminary_dimension / ratio
    )
    if preliminary_pinion_radius <= 0.0:
        raise ValueError("preliminary pinion mean pitch radius is not positive")
    eta = _checked_external_angle(
        offset,
        preliminary_wheel_radius
        * (
            math.tan(preliminary_wheel_angle) * math.cos(delta_sigma)
            - math.sin(delta_sigma)
        )
        + preliminary_pinion_radius,
        "initial wheel axial offset angle",
    )

    trial = None
    for iteration in range(1, METHOD1_MAX_ITERATIONS + 1):
        trial = _method1_trial(
            p,
            offset=offset,
            delta_sigma=delta_sigma,
            desired_beta=desired_beta,
            preliminary_dimension_factor=preliminary_dimension,
            preliminary_pinion_radius=preliminary_pinion_radius,
            wheel_mean_radius=preliminary_wheel_radius,
            eta=eta,
        )
        residual = trial.limit_radius_of_curvature - p.cutter_radius
        if abs(residual) <= METHOD1_CURVATURE_TOLERANCE_MM:
            break
        h = METHOD1_DERIVATIVE_STEP_RAD
        try:
            trial_h = _method1_trial(
                p,
                offset=offset,
                delta_sigma=delta_sigma,
                desired_beta=desired_beta,
                preliminary_dimension_factor=preliminary_dimension,
                preliminary_pinion_radius=preliminary_pinion_radius,
                wheel_mean_radius=preliminary_wheel_radius,
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
            raise ValueError(
                "hypoid Method 1 curvature closure became singular: "
                f"dR/deta={derivative:.3e} mm/rad"
            )
        next_eta = eta - residual / derivative
        if not 0.0 < next_eta < math.pi / 2.0:
            raise ValueError(
                "hypoid Method 1 curvature closure left the valid axial-offset "
                f"range: eta_next={math.degrees(next_eta):.6g} deg"
            )
        eta = next_eta
    else:
        raise ValueError(
            "hypoid Method 1 curvature closure did not converge after "
            f"{METHOD1_MAX_ITERATIONS} iterations: residual="
            f"{residual:.3e} mm, target cutter radius={p.cutter_radius:.6g} mm"
        )

    assert trial is not None
    final_residual = trial.limit_radius_of_curvature - p.cutter_radius
    if abs(final_residual) > METHOD1_CURVATURE_TOLERANCE_MM:
        raise ValueError(
            "hypoid Method 1 curvature closure did not meet tolerance: "
            f"residual={final_residual:.3e} mm, tolerance="
            f"{METHOD1_CURVATURE_TOLERANCE_MM:.3e} mm"
        )
    generated_drive, generated_coast = _generated_normal_pressure_angles(
        p, trial.limit_pressure_angle
    )
    signed = lambda value: sign * value
    return HypoidMethod1Geometry(
        gear_ratio=ratio,
        desired_pinion_spiral_angle=p.psi1,
        shaft_angle_departure=delta_sigma,
        preliminary_wheel_pitch_angle=preliminary_wheel_angle,
        preliminary_wheel_mean_radius=preliminary_wheel_radius,
        preliminary_pinion_offset_angle=signed(preliminary_offset),
        preliminary_dimension_factor=preliminary_dimension,
        preliminary_pinion_mean_radius=preliminary_pinion_radius,
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
        generated_drive_normal_pressure_angle=generated_drive,
        generated_coast_normal_pressure_angle=generated_coast,
        limit_radius_of_curvature=trial.limit_radius_of_curvature,
        mean_tooth_curvature=p.cutter_radius,
        curvature_residual=final_residual,
        iterations=iteration,
    )


@dataclass(frozen=True)
class _Method1Depth:
    """Method 1 tooth-depth factors and dimensions at the calculation point.

    Factors are dimensionless; all depth and clearance values are millimetres.
    """

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


def _method1_thickness(
    p: HypoidSetParams,
    mean_normal_module: float,
    wheel_mean_cone_distance: float,
    wheel_outer_cone_distance: float,
    wheel_spiral_angle: float,
) -> HypoidThicknessGeometry:
    """Calculate ISO 23509 formulas 201 and 203-212.

    The public backlash input is ``j_et2``: outer transverse backlash on the
    wheel.  ISO converts it to the calculation point before applying the one
    common thickness correction to both members.  The wheel spiral angle is
    used for that conversion; each member's own spiral angle is then used by
    formula 212 when its normal thickness is reported transversely.
    """
    alpha_n = p.alpha
    cos_alpha_n = math.cos(alpha_n)
    if mean_normal_module <= 0.0 or not math.isfinite(mean_normal_module):
        raise ValueError("Method 1 mean normal module is not positive")
    if wheel_mean_cone_distance <= 0.0 or wheel_outer_cone_distance <= 0.0:
        raise ValueError("Method 1 wheel cone distances are not positive")
    if abs(cos_alpha_n) < 1e-12:
        raise ValueError("Method 1 tooth-thickness pressure angle is singular")

    outer_transverse = p.outer_transverse_backlash
    if not math.isfinite(outer_transverse) or outer_transverse < 0.0:
        raise ValueError("outer transverse backlash must be finite and non-negative")
    mean_transverse = outer_transverse * (
        wheel_mean_cone_distance / wheel_outer_cone_distance
    )
    mean_normal = mean_transverse * abs(math.cos(wheel_spiral_angle))
    backlash_x = mean_normal / (4.0 * mean_normal_module * cos_alpha_n)
    theoretical_x = 0.5 * p.thickness_factor
    return HypoidThicknessGeometry(
        mean_normal_pressure_angle=alpha_n,
        theoretical_thickness_modification=theoretical_x,
        backlash_thickness_modification=backlash_x,
        pinion_thickness_modification=theoretical_x - backlash_x,
        gear_thickness_modification=-theoretical_x - backlash_x,
        outer_transverse_backlash=outer_transverse,
        mean_transverse_backlash=mean_transverse,
        mean_normal_backlash=mean_normal,
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
    face_width,
    outer_face_width,
    inner_face_width,
    pitch_apex_z,
    mean_pitch_z,
    face_apex_z,
    root_apex_z,
    profile_shift_coefficient,
    thickness_modification_coefficient,
    generated_drive_normal_pressure_angle,
    generated_coast_normal_pressure_angle,
    inner_spiral_angle,
    outer_spiral_angle,
):
    """Build one member from the ISO Method 1 blank dimensions.

    ``face_width`` is the Method 1 calculated member facewidth.  For an
    offset pinion it is ``b_reri1`` from ISO 23509 formula 160, whereas the
    physical tooth boundaries are located with ``b_e1`` and ``b_i1``.
    Consequently the physical pitch-cone span is ``b_e1 + b_i1`` and is not
    generally equal to ``face_width``.  The wheel's requested ``b2`` is split
    into ``b_e2`` and ``b_i2``, so its two values happen to coincide.

    ``tredgold_tip_radius`` and ``tredgold_mean_root_radius`` deliberately
    remain the Tredgold back-cone section radii used by the involute
    approximation.  The physical Method 1 blank radii are the explicit
    ``*_tip_radius`` and ``*_root_radius`` fields below; they are not forced
    to agree with the developed section.
    """
    cos_delta = math.cos(delta)
    sin_delta = math.sin(delta)
    if not 0.0 < delta < math.pi / 2.0 or abs(cos_delta) < 1e-12:
        raise ValueError(f"{name} Method 1 pitch angle is outside the external-pair range")
    if cone_distance <= 0.0 or inner_cone_distance <= 0.0:
        raise ValueError(f"{name} Method 1 cone distance is not positive")
    if face_width <= 0.0:
        raise ValueError(f"{name} Method 1 member facewidth is not positive")
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
    # This is a nominal mean-section reference circle.  The actual generated
    # drive and coast bases use their distinct Method 1 normal angles after
    # conversion to the local transverse-equivalent section.
    virtual_base = virtual_pitch * math.cos(
        normal_to_transverse_pressure_angle(p.alpha, spiral)
    )
    virtual_tip = virtual_pitch + addendum
    virtual_root = virtual_pitch - dedendum
    if virtual_root <= 0.0:
        raise ValueError(f"{name} Method 1 Tredgold root radius is not positive")
    tredgold_tip_radius = virtual_tip * cos_delta
    tredgold_mean_root_radius = virtual_root * cos_delta

    # ISO 23509 formulas 206 and 211.  The thickness modification coefficient
    # already contains the one common backlash conversion; it is not a direct
    # length subtracted independently from each member.
    if name == "gear":
        thickness_term = (
            thickness_modification_coefficient - profile_shift_coefficient
        )
    else:
        thickness_term = (
            thickness_modification_coefficient + profile_shift_coefficient
        )
    normal_thickness = 0.5 * tooth_module * (
        math.pi + 2.0 * thickness_term * math.tan(p.alpha)
    )
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
        tooth_face_inner_cone_distance=inner_cone_distance,
        tooth_face_outer_cone_distance=outer_cone_distance,
        mean_spiral_angle=spiral,
        inner_spiral_angle=inner_spiral_angle,
        outer_spiral_angle=outer_spiral_angle,
        generated_drive_normal_pressure_angle=(
            generated_drive_normal_pressure_angle
        ),
        generated_coast_normal_pressure_angle=(
            generated_coast_normal_pressure_angle
        ),
        addendum=addendum,
        dedendum=dedendum,
        working_depth=working_depth,
        clearance=clearance,
        whole_depth=whole_depth,
        addendum_angle=addendum_angle,
        dedendum_angle=dedendum_angle,
        face_angle=face_angle,
        root_angle=root_angle,
        face_width=face_width,
        face_width_along_pitch_cone=outer_face_width + inner_face_width,
        tooth_face_width=outer_face_width + inner_face_width,
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
        thickness_modification_coefficient=thickness_modification_coefficient,
        mean_normal_tooth_thickness=normal_thickness,
        mean_transverse_tooth_thickness=transverse_thickness,
        tredgold_tip_radius=tredgold_tip_radius,
        tredgold_mean_root_radius=tredgold_mean_root_radius,
        tredgold_outer_root_radius=(
            virtual_root * outer_cone_distance / cone_distance * cos_delta
        ),
    )


@dataclass(frozen=True)
class _Method1BoundarySpirals:
    """Method 1 boundary cone distances (mm) and spiral angles (radians)."""

    pinion_boundary_wheel_outer: float
    pinion_boundary_wheel_inner: float
    pinion_inner: float
    pinion_outer: float
    wheel_inner: float
    wheel_outer: float


def _method1_boundary_spirals(
    p: HypoidSetParams,
    *,
    R1: float,
    R2: float,
    zeta_mp: float,
    pitch_plane_offset: float,
    beta1: float,
    beta2: float,
    be1: float,
    bi1: float,
    be2: float,
    bi2: float,
) -> _Method1BoundarySpirals:
    """Evaluate the Method 1 face-milling spiral geometry at both faces.

    The wheel cone distances corresponding to the pinion boundaries are not
    the wheel's own ``Re2``/``Ri2``.  ISO 23509 formulas 174/175 obtain them
    from the pinion face widths and the mean pitch-plane offset angle.  The
    cutter arc is then evaluated at those corresponding wheel distances
    (formulas 183/184), and the local pinion offset angle is added to obtain
    the pinion boundary spiral angles (formulas 185-187).
    """
    cos_zeta = math.cos(abs(zeta_mp))
    Re21 = math.sqrt(R2 * R2 + be1 * be1 + 2.0 * R2 * be1 * cos_zeta)
    Ri21 = math.sqrt(R2 * R2 + bi1 * bi1 - 2.0 * R2 * bi1 * cos_zeta)
    if Re21 <= 0.0 or Ri21 <= 0.0:
        raise ValueError("Method 1 pinion boundary cone distance is not positive")

    hand_sign = 1.0 if p.hand == "right" else -1.0

    def trace_angle(mean_radius: float, mean_angle: float, radius: float) -> float:
        if p.cutter_radius is None:
            return mean_angle
        trace = CrownTrace.for_set(
            abs(mean_angle), p.cutter_radius, mean_radius, sign=hand_sign
        )
        if not trace.reaches(radius, radius):
            raise ValueError(
                "cutter radius does not reach a Method 1 longitudinal boundary"
            )
        return trace.spiral_angle_at(radius)

    wheel_inner = trace_angle(R2, beta2, R2 - bi2)
    wheel_outer = trace_angle(R2, beta2, R2 + be2)
    wheel_at_pinion_inner = trace_angle(R2, beta2, Ri21)
    wheel_at_pinion_outer = trace_angle(R2, beta2, Re21)
    if abs(pitch_plane_offset) <= 1e-12:
        pinion_inner = trace_angle(R1, beta1, R1 - bi1)
        pinion_outer = trace_angle(R1, beta1, R1 + be1)
    else:
        inner_offset_ratio = abs(pitch_plane_offset) / Ri21
        outer_offset_ratio = abs(pitch_plane_offset) / Re21
        if inner_offset_ratio > 1.0 or outer_offset_ratio > 1.0:
            raise ValueError(
                "Method 1 pitch-plane offset exceeds a longitudinal boundary"
            )
        pinion_inner = wheel_at_pinion_inner + hand_sign * math.asin(
            inner_offset_ratio
        )
        pinion_outer = wheel_at_pinion_outer + hand_sign * math.asin(
            outer_offset_ratio
        )

    return _Method1BoundarySpirals(
        pinion_boundary_wheel_outer=Re21,
        pinion_boundary_wheel_inner=Ri21,
        pinion_inner=pinion_inner,
        pinion_outer=pinion_outer,
        wheel_inner=wheel_inner,
        wheel_outer=wheel_outer,
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
    thickness = _method1_thickness(
        p, m_n, R2, Re2, beta2
    )

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

    boundary_spirals = _method1_boundary_spirals(
        p,
        R1=R1,
        R2=R2,
        zeta_mp=method1.pinion_offset_angle_pitch,
        pitch_plane_offset=method1.pitch_plane_offset,
        beta1=beta1,
        beta2=beta2,
        be1=be1,
        bi1=bi1,
        be2=be2,
        bi2=bi2,
    )

    method1 = replace(
        method1,
        wheel_face_width_factor=cbe2,
        wheel_outer_face_width=be2,
        wheel_inner_face_width=bi2,
        pinion_face_width=breri1,
        pinion_face_width_increment_along_axis=delta_bx1,
        pinion_outer_face_width=be1,
        pinion_inner_face_width=bi1,
        pinion_boundary_wheel_outer_cone_distance=(
            boundary_spirals.pinion_boundary_wheel_outer
        ),
        pinion_boundary_wheel_inner_cone_distance=(
            boundary_spirals.pinion_boundary_wheel_inner
        ),
        pinion_inner_spiral_angle=boundary_spirals.pinion_inner,
        pinion_outer_spiral_angle=boundary_spirals.pinion_outer,
        wheel_inner_spiral_angle=boundary_spirals.wheel_inner,
        wheel_outer_spiral_angle=boundary_spirals.wheel_outer,
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
        # ISO 23509 Method 1 formula 160 (b_reri1).  This is the calculated
        # pinion facewidth; the physical pitch-cone boundary span is be1+bi1.
        face_width=breri1,
        outer_face_width=be1,
        inner_face_width=bi1,
        pitch_apex_z=tz1,
        mean_pitch_z=tzm1,
        face_apex_z=tzF1,
        root_apex_z=tzR1,
        profile_shift_coefficient=depth.profile_shift_coefficient,
        thickness_modification_coefficient=(
            thickness.pinion_thickness_modification
        ),
        generated_drive_normal_pressure_angle=(
            method1.generated_drive_normal_pressure_angle
        ),
        generated_coast_normal_pressure_angle=(
            method1.generated_coast_normal_pressure_angle
        ),
        inner_spiral_angle=boundary_spirals.pinion_inner,
        outer_spiral_angle=boundary_spirals.pinion_outer,
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
        # For the wheel, the requested b2 is also its physical pitch-cone
        # span because the wheel calculation point is split by c_be2.
        face_width=p.face_width,
        outer_face_width=be2,
        inner_face_width=bi2,
        pitch_apex_z=tz2,
        mean_pitch_z=tzm2,
        face_apex_z=tzF2,
        root_apex_z=tzR2,
        profile_shift_coefficient=depth.profile_shift_coefficient,
        thickness_modification_coefficient=(
            thickness.gear_thickness_modification
        ),
        generated_drive_normal_pressure_angle=(
            method1.generated_drive_normal_pressure_angle
        ),
        generated_coast_normal_pressure_angle=(
            method1.generated_coast_normal_pressure_angle
        ),
        inner_spiral_angle=boundary_spirals.wheel_inner,
        outer_spiral_angle=boundary_spirals.wheel_outer,
    )
    return HypoidSetGeometry(
        params=p,
        outer_cone_dist=gear.outer_cone_distance,
        mean_cone_dist=gear.cone_distance,
        inner_cone_dist=gear.inner_cone_distance,
        mean_normal_module=m_n,
        basic_addendum_factor=depth.basic_addendum_factor,
        basic_dedendum_factor=depth.basic_dedendum_factor,
        method1_profile_shift_coefficient=depth.profile_shift_coefficient,
        mean_working_depth=depth.working_depth,
        mean_clearance=depth.clearance,
        mean_whole_depth=depth.whole_depth,
        offset_angle=method1.pinion_offset_angle_pitch,
        pitch_plane_offset=method1.pitch_plane_offset, method1=method1,
        thickness=thickness,
        pinion=pinion, gear=gear,
    )


def _cutter_trace(
    member: HypoidMemberGeometry, geo: HypoidSetGeometry
) -> CrownTrace | None:
    """Return the circular cutter trace in the member's own mean geometry.

    The cutter radius is a lengthwise curvature radius, so the circular arc is
    placed at the requested member's Method 1 calculation point and mean
    spiral angle.  For a non-zero-offset pinion, the production phase uses the
    wheel trace transported through the Method 1 offset construction; this
    helper still returns the member-local arc for the bevel-like zero-offset
    case and for diagnostics.
    """
    radius = geo.params.cutter_radius
    if radius is None:
        return None
    if radius <= 0.0:
        raise ValueError("cutter radius must be greater than zero")
    return CrownTrace.for_set(
        abs(member.mean_spiral_angle),
        radius,
        member.cone_distance,
        sign=1.0 if geo.params.hand == "right" else -1.0,
    )


def _pinion_corresponding_wheel_cone_distance(
    geo: HypoidSetGeometry, cone_dist: float
) -> float:
    """Return the Method 1 wheel distance corresponding to pinion ``A``.

    If ``t = A - R1`` and ``zeta_mp`` is the signed mean pitch-plane offset
    angle, ISO's boundary construction (formulas 174/175) gives

        B(A)^2 = R2^2 + t^2 + 2 R2 t cos(|zeta_mp|).

    At the pinion mean point this is ``B(R1) = R2``.  The sign of the physical
    hypoid offset is deliberately absent: it mirrors the skew assembly but
    does not change the member's Method 1 spiral magnitudes.
    """
    pinion, gear = geo.pinion, geo.gear
    t = cone_dist - pinion.cone_distance
    return math.sqrt(
        gear.cone_distance ** 2
        + t ** 2
        + 2.0 * gear.cone_distance * t
        * math.cos(abs(geo.method1.pinion_offset_angle_pitch))
    )


def _method1_spiral_angle_at(
    member: HypoidMemberGeometry, cone_dist: float, geo: HypoidSetGeometry
) -> float:
    """Return the signed Method 1 longitudinal angle at physical ``A``.

    The wheel uses its circular ``CrownTrace`` directly.  For a zero-offset
    pair the pinion has the same independent trace.  For a non-zero-offset
    pair, the Method 1 face construction evaluates the wheel cutter trace at
    the corresponding wheel distance ``B(A)`` and adds the local offset
    angle::

        beta_1(A) = beta_2(B(A))
                    + hand * asin(|pitch_plane_offset| / B(A)).

    This is the continuous form of the same relation used by
    ``_method1_boundary_spirals`` for the inner and outer physical faces.  It
    describes the Method 1 longitudinal trace used by this approximation; it
    is not a claim that the Tredgold sections are a true generated or
    conjugate hypoid flank.
    """
    trace = _cutter_trace(member, geo)
    if member.name == "gear" or abs(geo.pitch_plane_offset) <= 1e-12:
        return (
            trace.spiral_angle_at(cone_dist)
            if trace is not None
            else member.mean_spiral_angle
        )

    # The non-zero-offset Method 1 pinion boundary is transported from the
    # wheel trace rather than from an unrelated circular arc on the pinion.
    wheel_trace = _cutter_trace(geo.gear, geo)
    wheel_dist = _pinion_corresponding_wheel_cone_distance(geo, cone_dist)
    wheel_angle = (
        wheel_trace.spiral_angle_at(wheel_dist)
        if wheel_trace is not None
        else geo.gear.mean_spiral_angle
    )
    offset_ratio = abs(geo.pitch_plane_offset) / wheel_dist
    if offset_ratio >= 1.0:
        raise ValueError("hypoid Method 1 pinion trace offset is singular")
    hand = 1.0 if geo.params.hand == "right" else -1.0
    return wheel_angle + hand * math.asin(offset_ratio)


def _integrate_phase_tangent(
    member: HypoidMemberGeometry,
    start: float,
    stop: float,
    geo: HypoidSetGeometry,
) -> float:
    """Integrate the physical cone phase from ``start`` to ``stop``.

    Let the pitch-cone point be

        P(A) = A (sin(delta) cos(phi), sin(delta) sin(phi), cos(delta)).

    Its generator and circumferential components give

        tan(beta_actual) = A sin(delta) d(phi)/dA.

    ``phi`` is the phase passed to ``to_cone_3d`` (apart from a constant
    profile azimuth), so the required physical phase is therefore

        phi(A) - phi(R) = integral_R^A
            tan(beta_method1(u)) / (u sin(delta)) du.

    This quadrature is only needed for a non-zero-offset pinion, where the
    Method 1 offset transport makes ``beta_method1(A)`` no longer the tangent
    of one circular crown arc in the pinion's own cone parameter.  The wheel
    and zero-offset paths use the exact ``CrownTrace.theta_at / sin(delta)``
    mapping directly.
    """
    if start == stop:
        return 0.0
    sin_delta = math.sin(member.pitch_angle)
    if abs(sin_delta) < 1e-12:
        raise ValueError("hypoid trace phase has a singular pitch angle")

    reverse = stop < start
    lo, hi = (stop, start) if reverse else (start, stop)

    def integrand(value: float) -> float:
        beta = _method1_spiral_angle_at(member, value, geo)
        return math.tan(beta) / (value * sin_delta)

    def simpson(a, b, fa, fb, fc) -> float:
        return (b - a) * (fa + 4.0 * fc + fb) / 6.0

    fa = integrand(lo)
    fb = integrand(hi)
    mid = 0.5 * (lo + hi)
    fm = integrand(mid)
    whole = simpson(lo, hi, fa, fb, fm)
    tolerance = PHASE_INTEGRATION_TOLERANCE_RAD * (1.0 + abs(whole))

    def recurse(a, b, fa, fb, fc, whole, depth) -> float:
        mid = 0.5 * (a + b)
        left_mid = 0.5 * (a + mid)
        right_mid = 0.5 * (mid + b)
        f_left_mid = integrand(left_mid)
        f_right_mid = integrand(right_mid)
        left = simpson(a, mid, fa, fc, f_left_mid)
        right = simpson(mid, b, fc, fb, f_right_mid)
        refined = left + right
        if (
            depth <= 0
            or abs(refined - whole) <= 15.0 * tolerance
        ):
            return refined + (refined - whole) / 15.0
        return (
            recurse(a, mid, fa, fc, f_left_mid, left, depth - 1)
            + recurse(mid, b, fc, fb, f_right_mid, right, depth - 1)
        )

    result = recurse(
        lo, hi, fa, fb, fm, whole, PHASE_INTEGRATION_MAX_DEPTH
    )
    return -result if reverse else result


def _phase(member: HypoidMemberGeometry, cone_dist: float, geo: HypoidSetGeometry) -> float:
    """Return the physical section rotation at cone distance ``A``.

    ``CrownTrace.theta_at(A)`` is an angle in the generating crown plane.  A
    point at crown radius ``A`` maps to circumferential radius
    ``A * sin(delta)`` on a member's pitch cone.  Preserving the swept arc
    length requires

        (A sin(delta)) d(phi) = A d(theta_crown),
        hence d(phi) = d(theta_crown) / sin(delta).

    The phase here is the physical ``phi`` added by ``to_cone_3d``; it must not
    be treated as another back-cone-development angle.  On the cosine-rule
    branch used by ``CrownTrace``, differentiating the raw angle gives

        d(-trace.sign * theta_at) / dA = tan(beta_trace(A)) / A.

    That signed orientation is why the circular-arc phase below has the
    leading ``-trace.sign``; it also handles a Zerol trace whose tangent
    changes sign across the face.

    For a non-zero-offset pinion, Method 1 gives a member-specific local
    spiral angle through the corresponding wheel distance and offset angle.
    `_integrate_phase_tangent` enforces the same cone differential equation so
    both physical pinion face boundaries retain their Method 1 values.  The
    final ``member_sense`` keeps the existing opposite local phase senses of a
    meshed pair; Method 1 member spiral fields are compared as magnitudes (or
    with this sense restored), not as the signs of the two local rotations.
    """
    if cone_dist <= 0.0:
        raise ValueError("hypoid trace cone distance must be positive")
    sin_delta = math.sin(member.pitch_angle)
    if abs(sin_delta) < 1e-12:
        return 0.0

    trace = _cutter_trace(member, geo)
    zero_offset = abs(geo.pitch_plane_offset) <= 1e-12
    if trace is not None and (member.name == "gear" or zero_offset):
        # The raw crown angle's cosine-rule orientation is opposite to the
        # signed CrownTrace tangent, so -trace.sign*theta/sin(delta) is the
        # physical cone phase.
        phase_curve = -trace.sign * trace.theta_at(cone_dist) / sin_delta
    elif trace is None and zero_offset:
        # An absent cutter is the constant-angle idealisation.  Integrating
        # tan(beta)/A gives tan(beta) * log(A/R), still with the cone mapping.
        phase_curve = (
            math.tan(member.mean_spiral_angle)
            * math.log(cone_dist / member.cone_distance)
            / sin_delta
        )
    else:
        phase_curve = _integrate_phase_tangent(
            member, member.cone_distance, cone_dist, geo
        )

    # The production pair intentionally traverses the common trace in
    # opposite local rotational senses, as the bevel production path does.
    # ``phase_curve`` is oriented by the signed Method 1 spiral tangent.  Keep
    # the existing opposite local phase senses of the hypoid pair: the pinion
    # receives the curve directly and the wheel receives its negative.
    member_sense = 1.0 if member.name == "pinion" else -1.0
    return member_sense * phase_curve


def _reflect_flank(points: list[Point2]) -> list[Point2]:
    return [(x, -y) for x, y in points]


def _hypoid_root_fillet(
    flank: list[Point2], r_root: float, rho: float, *, negative: bool,
    strict: bool = False,
) -> tuple[list[Point2], list[Point2]]:
    """Fit one independent approximate circular root fillet.

    A positive radius is a requested geometric feature, not a best-effort
    hint. Refusing a radius that cannot meet this particular flank prevents
    one side of an independent drive/coast section from silently becoming a
    sharp corner.
    """
    if not math.isfinite(rho) or rho < 0.0:
        raise ValueError("hypoid root fillet radius must be finite and non-negative")
    if not negative:
        result = involute.root_fillet(flank, r_root, rho)
    else:
        # ``root_fillet`` is deliberately shared with the external involute
        # code and expects its working flank on the positive-y side.  Reflect
        # only this independently constructed coast/drive curve for the fit;
        # the resulting geometry is reflected back immediately.
        result = involute.root_fillet(_reflect_flank(flank), r_root, rho)
    if result is None and rho > 0.0 and strict:
        side = "negative" if negative else "positive"
        raise ValueError(
            f"hypoid root fillet radius {rho:.6g} mm does not fit the "
            f"{side} tooth-space flank"
        )
    if result is None:
        return flank, []
    trimmed, arc = result
    if negative:
        return _reflect_flank(trimmed), _reflect_flank(arc)
    return trimmed, arc


def hypoid_tooth_space_loop(
    left_flank: list[Point2],
    right_flank: list[Point2],
    r_root: float,
    r_cap: float,
    fillet_rho: float,
    *,
    split_cap: bool = False,
    strict_fillet: bool = False,
) -> tuple[
    dict[str, list[Point2]], list[Point2], bool, list[Point2], list[Point2]
]:
    """Assemble a closed hypoid space from independent left/right flanks.

    Both input curves run from the root toward the tip.  They are not assumed
    to be reflections of one another.  This is still a Tredgold/back-cone
    approximation: independently parameterized involutes do not constitute a
    generated cutter envelope or a fully conjugate hypoid surface.
    """
    if len(left_flank) < 2 or len(right_flank) < 2:
        raise ValueError("hypoid flank curves need at least two points")
    if r_root <= 0.0 or r_cap <= 0.0 or r_cap <= r_root:
        raise ValueError("hypoid tooth-space root and cap radii are invalid")
    if not math.isfinite(fillet_rho) or fillet_rho < 0.0:
        raise ValueError("hypoid root fillet radius must be finite and non-negative")
    points = [*left_flank, *right_flank]
    if any(
        not math.isfinite(value)
        for point in points
        for value in point
    ):
        raise ValueError("hypoid flank curve contains a non-finite point")

    left_flank, left_arc = _hypoid_root_fillet(
        left_flank, r_root, fillet_rho, negative=True, strict=strict_fillet
    )
    right_flank, right_arc = _hypoid_root_fillet(
        right_flank, r_root, fillet_rho, negative=False, strict=strict_fillet
    )
    left_root = math.atan2(
        (left_arc[0] if left_arc else left_flank[0])[1],
        (left_arc[0] if left_arc else left_flank[0])[0],
    )
    right_root = math.atan2(
        (right_arc[0] if right_arc else right_flank[0])[1],
        (right_arc[0] if right_arc else right_flank[0])[0],
    )
    left_tip = math.atan2(left_flank[-1][1], left_flank[-1][0])
    right_tip = math.atan2(right_flank[-1][1], right_flank[-1][0])
    if not left_root < 0.0 < right_root:
        raise ValueError("hypoid root flanks do not bound the space centreline")
    if not left_tip < 0.0 < right_tip:
        raise ValueError("hypoid tip flanks do not bound the space centreline")

    # Include an actual centreline vertex.  Unlike the mirrored involute
    # builder, the two tip angles need not be equal, so the midpoint of a
    # uniformly sampled cap is not generally the centreline.
    cap_angles = [left_tip, 0.5 * left_tip, 0.0, 0.5 * right_tip, right_tip]
    cap = [involute.polar(r_cap, angle) for angle in cap_angles]
    root = [
        involute.polar(
            r_root, right_root + (left_root - right_root) * i / 8.0
        )
        for i in range(9)
    ]
    segments = {
        "fillet_neg": left_arc,
        "flank_neg": left_flank,
        "riser_neg": [left_flank[-1], cap[0]],
        "cap": cap,
        "riser_pos": [cap[-1], right_flank[-1]],
        "flank_pos": right_flank[::-1],
        "fillet_pos": right_arc[::-1],
        "root": root,
    }
    order = [
        "fillet_neg", "flank_neg", "riser_neg", "cap",
        "riser_pos", "flank_pos", "fillet_pos", "root",
    ]
    if split_cap:
        mid = len(cap) // 2
        segments["cap_neg"] = cap[: mid + 1]
        segments["cap_pos"] = cap[mid:]
        del segments["cap"]
        order[order.index("cap")] = "cap_neg"
        order.insert(order.index("cap_neg") + 1, "cap_pos")

    loop: list[Point2] = []
    for name in order:
        for point in segments[name]:
            if not loop or math.dist(loop[-1], point) > 1e-9:
                loop.append(point)
    if loop and math.dist(loop[0], loop[-1]) < 1e-9:
        loop.pop()
    return (
        segments,
        loop,
        bool(left_arc and right_arc),
        left_flank,
        right_flank,
    )


def _hypoid_flank_points(
    member: HypoidMemberGeometry,
    cone_dist: float,
    normal_pressure_angle: float,
    spiral_angle: float,
    normal_tooth_thickness: float,
    n_flank: int,
) -> list[Point2]:
    """Build one transverse-equivalent Tredgold involute.

    The Method 1 angle is normal to the local spiral tooth trace, while this
    involute lives in the developed back-cone plane.  Convert the angle and the
    local normal tooth thickness before constructing the planar profile.
    """
    scale = cone_dist / max(member.cone_distance, 1e-9)
    pitch = member.virtual_pitch_r * scale
    transverse_pressure_angle = normal_to_transverse_pressure_angle(
        normal_pressure_angle, spiral_angle
    )
    transverse_tooth_thickness = normal_tooth_thickness / max(
        abs(math.cos(spiral_angle)), 1e-12
    )
    base = pitch * math.cos(transverse_pressure_angle)
    root = member.virtual_root_r * scale
    tip = member.virtual_tip_r * scale
    half_pitch = math.pi / member.virtual_teeth
    tooth_half_angle = transverse_tooth_thickness / (2.0 * max(pitch, 1e-9))
    space_half_angle = half_pitch - tooth_half_angle
    if base <= 0.0 or tip <= base:
        raise ValueError(
            f"{member.name} generated {math.degrees(transverse_pressure_angle):.3f} degree "
            "flank does not reach the involute tip"
        )
    if not 0.0 < space_half_angle < half_pitch:
        raise ValueError(f"{member.name} generated tooth space is not positive")
    # ``flank_points`` represents the boundary of the tooth *space*.  Its
    # phase is therefore the tooth half-angle at the pitch circle plus the
    # involute function, not the complementary space half-angle.  Keeping
    # this relation explicit is important when the drive and coast angles
    # have different involute functions.
    psi0 = tooth_half_angle + involute.inv(transverse_pressure_angle)
    points = involute.flank_points(
        base, root, tip, psi0, half_pitch, n_flank
    )
    if points[0][1] >= 0.0:
        return points

    # At unusually large local transverse angles the standard radial
    # below-base simplification can extrapolate through the section centreline
    # even though the involute above the base circle returns to the requested
    # positive side.  Retain the involute and replace only that below-base
    # transition with a tiny positive root lead; the true trochoid is already
    # outside this Tredgold approximation.  This keeps loft-only and physical
    # sections constructible without changing the pitch-circle involute.
    for index in range(1, len(points)):
        previous, current = points[index - 1], points[index]
        if current[1] >= 0.0:
            fraction = -previous[1] / (current[1] - previous[1])
            crossing = (
                previous[0] + fraction * (current[0] - previous[0]),
                0.0,
            )
            root_lead = involute.polar(root, 1e-9)
            return [root_lead, crossing, *points[index + 1:]]
    raise ValueError(
        f"{member.name} generated transverse involute does not reach its "
        "requested flank side"
    )


def tooth_space_section(geo: HypoidSetGeometry, member: str, cone_dist: float | None = None,
                        split_cap: bool = False,
                        n_flank: int = involute.FLANK_POINTS) -> HypoidSection:
    """Build an independently constructed Method 1 Tredgold tooth space.

    The drive and coast normal pressure angles are distinct Method 1
    quantities.  Each is converted to the local transverse-equivalent angle
    with the authoritative Method 1 spiral evaluator before its independent
    involute is built.  The normal tooth thickness is scaled with cone distance
    and converted to local transverse thickness in the same section.  The
    resulting pair remains an approximation rather than a true cutter-envelope
    hypoid surface.
    """
    m = geo.member(member)
    if cone_dist is None:
        cone_dist = m.cone_distance
    scale = cone_dist / max(m.cone_distance, 1e-9)
    root = m.virtual_root_r * scale
    tip = m.virtual_tip_r * scale
    cap = tip + involute.CUT_OVERSHOOT_FACTOR * geo.params.module
    # Method 1 defines beta(A) over the physical tooth face.  Loft-only
    # clearance stations can lie beyond that interval, where extending the
    # cutter trace would eventually drive cos(beta) through zero and make a
    # transverse section meaningless.  Hold the profile conversion at the
    # nearest physical-face value for those construction stations; the phase
    # still uses the true requested cone distance below.
    profile_cone_dist = min(
        max(cone_dist, m.tooth_face_inner_cone_distance),
        m.tooth_face_outer_cone_distance,
    )
    spiral_angle = _method1_spiral_angle_at(m, profile_cone_dist, geo)
    mean_spiral_cos = abs(math.cos(m.mean_spiral_angle))
    local_spiral_cos = abs(math.cos(spiral_angle))
    if mean_spiral_cos < 1e-12 or local_spiral_cos < 1e-12:
        raise ValueError("hypoid section tooth thickness is singular at a 90 degree spiral")
    # The developed section has a transverse pitch that scales with cone
    # distance.  Its corresponding normal pitch therefore also carries the
    # local cos(beta) factor.  Converting that local normal tooth thickness back
    # to transverse gives the mean transverse thickness scaled through the
    # section, which is the Tredgold angular-thickness construction:
    #
    #   s_n(A) = s_n,m * scale * cos(beta(A)) / cos(beta_m)
    #   s_t(A) = s_n(A) / cos(beta(A))
    #          = s_t,m * scale.
    transverse_tooth_thickness = m.mean_transverse_tooth_thickness * scale
    normal_tooth_thickness = transverse_tooth_thickness * local_spiral_cos
    positive_drive = geo.params.hand == "right"
    positive_normal_angle = (
        m.generated_drive_normal_pressure_angle
        if positive_drive
        else m.generated_coast_normal_pressure_angle
    )
    negative_normal_angle = (
        m.generated_coast_normal_pressure_angle
        if positive_drive
        else m.generated_drive_normal_pressure_angle
    )
    positive_transverse_angle = normal_to_transverse_pressure_angle(
        positive_normal_angle, spiral_angle
    )
    negative_transverse_angle = normal_to_transverse_pressure_angle(
        negative_normal_angle, spiral_angle
    )
    pitch = m.virtual_pitch_r * scale
    positive_base = pitch * math.cos(positive_transverse_angle)
    negative_base = pitch * math.cos(negative_transverse_angle)
    positive_flank = _hypoid_flank_points(
        m,
        cone_dist,
        positive_normal_angle,
        spiral_angle,
        normal_tooth_thickness,
        n_flank,
    )
    negative_flank = _reflect_flank(
        _hypoid_flank_points(
            m,
            cone_dist,
            negative_normal_angle,
            spiral_angle,
            normal_tooth_thickness,
            n_flank,
        )
    )
    segments, loop, _, left_flank, right_flank = hypoid_tooth_space_loop(
        negative_flank,
        positive_flank,
        root,
        cap,
        geo.params.effective_root_fillet_radius,
        split_cap=split_cap,
        strict_fillet=geo.params.root_fillet_radius is not None,
    )
    drive_flank, coast_flank = (
        (right_flank, left_flank)
        if positive_drive
        else (left_flank, right_flank)
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
        r_root=root, r_tip=tip, r_cap=cap,
        spiral_angle=spiral_angle,
        normal_tooth_thickness=normal_tooth_thickness,
        transverse_tooth_thickness=transverse_tooth_thickness,
        drive_transverse_pressure_angle=(
            positive_transverse_angle if positive_drive else negative_transverse_angle
        ),
        coast_transverse_pressure_angle=(
            negative_transverse_angle if positive_drive else positive_transverse_angle
        ),
        drive_base_radius=positive_base if positive_drive else negative_base,
        coast_base_radius=negative_base if positive_drive else positive_base,
        root_fillet_radius=geo.params.effective_root_fillet_radius,
        filleted=bool(segments["fillet_neg"] and segments["fillet_pos"]),
        drive_flank=drive_flank, coast_flank=coast_flank,
        segments=segments, loop_2d=loop,
    )


def _section_clearance_bounds(
    geo: HypoidSetGeometry, member: str
) -> tuple[float, float]:
    """Find terminal loft distances beyond the actual Method 1 tooth face.

    The search is performed from the physical face limits, then bisected.  A
    fixed face-width overshoot is not valid for the two members because their
    Method 1 face widths, tooth depths, and cone apex locations differ.
    """
    m = geo.member(member)
    p = geo.params
    outline = blank_outline(geo, member)
    z_front = outline[0][1]
    i_back = 4 if p.min_root_thickness > 0.0 else 3
    z_back = outline[i_back][1]
    margin = max(0.1, 0.1 * p.module)
    step = max(p.module, m.face_width_along_pitch_cone / 8.0)

    def clears_front(cone_dist: float) -> bool:
        section = tooth_space_section(geo, member, cone_dist)
        return max(z for _, _, z in section.loop_3d()) < z_front - margin

    def clears_back(cone_dist: float) -> bool:
        section = tooth_space_section(geo, member, cone_dist)
        return min(z for _, _, z in section.loop_3d()) > z_back + margin

    face_inner = m.tooth_face_inner_cone_distance
    face_outer = m.tooth_face_outer_cone_distance

    if clears_front(face_inner):
        loft_inner = face_inner
    else:
        hi = face_inner
        lo = hi - step
        for _ in range(100):
            if lo <= 0.0:
                raise ValueError(
                    f"could not clear the {member} hypoid blank front face"
                )
            if clears_front(lo):
                break
            hi, lo = lo, lo - step
        else:
            raise ValueError(
                f"could not clear the {member} hypoid blank front face"
            )
        for _ in range(60):
            mid = 0.5 * (lo + hi)
            if clears_front(mid):
                lo = mid
            else:
                hi = mid
        loft_inner = lo

    if clears_back(face_outer):
        loft_outer = face_outer
    else:
        lo = face_outer
        hi = lo + step
        for _ in range(100):
            if clears_back(hi):
                break
            lo, hi = hi, hi + step
        else:
            raise ValueError(
                f"could not clear the {member} hypoid blank back face"
            )
        for _ in range(60):
            mid = 0.5 * (lo + hi)
            if clears_back(mid):
                hi = mid
            else:
                lo = mid
        loft_outer = hi

    return loft_inner, loft_outer


def section_cone_bounds(
    geo: HypoidSetGeometry, member: str
) -> HypoidSectionBounds:
    """Return physical Method 1 face limits plus loft-only extensions."""
    m = geo.member(member)
    loft_inner, loft_outer = _section_clearance_bounds(geo, member)
    return HypoidSectionBounds(
        calculation_point=m.cone_distance,
        tooth_face_inner=m.tooth_face_inner_cone_distance,
        tooth_face_outer=m.tooth_face_outer_cone_distance,
        loft_inner=loft_inner,
        loft_outer=loft_outer,
    )


def section_cone_distances(
    geo: HypoidSetGeometry, member: str, count: int = 8
) -> list[float]:
    """Return loft profiles, including marked-by-range terminal extensions.

    The first and last values are outside the physical face when clearance is
    required.  The actual inner/outer face limits and the Method 1 calculation
    point are inserted explicitly into the profile list so they cannot be lost
    by an arithmetic interpolation of the extension interval.
    """
    bounds = section_cone_bounds(geo, member)
    n = max(2, count)
    distances = [
        bounds.loft_inner
        + (bounds.loft_outer - bounds.loft_inner) * i / max(n - 1, 1)
        for i in range(n)
    ]
    distances.extend(
        value
        for value in (
            bounds.tooth_face_inner,
            bounds.calculation_point,
            bounds.tooth_face_outer,
        )
        if bounds.loft_inner < value < bounds.loft_outer
    )
    result: list[float] = []
    for value in sorted(distances):
        if not result or abs(value - result[-1]) > 1e-12:
            result.append(value)
    return result


def blank_outline(geo: HypoidSetGeometry, member: str) -> list[tuple[float, float]]:
    m = geo.member(member)
    p = geo.params
    bore = p.bore / 2.0 if member == "pinion" else max(0.5, p.bore / 2.0)
    # These are physical Method 1 points between the stored tooth-face cone
    # limits.  The Tredgold developed radii used for the approximate tooth
    # section are intentionally not used to size the revolved blank.
    if (
        m.tooth_face_inner_cone_distance <= 0.0
        or m.tooth_face_outer_cone_distance <= m.tooth_face_inner_cone_distance
    ):
        raise ValueError(f"{member} Method 1 tooth-face boundaries are invalid")
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
    # This is a SOLIDWORKS loft-station tolerance, not a Method 1 dimension.
    max_sagitta = max(0.01, 0.02 * geo.params.module)
    bounds = section_cone_bounds(geo, member)
    inner = bounds.loft_inner
    outer = bounds.loft_outer
    twist = abs(_phase(m, outer, geo) - _phase(m, inner, geo))
    step = 2.0 * math.acos(
        max(
            -1.0,
            min(1.0, 1.0 - max_sagitta / max(m.tredgold_tip_radius, 1e-9)),
        )
    )
    minimum = max(2, math.ceil(twist / max(step, 1e-9)) + 1)
    # The mandatory physical face limits and Method 1 calculation point may
    # add profiles to the uniform extension sweep.  Report the count that the
    # section sampler will actually return, not only the sagitta estimate.
    return len(section_cone_distances(geo, member, minimum))
