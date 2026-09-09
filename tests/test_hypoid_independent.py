"""Reference, invariant, and implementation-audit checks for hypoid geometry.

These checks intentionally do not import helpers from ``tests.test_hypoid`` and
do not use the solver's private trial function.  The fixed anchor assertions
are published Method 1 values.  The contact, placement, trace, hand, and
offset tests are independent physical invariants.  The tests whose names say
``recomputed``, ``formula``, or ``equation`` deliberately audit implementation
equations from public fields; they are not independent evidence of those same
equations.

The mesh probe at the end validates pitch-surface contact and tooth-phase
advance only.  It is not a claim that the Tredgold sections are a generated
cutter envelope or a full conjugate hypoid surface.  Full solid interference
measurement remains a separate SOLIDWORKS integration concern.
"""

from __future__ import annotations

import math
from dataclasses import replace

import pytest

from gears.hypoid.geometry import (
    compute_set,
    section_cone_bounds,
    section_cone_distances,
    tooth_space_section,
)
from gears.hypoid.mesh import (
    contact_azimuths,
    gear_mate_ratio,
    gear_translation,
    skew_axis_distance,
)
from gears.hypoid.params import HypoidSetParams
from gears.placement import angular_velocity_ratio, apply, rot_y, rot_z
from gears.bevel.geometry import to_cone_3d


ANCHOR = HypoidSetParams.with_defaults(
    170.0 / 42.0,
    13,
    42,
    offset=15.0,
    face_width=30.0,
    spiral_angle=50.0,
    cutter_radius=63.5,
)


# These are deliberately different from the published 13/42 set.  They are
# broad enough to exercise the axial/curvature closure without becoming
# boundary cases of the input validator.
DESIGN_A = dict(
    module=2.5,
    z1=19,
    z2=47,
    offset=7.5,
    face_width=12.0,
    spiral_angle=32.0,
    cutter_radius=45.0,
    shaft_angle=80.0,
)
DESIGN_B = dict(
    module=2.2,
    z1=17,
    z2=41,
    offset=-8.0,
    face_width=10.0,
    spiral_angle=28.0,
    cutter_radius=38.0,
    shaft_angle=90.0,
)
OTHER_DESIGNS = (
    pytest.param(DESIGN_A, id="19x47-positive-80deg"),
    pytest.param(DESIGN_B, id="17x41-negative-90deg"),
)


def _params(spec: dict, **changes) -> HypoidSetParams:
    values = dict(spec)
    values.update(changes)
    module = values.pop("module")
    z1 = values.pop("z1")
    z2 = values.pop("z2")
    return HypoidSetParams.with_defaults(module, z1, z2, **values)


CONTACT_CASES = (
    pytest.param(ANCHOR, id="anchor"),
    pytest.param(_params(DESIGN_A), id="positive-offset"),
    pytest.param(_params(DESIGN_B), id="negative-offset"),
    pytest.param(replace(ANCHOR, offset=0.0), id="zero-offset"),
)


def _finite_positive(*values: float) -> bool:
    return all(math.isfinite(value) and value > 0.0 for value in values)


def _recomputed_limit_radius_from_public_fields(geo) -> float:
    """ISO Method 1 curvature equation evaluated from public result fields."""
    pinion, wheel, method = geo.pinion, geo.gear, geo.method1
    beta1 = abs(pinion.mean_spiral_angle)
    beta2 = abs(wheel.mean_spiral_angle)
    zeta = abs(method.pinion_offset_angle_pitch)
    alpha_lim = math.atan(
        -math.tan(pinion.pitch_angle) * math.tan(wheel.pitch_angle)
        * (
            pinion.cone_distance * math.sin(beta1)
            - wheel.cone_distance * math.sin(beta2)
        )
        / (
            math.cos(zeta)
            * (
                pinion.cone_distance * math.tan(pinion.pitch_angle)
                + wheel.cone_distance * math.tan(wheel.pitch_angle)
            )
        )
    )
    denominator = (
        -math.tan(alpha_lim)
        * (
            math.tan(beta1)
            / (pinion.cone_distance * math.tan(pinion.pitch_angle))
            + math.tan(beta2)
            / (wheel.cone_distance * math.tan(wheel.pitch_angle))
        )
        + 1.0 / (pinion.cone_distance * math.cos(beta1))
        - 1.0 / (wheel.cone_distance * math.cos(beta2))
    )
    return (
        math.tan(beta1) - math.tan(beta2)
    ) / (math.cos(alpha_lim) * denominator)


def _local_contact_point(geo, member: str, theta: float):
    m = geo.member(member)
    return (
        m.pitch_radius * math.cos(theta),
        m.pitch_radius * math.sin(theta),
        m.cone_distance * math.cos(m.pitch_angle),
    )


def _world_contact_point(geo, member: str, theta: float):
    local = _local_contact_point(geo, member, theta)
    if member == "pinion":
        return local
    rotated = apply(rot_y(geo.params.sigma), local)
    translation = gear_translation(geo)
    return tuple(rotated[i] + translation[i] for i in range(3))


def _unit(vector):
    length = math.sqrt(sum(value * value for value in vector))
    return tuple(value / length for value in vector)


def _pitch_trace_point_at_contact(geo, member: str, cone_distance: float):
    """Map the contact generator through the production section path."""
    theta1, theta2 = contact_azimuths(geo)
    m = geo.member(member)
    theta = theta1 if member == "pinion" else theta2
    developed_radius = m.virtual_pitch_r * cone_distance / m.cone_distance
    # `to_cone_3d` divides a developed polar angle by cos(delta).  Supply the
    # developed angle that lands on the requested physical contact azimuth.
    developed_theta = theta * math.cos(m.pitch_angle)
    section = tooth_space_section(geo, member, cone_distance)
    local = to_cone_3d(
        developed_radius * math.cos(developed_theta),
        developed_radius * math.sin(developed_theta),
        m.pitch_angle,
        section.cone_apex_z,
        section.phase,
    )
    if member == "pinion":
        return local
    rotated = apply(rot_y(geo.params.sigma), local)
    translation = gear_translation(geo)
    return tuple(rotated[i] + translation[i] for i in range(3))


def _pitch_trace_tangent_at_contact(geo, member: str):
    m = geo.member(member)
    h = 1e-4
    before = _pitch_trace_point_at_contact(geo, member, m.cone_distance - h)
    after = _pitch_trace_point_at_contact(geo, member, m.cone_distance + h)
    return _unit(tuple(after[i] - before[i] for i in range(3)))


def _common_pitch_normals(geo):
    theta1, theta2 = contact_azimuths(geo)
    d1, d2, sigma = (
        geo.pinion.pitch_angle,
        geo.gear.pitch_angle,
        geo.params.sigma,
    )
    axis1 = (0.0, 0.0, 1.0)
    axis2 = (math.sin(sigma), 0.0, math.cos(sigma))
    radial1 = (math.cos(theta1), math.sin(theta1), 0.0)
    radial_x2 = (math.cos(sigma), 0.0, -math.sin(sigma))
    radial2 = (
        math.cos(theta2) * radial_x2[0],
        math.sin(theta2),
        math.cos(theta2) * radial_x2[2],
    )
    normal1 = tuple(
        math.cos(d1) * radial1[i] - math.sin(d1) * axis1[i]
        for i in range(3)
    )
    normal2 = tuple(
        -math.cos(d2) * radial2[i] + math.sin(d2) * axis2[i]
        for i in range(3)
    )
    return normal1, normal2


def _trace_angle_from_tangent(geo, member: str, tangent):
    theta1, theta2 = contact_azimuths(geo)
    m = geo.member(member)
    theta = theta1 if member == "pinion" else theta2
    generator = (
        math.sin(m.pitch_angle) * math.cos(theta),
        math.sin(m.pitch_angle) * math.sin(theta),
        math.cos(m.pitch_angle),
    )
    circumferential = (-math.sin(theta), math.cos(theta), 0.0)
    if member == "gear":
        rotation = rot_y(geo.params.sigma)
        generator = apply(rotation, generator)
        circumferential = apply(rotation, circumferential)
    return math.atan2(
        sum(tangent[i] * circumferential[i] for i in range(3)),
        sum(tangent[i] * generator[i] for i in range(3)),
    )


def test_published_method1_anchor_is_checked_against_fixed_values():
    geo = compute_set(ANCHOR)

    assert geo.pinion.pitch_angle_deg == pytest.approx(21.288, abs=0.02)
    assert geo.gear.pitch_angle_deg == pytest.approx(68.324, abs=0.02)
    assert geo.offset_angle_deg == pytest.approx(11.39, abs=0.03)
    assert geo.pitch_plane_offset == pytest.approx(15.075, abs=0.002)
    assert geo.pinion.mean_spiral_angle_deg == pytest.approx(50.0, abs=0.01)
    assert geo.gear.mean_spiral_angle_deg == pytest.approx(38.61, abs=0.03)
    assert geo.mean_normal_module == pytest.approx(2.64, abs=0.01)


def test_published_anchor_depth_face_and_generated_flank_values_are_fixed():
    geo = compute_set(ANCHOR)

    assert geo.pinion.addendum == pytest.approx(3.432, abs=0.002)
    assert geo.pinion.dedendum == pytest.approx(2.508, abs=0.002)
    assert geo.gear.addendum == pytest.approx(1.848, abs=0.002)
    assert geo.gear.dedendum == pytest.approx(4.092, abs=0.002)
    assert geo.pinion.face_angle_deg == pytest.approx(25.231, abs=0.002)
    assert geo.pinion.root_angle_deg == pytest.approx(20.302, abs=0.002)
    assert geo.gear.face_angle_deg == pytest.approx(69.324, abs=0.002)
    assert geo.gear.root_angle_deg == pytest.approx(64.324, abs=0.002)
    assert geo.method1.generated_drive_normal_pressure_angle_deg == pytest.approx(
        17.746, abs=0.01
    )
    assert geo.method1.generated_coast_normal_pressure_angle_deg == pytest.approx(
        22.254, abs=0.01
    )


def test_published_anchor_has_fixed_and_derived_member_face_boundaries():
    geo = compute_set(ANCHOR)
    method = geo.method1
    zeta = abs(method.pinion_offset_angle_pitch)
    expected_re21 = math.sqrt(
        geo.gear.cone_distance ** 2
        + geo.pinion.outer_face_width ** 2
        + 2.0
        * geo.gear.cone_distance
        * geo.pinion.outer_face_width
        * math.cos(zeta)
    )
    expected_ri21 = math.sqrt(
        geo.gear.cone_distance ** 2
        + geo.pinion.inner_face_width ** 2
        - 2.0
        * geo.gear.cone_distance
        * geo.pinion.inner_face_width
        * math.cos(zeta)
    )

    assert geo.pinion.tooth_face_inner_cone_distance == pytest.approx(
        57.6995, abs=0.002
    )
    assert geo.pinion.tooth_face_outer_cone_distance == pytest.approx(
        89.6097, abs=0.002
    )
    assert geo.gear.tooth_face_inner_cone_distance == pytest.approx(
        61.4680, abs=0.002
    )
    assert geo.gear.tooth_face_outer_cone_distance == pytest.approx(
        91.4680, abs=0.002
    )
    assert method.pinion_boundary_wheel_outer_cone_distance == pytest.approx(
        expected_re21, abs=1e-12
    )
    assert method.pinion_boundary_wheel_inner_cone_distance == pytest.approx(
        expected_ri21, abs=1e-12
    )
    assert geo.pinion.face_width == pytest.approx(30.470, abs=0.002)
    assert geo.pinion.face_width_along_pitch_cone == pytest.approx(31.910, abs=0.002)
    assert geo.gear.face_width == pytest.approx(30.000, abs=1e-12)
    assert geo.gear.face_width_along_pitch_cone == pytest.approx(30.000, abs=1e-12)
    assert geo.pinion.tooth_face_width == pytest.approx(
        geo.pinion.face_width_along_pitch_cone, abs=1e-12
    )
    assert geo.pinion.face_width != pytest.approx(
        geo.pinion.face_width_along_pitch_cone
    )


@pytest.mark.parametrize("spec", OTHER_DESIGNS)
def test_two_non_anchor_designs_close_without_anchor_regression_values(spec):
    params = _params(spec)
    geo = compute_set(params)

    assert geo.method1.iterations >= 1
    assert geo.method1.limit_radius_of_curvature == pytest.approx(
        _params(spec).cutter_radius, abs=1e-8
    )
    assert geo.pinion.pitch_angle + geo.gear.pitch_angle < (
        geo.params.sigma + math.pi / 2.0
    )
    assert geo.pinion.pitch_angle > 0.0
    assert geo.gear.pitch_angle > 0.0
    assert geo.pinion.tooth_face_inner_cone_distance < geo.pinion.tooth_face_outer_cone_distance
    assert geo.gear.tooth_face_inner_cone_distance < geo.gear.tooth_face_outer_cone_distance
    assert geo.pinion.face_width == pytest.approx(
        geo.method1.pinion_face_width, abs=1e-12
    )
    for member in (geo.pinion, geo.gear):
        assert member.face_width_along_pitch_cone == pytest.approx(
            member.outer_face_width + member.inner_face_width, abs=1e-12
        )
    if abs(params.offset) > 1e-12:
        assert geo.pinion.face_width != pytest.approx(
            geo.pinion.face_width_along_pitch_cone
        )


def test_zero_offset_reduces_to_the_independent_intersecting_bevel_limit():
    params = HypoidSetParams.with_defaults(
        2.4, 17, 43, offset=0.0, face_width=9.0, shaft_angle=60.0,
        spiral_angle=35.0, cutter_radius=40.0,
    )
    geo = compute_set(params)
    d1 = math.atan2(math.sin(params.sigma), params.ratio + math.cos(params.sigma))
    d2 = params.sigma - d1
    wheel_mean_cone = params.wheel_outer_radius / math.sin(d2) - params.face_width / 2.0
    wheel_radius = wheel_mean_cone * math.sin(d2)
    pinion_radius = wheel_radius / params.ratio

    assert geo.pinion.pitch_angle == pytest.approx(d1, abs=1e-12)
    assert geo.gear.pitch_angle == pytest.approx(d2, abs=1e-12)
    assert geo.pinion.pitch_radius == pytest.approx(pinion_radius, abs=1e-12)
    assert geo.gear.pitch_radius == pytest.approx(wheel_radius, abs=1e-12)
    assert geo.pinion.pitch_angle + geo.gear.pitch_angle == pytest.approx(
        params.sigma, abs=1e-12
    )
    assert geo.pitch_plane_offset == 0.0
    assert geo.method1.limit_radius_of_curvature is None
    assert geo.pinion.face_width == pytest.approx(geo.gear.face_width)
    assert geo.pinion.face_width_along_pitch_cone == pytest.approx(
        geo.gear.face_width_along_pitch_cone
    )


def test_positive_and_negative_offsets_have_signed_only_changes():
    positive = compute_set(_params(DESIGN_A, offset=7.5))
    negative = compute_set(_params(DESIGN_A, offset=-7.5))

    odd = (
        "offset_angle",
        "pitch_plane_offset",
    )
    for field in odd:
        assert getattr(positive, field) == pytest.approx(-getattr(negative, field))
    for field in (
        "pinion_pitch_angle",
        "wheel_pitch_angle",
        "pinion_mean_radius",
        "wheel_mean_radius",
        "pinion_mean_cone_distance",
        "wheel_mean_cone_distance",
        "limit_radius_of_curvature",
    ):
        assert getattr(positive.method1, field) == pytest.approx(
            getattr(negative.method1, field), abs=1e-11
        )
    for name in ("pinion", "gear"):
        left, right = positive.member(name), negative.member(name)
        for field in (
            "pitch_angle",
            "pitch_radius",
            "cone_distance",
            "tooth_face_inner_cone_distance",
            "tooth_face_outer_cone_distance",
            "tooth_face_width",
            "outer_tip_radius",
            "outer_root_radius",
        ):
            assert getattr(left, field) == pytest.approx(getattr(right, field), abs=1e-11)
        for fraction in (0.0, 0.5, 1.0):
            distance = left.tooth_face_inner_cone_distance + fraction * (
                left.tooth_face_outer_cone_distance
                - left.tooth_face_inner_cone_distance
            )
            assert tooth_space_section(positive, name, distance).phase == pytest.approx(
                tooth_space_section(negative, name, distance).phase,
                abs=1e-11,
            )


def test_right_and_left_hand_geometry_is_a_mirror_with_same_magnitudes():
    right = compute_set(ANCHOR)
    left = compute_set(replace(ANCHOR, hand="left"))

    for name in ("pinion", "gear"):
        r, l = right.member(name), left.member(name)
        for field in (
            "pitch_angle",
            "pitch_radius",
            "cone_distance",
            "tooth_face_inner_cone_distance",
            "tooth_face_outer_cone_distance",
            "tooth_face_width",
            "generated_drive_normal_pressure_angle",
            "generated_coast_normal_pressure_angle",
        ):
            assert getattr(l, field) == pytest.approx(getattr(r, field), abs=1e-12)
        assert l.mean_spiral_angle == pytest.approx(-r.mean_spiral_angle, abs=1e-12)
        assert l.inner_spiral_angle == pytest.approx(-r.inner_spiral_angle, abs=1e-12)
        assert l.outer_spiral_angle == pytest.approx(-r.outer_spiral_angle, abs=1e-12)

        right_section = tooth_space_section(right, name)
        left_section = tooth_space_section(left, name)
        for right_flank, left_flank in (
            (right_section.drive_flank, left_section.drive_flank),
            (right_section.coast_flank, left_section.coast_flank),
        ):
            assert len(right_flank) == len(left_flank)
            for rp, lp in zip(right_flank, left_flank):
                assert lp[0] == pytest.approx(rp[0], abs=1e-12)
                assert lp[1] == pytest.approx(-rp[1], abs=1e-12)


@pytest.mark.parametrize("cutter_radius", [30.0, 60.0])
def test_cutter_radius_is_in_the_recomputed_method1_curvature_closure(cutter_radius):
    params = _params(DESIGN_A, cutter_radius=cutter_radius)
    geo = compute_set(params)
    expected = _recomputed_limit_radius_from_public_fields(geo)

    assert expected == pytest.approx(cutter_radius, abs=1e-8)
    assert geo.method1.limit_radius_of_curvature == pytest.approx(expected, abs=1e-8)


def test_cutter_radius_changes_pitch_solution_not_only_a_reported_field():
    small = compute_set(_params(DESIGN_A, cutter_radius=30.0))
    large = compute_set(_params(DESIGN_A, cutter_radius=60.0))

    assert small.pinion.pitch_angle != pytest.approx(large.pinion.pitch_angle)
    assert small.gear.pitch_angle != pytest.approx(large.gear.pitch_angle)
    assert small.pitch_plane_offset != pytest.approx(large.pitch_plane_offset)


def test_anchor_backlash_is_outer_transverse_and_is_converted_once():
    zero = compute_set(replace(ANCHOR, backlash=0.0))
    geo = compute_set(replace(ANCHOR, backlash=0.2))
    expected_mean_transverse = (
        0.2 * geo.gear.cone_distance / geo.gear.outer_cone_distance
    )
    expected_mean_normal = expected_mean_transverse * abs(
        math.cos(geo.gear.mean_spiral_angle)
    )
    expected_x = expected_mean_normal / (
        4.0 * geo.mean_normal_module * math.cos(ANCHOR.alpha)
    )

    assert geo.outer_transverse_backlash == pytest.approx(0.2)
    assert geo.mean_transverse_backlash == pytest.approx(expected_mean_transverse)
    assert geo.mean_normal_backlash == pytest.approx(expected_mean_normal)
    assert geo.thickness.backlash_thickness_modification == pytest.approx(expected_x)
    assert geo.pinion.x_sm - zero.pinion.x_sm == pytest.approx(-expected_x)
    assert geo.gear.x_sm - zero.gear.x_sm == pytest.approx(-expected_x)
    assert (
        geo.pinion.mean_normal_tooth_thickness
        + geo.gear.mean_normal_tooth_thickness
    ) < (
        zero.pinion.mean_normal_tooth_thickness
        + zero.gear.mean_normal_tooth_thickness
    )


@pytest.mark.parametrize("spec", OTHER_DESIGNS)
def test_non_anchor_backlash_uses_the_wheel_spiral_for_conversion(spec):
    params = _params(spec, backlash=0.2)
    geo = compute_set(params)
    expected_transverse = (
        params.backlash * geo.gear.cone_distance / geo.gear.outer_cone_distance
    )
    expected_normal = expected_transverse * abs(math.cos(geo.gear.mean_spiral_angle))

    assert geo.mean_transverse_backlash == pytest.approx(expected_transverse, abs=1e-12)
    assert geo.mean_normal_backlash == pytest.approx(expected_normal, abs=1e-12)
    assert geo.pinion.mean_transverse_tooth_thickness == pytest.approx(
        geo.pinion.mean_normal_tooth_thickness
        / abs(math.cos(geo.pinion.mean_spiral_angle)),
        abs=1e-12,
    )
    assert geo.gear.mean_transverse_tooth_thickness == pytest.approx(
        geo.gear.mean_normal_tooth_thickness
        / abs(math.cos(geo.gear.mean_spiral_angle)),
        abs=1e-12,
    )


def test_zero_backlash_leaves_the_method1_thickness_modification_unchanged():
    geo = compute_set(replace(ANCHOR, backlash=0.0))
    assert geo.outer_transverse_backlash == 0.0
    assert geo.mean_transverse_backlash == 0.0
    assert geo.mean_normal_backlash == 0.0
    assert geo.thickness.backlash_thickness_modification == 0.0


def test_excessive_backlash_is_rejected_by_physical_tooth_thickness():
    with pytest.raises(ValueError, match="tooth thickness"):
        compute_set(replace(ANCHOR, backlash=100.0))


def test_pitch_points_coincide_after_independent_skew_placement():
    geo = compute_set(ANCHOR)
    theta1, theta2 = contact_azimuths(geo)
    pinion_point = _world_contact_point(geo, "pinion", theta1)
    gear_point = _world_contact_point(geo, "gear", theta2)
    assert pinion_point == pytest.approx(gear_point, abs=1e-9)


@pytest.mark.parametrize("params", CONTACT_CASES)
def test_contact_azimuths_keep_pitch_contact_and_requested_axis_offset(params):
    """The contact solver constrains pitch surfaces, not tooth-trace tangents."""
    geo = compute_set(params)
    theta1, theta2 = contact_azimuths(geo)
    pinion_point = _world_contact_point(geo, "pinion", theta1)
    gear_point = _world_contact_point(geo, "gear", theta2)
    assert pinion_point == pytest.approx(gear_point, abs=1e-9)

    translation = gear_translation(geo)
    axis1 = (0.0, 0.0, 1.0)
    axis2 = (math.sin(geo.params.sigma), 0.0, math.cos(geo.params.sigma))
    assert skew_axis_distance(
        (0.0, 0.0, 0.0), axis1, translation, axis2
    ) == pytest.approx(
        abs(params.offset),
        # At zero offset the two axes intersect and the selected common
        # normal is a limiting direction; the closed-form placement retains
        # only a few micrometres of double-precision residual.
        abs=5e-6 if abs(params.offset) <= 1e-12 else 1e-9,
    )

    normal1, normal2 = _common_pitch_normals(geo)
    assert normal1 == pytest.approx(normal2, abs=1e-10)
    for member in ("pinion", "gear"):
        tangent = _pitch_trace_tangent_at_contact(geo, member)
        assert sum(tangent[i] * normal1[i] for i in range(3)) == pytest.approx(
            0.0, abs=2e-7
        )


@pytest.mark.parametrize("params", CONTACT_CASES)
def test_member_trace_tangents_are_independent_method1_directions(params):
    """Different trace angles are valid even though both lie in one tangent plane."""
    geo = compute_set(params)
    pinion_tangent = _pitch_trace_tangent_at_contact(geo, "pinion")
    gear_tangent = _pitch_trace_tangent_at_contact(geo, "gear")
    pinion_angle = _trace_angle_from_tangent(geo, "pinion", pinion_tangent)
    gear_angle = _trace_angle_from_tangent(geo, "gear", gear_tangent)

    assert pinion_angle == pytest.approx(geo.pinion.mean_spiral_angle, abs=2e-6)
    assert gear_angle == pytest.approx(-geo.gear.mean_spiral_angle, abs=2e-6)
    if abs(params.offset) > 1e-12:
        assert abs(abs(pinion_angle) - abs(gear_angle)) > math.radians(1.0)


def test_anchor_traces_are_not_forced_to_have_a_common_world_tangent():
    """The published pair's distinct Method 1 traces are not one tangent."""
    geo = compute_set(ANCHOR)
    pinion_tangent = _pitch_trace_tangent_at_contact(geo, "pinion")
    gear_tangent = _pitch_trace_tangent_at_contact(geo, "gear")
    assert pinion_tangent != pytest.approx(gear_tangent, abs=1e-3)


@pytest.mark.parametrize("params", CONTACT_CASES)
def test_hypoid_velocity_ratio_remains_the_tooth_count_ratio(params):
    geo = compute_set(params)
    expected = -params.z2 / params.z1
    assert angular_velocity_ratio(params.z1, params.z2) == pytest.approx(expected)
    assert geo.pinion.z * angular_velocity_ratio(
        geo.pinion.z, geo.gear.z
    ) + geo.gear.z == pytest.approx(0.0, abs=1e-12)
    assert gear_mate_ratio(geo.pinion.z, geo.gear.z) == pytest.approx(
        (float(geo.pinion.z), float(geo.gear.z)), abs=1e-12
    )


def test_shortest_distance_between_solved_shaft_axes_is_the_signed_offset_magnitude():
    geo = compute_set(ANCHOR)
    translation = gear_translation(geo)
    axis1 = (0.0, 0.0, 1.0)
    axis2 = (math.sin(geo.params.sigma), 0.0, math.cos(geo.params.sigma))
    assert skew_axis_distance(
        (0.0, 0.0, 0.0), axis1, translation, axis2
    ) == pytest.approx(abs(ANCHOR.offset), abs=1e-9)


@pytest.mark.parametrize("shaft_angle", [60.0, 90.0])
def test_velocity_ratio_is_derived_from_the_two_pitch_cones(shaft_angle):
    params = HypoidSetParams.with_defaults(
        2.0, 17, 43, offset=0.0, face_width=8.0, shaft_angle=shaft_angle
    )
    geo = compute_set(params)
    from_pitch_cones = math.cos(params.sigma) - math.sin(params.sigma) / math.tan(
        geo.pinion.pitch_angle
    )
    assert from_pitch_cones == pytest.approx(-params.z2 / params.z1, abs=1e-12)
    assert angular_velocity_ratio(params.z1, params.z2) == pytest.approx(
        from_pitch_cones, abs=1e-12
    )


def test_mean_cutter_trace_has_the_requested_tangent_at_each_member_calculation_point():
    geo = compute_set(ANCHOR)
    for member in (geo.pinion, geo.gear):
        radius = member.cone_distance
        cutter = ANCHOR.cutter_radius
        beta = abs(member.mean_spiral_angle)
        centre = math.sqrt(
            radius * radius + cutter * cutter
            - 2.0 * radius * cutter * math.sin(beta)
        )

        def theta_raw(value):
            cosine = (centre * centre + value * value - cutter * cutter) / (
                2.0 * centre * value
            )
            return math.acos(cosine)

        h = radius * 1e-6
        derivative = (theta_raw(radius + h) - theta_raw(radius - h)) / (2.0 * h)
        assert math.asin(
            (radius * radius + cutter * cutter - centre * centre)
            / (2.0 * radius * cutter)
        ) == pytest.approx(beta, abs=1e-12)
        assert radius * abs(derivative) == pytest.approx(
            math.tan(beta), rel=2e-6
        )


def test_pitch_surface_mesh_probe_has_no_obvious_pitch_cone_penetration():
    """Sample a ratio-driven mesh without claiming full flank conjugacy.

    The revolved pitch cones are rotationally invariant, so pair rotation does
    not move their common mean contact point.  The test samples that contact
    point and the tooth phase advance at 25 relative positions.  It catches a
    wrong rotation sign, wrong gear ratio, or a broken skew placement, but it
    deliberately does not pretend to replace solid interference analysis.
    """
    geo = compute_set(ANCHOR)
    theta1, theta2 = contact_azimuths(geo)
    base_p = _world_contact_point(geo, "pinion", theta1)
    base_g = _world_contact_point(geo, "gear", theta2)
    tau1 = 2.0 * math.pi / geo.pinion.z

    # A pitch cone has zero signed penetration when the angle between its
    # origin-to-point vector and its axis equals its solved pitch angle.
    # Tooth rotations do not change that macro surface, so this is the pure
    # geometry equivalent of a no-interference probe.  It intentionally does
    # not claim to measure the independently approximated flank solids.
    translation = gear_translation(geo)
    pinion_axis = (0.0, 0.0, 1.0)

    def cone_residual(point, origin, axis, angle):
        vector = tuple(point[i] - origin[i] for i in range(3))
        length = math.sqrt(sum(value * value for value in vector))
        cosine = sum(vector[i] * axis[i] for i in range(3)) / length
        return cosine - math.cos(angle)

    for index in range(25):
        pinion_rotation = tau1 * (index + 0.37) / 25.0
        gear_rotation = -geo.pinion.z * pinion_rotation / geo.gear.z
        assert geo.pinion.z * pinion_rotation + geo.gear.z * gear_rotation == pytest.approx(
            0.0, abs=1e-12
        )

        assert cone_residual(
            base_p, (0.0, 0.0, 0.0), pinion_axis, geo.pinion.pitch_angle
        ) == pytest.approx(0.0, abs=1e-12)
        gear_local_point = apply(
            rot_y(-geo.params.sigma),
            tuple(base_g[i] - translation[i] for i in range(3)),
        )
        assert cone_residual(
            gear_local_point,
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 1.0),
            geo.gear.pitch_angle,
        ) == pytest.approx(0.0, abs=1e-12)
        # Rotate each local contact ray backwards with its body rotation.  A
        # rotationally invariant pitch cone must still put that ray at the
        # same world point; the equation above separately checks the required
        # opposite tooth-phase advance.
        pinion_rotated = apply(
            rot_z(pinion_rotation),
            _local_contact_point(geo, "pinion", theta1 - pinion_rotation),
        )
        gear_rotated = apply(
            rot_y(geo.params.sigma),
            apply(
                rot_z(gear_rotation),
                _local_contact_point(geo, "gear", theta2 - gear_rotation),
            ),
        )
        gear_rotated = tuple(gear_rotated[i] + translation[i] for i in range(3))
        assert pinion_rotated == pytest.approx(base_p, abs=1e-9)
        assert gear_rotated == pytest.approx(base_g, abs=1e-9)
        assert base_p == pytest.approx(base_g, abs=1e-9)


SWEEP = (
    (2.0, 17, 43, 3.0, 8.0, 35.0, 30.0, 90.0),
    (2.0, 17, 43, -6.0, 8.0, 35.0, 30.0, 90.0),
    (2.5, 19, 47, 5.0, 12.0, 32.0, 30.0, 80.0),
    (2.5, 19, 47, -10.0, 12.0, 32.0, 45.0, 80.0),
    (2.2, 17, 41, 8.0, 10.0, 28.0, 38.0, 90.0),
    (3.0, 23, 59, -12.0, 14.0, 42.0, 50.0, 100.0),
    (1.8, 13, 37, 4.0, 7.0, 25.0, 28.0, 90.0),
    (2.8, 21, 55, -12.0, 13.0, 38.0, 80.0, 75.0),
)


@pytest.mark.parametrize(
    "module,z1,z2,offset,face_width,spiral,cutter,shaft_angle", SWEEP
)
def test_method1_property_sweep_has_finite_ordered_geometry(
    module, z1, z2, offset, face_width, spiral, cutter, shaft_angle
):
    params = HypoidSetParams.with_defaults(
        module,
        z1,
        z2,
        offset=offset,
        face_width=face_width,
        spiral_angle=spiral,
        cutter_radius=cutter,
        shaft_angle=shaft_angle,
    )
    geo = compute_set(params)

    assert geo.method1.limit_radius_of_curvature == pytest.approx(cutter, abs=1e-8)
    assert _finite_positive(
        geo.mean_normal_module,
        geo.pinion.pitch_radius,
        geo.gear.pitch_radius,
        geo.pinion.cone_distance,
        geo.gear.cone_distance,
    )
    for member in (geo.pinion, geo.gear):
        for value in vars(member).values():
            if isinstance(value, float):
                assert math.isfinite(value)
        assert _finite_positive(
            member.tooth_face_inner_cone_distance,
            member.tooth_face_outer_cone_distance,
            member.virtual_root_r,
            member.virtual_tip_r,
            member.outer_tip_radius,
            member.outer_root_radius,
            member.inner_tip_radius,
            member.inner_root_radius,
        )
        assert member.tooth_face_inner_cone_distance < member.cone_distance
        assert member.cone_distance < member.tooth_face_outer_cone_distance
        assert member.virtual_root_r < member.virtual_pitch_r < member.virtual_tip_r
        assert all(
            math.isfinite(value)
            for value in (
                member.face_angle,
                member.root_angle,
                member.inner_spiral_angle,
                member.outer_spiral_angle,
            )
        )
        bounds = section_cone_bounds(geo, member.name)
        distances = section_cone_distances(geo, member.name, 6)
        assert bounds.tooth_face_inner < bounds.tooth_face_outer
        assert distances == sorted(distances)
        assert distances[0] <= bounds.tooth_face_inner
        assert distances[-1] >= bounds.tooth_face_outer
        section = tooth_space_section(geo, member.name, member.cone_distance)
        assert all(
            math.isfinite(value)
            for point in section.loop_2d
            for value in point
        )
    for value in vars(geo.method1).values():
        if isinstance(value, float):
            assert math.isfinite(value)


def test_property_sweep_has_a_reproducible_nonconvergent_case():
    invalid = replace(ANCHOR, cutter_radius=1.0)
    with pytest.raises(ValueError, match="curvature"):
        compute_set(invalid)


def test_near_zero_offset_sweep_is_symmetric_and_reports_unsupported_closure():
    """Keep the zero branch and the first valid non-zero case explicit.

    The exact zero case is an intersecting bevel limit.  The current Method 1
    curvature closure has no positive-radius solution for this deliberately
    specified 1 mm/30 mm-cutter case; it must report that fact rather than
    silently fitting a discontinuous result.  The paired 3 mm cases verify
    the first valid non-zero points remain sign symmetric.
    """
    base = HypoidSetParams.with_defaults(
        2.0, 17, 43, offset=0.0, face_width=8.0,
        spiral_angle=35.0, cutter_radius=30.0,
    )
    zero = compute_set(base)
    with pytest.raises(ValueError, match="curvature"):
        compute_set(replace(base, offset=1.0))
    positive = compute_set(replace(base, offset=3.0))
    negative = compute_set(replace(base, offset=-3.0))
    assert positive.pinion.pitch_angle == pytest.approx(
        negative.pinion.pitch_angle, abs=1e-12
    )
    assert positive.pitch_plane_offset == pytest.approx(
        -negative.pitch_plane_offset, abs=1e-12
    )
    assert abs(positive.pinion.pitch_angle - zero.pinion.pitch_angle) < math.radians(5.0)
