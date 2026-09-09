"""Pure-Python hypoid acceptance and production-path tests.

The fixed 13/42 values below are external/reference checks.  Tests that
recompute ISO equations from returned fields are explicitly formula audits;
the contact, placement, trace-differentiation, hand, and boundary tests are
the independent physical invariants.  No test here treats the Tredgold loft
as a true generated or conjugate hypoid flank.
"""

import math
from dataclasses import replace

import pytest

from gears.__main__ import main
from gears.hypoid import preview
from gears.hypoid.geometry import (
    _cutter_trace,
    _hypoid_flank_points,
    _method1_spiral_angle_at,
    blank_outline,
    compute_set,
    normal_to_transverse_pressure_angle,
    section_cone_bounds,
    section_cone_distances,
    tooth_space_section,
)
from gears.hypoid.mesh import (
    contact_azimuths,
    contact_point,
    gear_clocking,
    gear_contact_point,
    gear_translation,
    skew_axis_distance,
)
from gears.hypoid.params import HypoidSetParams
from gears.hypoid.validate import validate
from gears.bevel.geometry import to_cone_3d
from gears.placement import apply, rot_y


ANCHOR = HypoidSetParams.with_defaults(
    170.0 / 42.0, 13, 42,
    offset=15.0, face_width=30.0, spiral_angle=50.0, cutter_radius=63.5,
)
ANCHOR_BACKLASH = HypoidSetParams(
    **{**ANCHOR.__dict__, "backlash": 0.2}
)


def test_method_1_anchor_converges_to_published_macro_geometry():
    geo = compute_set(ANCHOR)
    assert geo.pinion.pitch_angle_deg == pytest.approx(21.288, abs=0.02)
    assert geo.gear.pitch_angle_deg == pytest.approx(68.324, abs=0.02)
    assert geo.offset_angle_deg == pytest.approx(11.39, abs=0.03)
    assert geo.pitch_plane_offset == pytest.approx(15.075, abs=0.002)
    assert geo.pinion.mean_spiral_angle_deg == pytest.approx(50.0, abs=0.01)
    assert geo.gear.mean_spiral_angle_deg == pytest.approx(38.61, abs=0.03)
    assert geo.mean_normal_module == pytest.approx(2.64, abs=0.01)
    assert (
        geo.pinion.normal_tooth_thickness + geo.gear.normal_tooth_thickness
    ) == pytest.approx(math.pi * geo.mean_normal_module - ANCHOR.backlash)


def test_method_1_anchor_exposes_distinct_generated_normal_flank_angles():
    geo = compute_set(ANCHOR)
    assert geo.method1.generated_drive_normal_pressure_angle == pytest.approx(
        ANCHOR.alpha + geo.method1.limit_pressure_angle, abs=1e-12
    )
    assert geo.method1.generated_coast_normal_pressure_angle == pytest.approx(
        ANCHOR.alpha - geo.method1.limit_pressure_angle, abs=1e-12
    )
    assert math.degrees(
        geo.method1.generated_drive_normal_pressure_angle
    ) == pytest.approx(17.746, abs=0.01)
    assert math.degrees(
        geo.method1.generated_coast_normal_pressure_angle
    ) == pytest.approx(22.254, abs=0.01)
    for member in (geo.pinion, geo.gear):
        assert member.generated_drive_normal_pressure_angle == pytest.approx(
            geo.method1.generated_drive_normal_pressure_angle, abs=1e-12
        )
        assert member.generated_coast_normal_pressure_angle == pytest.approx(
            geo.method1.generated_coast_normal_pressure_angle, abs=1e-12
        )


NON_ANCHOR_SECTION = HypoidSetParams.with_defaults(
    2.0, 17, 43, offset=6.0, face_width=8.0,
    spiral_angle=35.0, cutter_radius=30.0,
)


@pytest.mark.parametrize(
    "params",
    [ANCHOR, NON_ANCHOR_SECTION],
    ids=["anchor", "non_anchor"],
)
def test_mean_tredgold_section_converts_both_members_and_flanks_to_transverse(
    params,
):
    geo = compute_set(params)
    for name in ("pinion", "gear"):
        member = geo.member(name)
        section = tooth_space_section(geo, name, member.cone_distance)
        assert section.spiral_angle == pytest.approx(member.mean_spiral_angle, abs=1e-12)
        for _, normal_angle, transverse_angle, base_radius in (
            (
                "drive",
                member.generated_drive_normal_pressure_angle,
                member.generated_drive_transverse_pressure_angle,
                section.drive_base_radius,
            ),
            (
                "coast",
                member.generated_coast_normal_pressure_angle,
                member.generated_coast_transverse_pressure_angle,
                section.coast_base_radius,
            ),
        ):
            expected_angle = normal_to_transverse_pressure_angle(
                normal_angle, member.mean_spiral_angle
            )
            assert transverse_angle == pytest.approx(expected_angle, abs=1e-12)
            assert base_radius == pytest.approx(
                member.virtual_pitch_r * math.cos(expected_angle), abs=1e-12
            )
            flank = _hypoid_flank_points(
                member,
                member.cone_distance,
                normal_angle,
                member.mean_spiral_angle,
                member.mean_normal_tooth_thickness,
                1001,
            )
            tooth_half_angle = (
                member.mean_transverse_tooth_thickness
                / (2.0 * member.virtual_pitch_r)
            )
            half_pitch = math.pi / member.virtual_teeth
            for x, y in flank[::100]:
                radius = math.hypot(x, y)
                if radius < base_radius - 1e-12:
                    continue  # below-base radial approximation
                roll_angle = math.acos(min(1.0, base_radius / radius))
                expected_space_angle = (
                    half_pitch - tooth_half_angle
                    - (math.tan(expected_angle) - expected_angle)
                    + (math.tan(roll_angle) - roll_angle)
                )
                assert math.atan2(y, x) == pytest.approx(
                    expected_space_angle, abs=1e-12
                )


@pytest.mark.parametrize("normal_angle", [math.radians(17.5), math.radians(22.5)])
def test_normal_to_transverse_pressure_angle_has_zero_and_monotone_spiral_limits(
    normal_angle,
):
    assert normal_to_transverse_pressure_angle(normal_angle, 0.0) == pytest.approx(
        normal_angle, abs=1e-12
    )
    straight = normal_to_transverse_pressure_angle(normal_angle, math.radians(20.0))
    spiral = normal_to_transverse_pressure_angle(normal_angle, math.radians(50.0))
    assert abs(spiral) > abs(straight) > abs(normal_angle)


def test_zero_spiral_hypoid_sections_use_the_generated_normal_angles_directly():
    geo = compute_set(replace(ANCHOR, offset=0.0, spiral_angle=0.0))
    for name in ("pinion", "gear"):
        member = geo.member(name)
        section = tooth_space_section(geo, name, member.cone_distance)
        assert section.spiral_angle == pytest.approx(0.0, abs=1e-12)
        assert section.drive_transverse_pressure_angle == pytest.approx(
            member.generated_drive_normal_pressure_angle, abs=1e-12
        )
        assert section.coast_transverse_pressure_angle == pytest.approx(
            member.generated_coast_normal_pressure_angle, abs=1e-12
        )


def test_developed_back_cone_angular_pitch_maps_to_the_physical_tooth_spacing():
    geo = compute_set(ANCHOR)
    for member in (geo.pinion, geo.gear):
        developed_half_pitch = math.pi / member.virtual_teeth
        physical_spacing = 2.0 * developed_half_pitch / math.cos(
            member.pitch_angle
        )
        assert physical_spacing == pytest.approx(2.0 * math.pi / member.z, abs=1e-12)


def test_method_1_anchor_depths_and_angles_recompute_method1_equations():
    geo = compute_set(ANCHOR)
    x_hm1 = ANCHOR.depth_factor * (0.5 - ANCHOR.gear_mean_addendum_factor)
    k_hap = 0.5 * ANCHOR.depth_factor
    k_hfp = 0.5 * ANCHOR.depth_factor + 2.0 * ANCHOR.clearance_factor
    mn = geo.mean_normal_module

    assert geo.profile_shift_coefficient == pytest.approx(x_hm1, abs=1e-12)
    assert geo.basic_addendum_factor == pytest.approx(k_hap, abs=1e-12)
    assert geo.basic_dedendum_factor == pytest.approx(k_hfp, abs=1e-12)
    assert geo.working_depth == pytest.approx(2.0 * k_hap * mn, abs=1e-12)
    assert geo.clearance == pytest.approx((k_hfp - k_hap) * mn, abs=1e-12)
    assert geo.whole_depth == pytest.approx((k_hap + k_hfp) * mn, abs=1e-12)
    assert geo.pinion.addendum == pytest.approx(mn * (k_hap + x_hm1), abs=1e-12)
    assert geo.pinion.dedendum == pytest.approx(mn * (k_hfp - x_hm1), abs=1e-12)
    assert geo.gear.addendum == pytest.approx(mn * (k_hap - x_hm1), abs=1e-12)
    assert geo.gear.dedendum == pytest.approx(mn * (k_hfp + x_hm1), abs=1e-12)

    assert geo.pinion.addendum == pytest.approx(3.432, abs=0.002)
    assert geo.pinion.dedendum == pytest.approx(2.508, abs=0.002)
    assert geo.gear.addendum == pytest.approx(1.848, abs=0.002)
    assert geo.gear.dedendum == pytest.approx(4.092, abs=0.002)
    assert geo.pinion.face_angle_deg == pytest.approx(25.231, abs=0.002)
    assert geo.pinion.root_angle_deg == pytest.approx(20.302, abs=0.002)
    assert geo.gear.face_angle_deg == pytest.approx(69.324, abs=0.002)
    assert geo.gear.root_angle_deg == pytest.approx(64.324, abs=0.002)

    for member in (geo.pinion, geo.gear):
        assert member.face_angle == pytest.approx(
            member.pitch_angle + member.addendum_angle, abs=1e-12
        )
        assert member.root_angle == pytest.approx(
            member.pitch_angle - member.dedendum_angle, abs=1e-12
        )
        assert member.outer_addendum == pytest.approx(
            member.addendum + member.outer_face_width * math.tan(member.addendum_angle),
            abs=1e-12,
        )
        assert member.inner_dedendum == pytest.approx(
            member.dedendum - member.inner_face_width * math.tan(member.dedendum_angle),
            abs=1e-12,
        )
        assert member.outer_tip_diameter == pytest.approx(
            member.outer_pitch_diameter
            + 2.0 * member.outer_addendum * math.cos(member.pitch_angle),
            abs=1e-12,
        )
        assert member.outer_root_diameter == pytest.approx(
            member.outer_pitch_diameter
            - 2.0 * member.outer_dedendum * math.cos(member.pitch_angle),
            abs=1e-12,
        )

    # ISO 23509 Method 1 formula 160 gives b_reri1.  It is distinct from the
    # physical pitch-cone span b_e1+b_i1 used by the tooth boundaries.
    assert geo.pinion.face_width == pytest.approx(30.470, abs=0.002)
    assert geo.pinion.face_width_along_pitch_cone == pytest.approx(31.910, abs=0.002)
    assert geo.pinion.tooth_face_width == pytest.approx(
        geo.pinion.face_width_along_pitch_cone, abs=1e-12
    )
    assert geo.gear.face_width == pytest.approx(30.0, abs=1e-12)
    assert geo.gear.face_width_along_pitch_cone == pytest.approx(30.0, abs=1e-12)
    method = geo.method1
    assert method.pinion_face_width == pytest.approx(30.470, abs=0.002)
    assert method.pinion_face_width_increment_along_axis == pytest.approx(
        0.653, abs=0.002
    )
    assert method.wheel_face_width_factor == pytest.approx(
        geo.gear.outer_face_width / geo.gear.face_width, abs=1e-12
    )
    assert method.pinion_outer_face_width == pytest.approx(
        geo.pinion.outer_face_width, abs=1e-12
    )
    assert method.pinion_inner_face_width == pytest.approx(
        geo.pinion.inner_face_width, abs=1e-12
    )
    assert method.wheel_face_apex_z == pytest.approx(geo.gear.face_apex_z, abs=1e-12)
    assert method.pinion_root_apex_z == pytest.approx(geo.pinion.root_apex_z, abs=1e-12)


def test_method_1_anchor_stores_fixed_and_derived_longitudinal_face_boundaries():
    geo = compute_set(ANCHOR)
    pinion, gear = geo.pinion, geo.gear

    assert pinion.tooth_face_inner_cone_distance == pytest.approx(57.6995, abs=0.002)
    assert pinion.tooth_face_outer_cone_distance == pytest.approx(89.6097, abs=0.002)
    assert gear.tooth_face_inner_cone_distance == pytest.approx(61.4680, abs=0.002)
    assert gear.tooth_face_outer_cone_distance == pytest.approx(91.4680, abs=0.002)
    assert pinion.face_width == pytest.approx(30.470, abs=0.002)
    assert pinion.face_width_along_pitch_cone == pytest.approx(31.910, abs=0.002)
    assert pinion.tooth_face_width == pytest.approx(
        pinion.face_width_along_pitch_cone, abs=1e-12
    )
    assert gear.face_width == pytest.approx(30.000, abs=1e-12)
    assert gear.face_width_along_pitch_cone == pytest.approx(30.000, abs=1e-12)
    assert pinion.face_width != pytest.approx(pinion.face_width_along_pitch_cone)

    method = geo.method1
    assert method.pinion_boundary_wheel_inner_cone_distance == pytest.approx(
        60.9067, abs=0.002
    )
    assert method.pinion_boundary_wheel_outer_cone_distance == pytest.approx(
        92.1631, abs=0.002
    )
    zeta_mp = abs(method.pinion_offset_angle_pitch)
    expected_re21 = math.sqrt(
        gear.cone_distance ** 2
        + pinion.outer_face_width ** 2
        + 2.0 * gear.cone_distance * pinion.outer_face_width * math.cos(zeta_mp)
    )
    expected_ri21 = math.sqrt(
        gear.cone_distance ** 2
        + pinion.inner_face_width ** 2
        - 2.0 * gear.cone_distance * pinion.inner_face_width * math.cos(zeta_mp)
    )
    assert method.pinion_boundary_wheel_outer_cone_distance == pytest.approx(
        expected_re21, abs=1e-12
    )
    assert method.pinion_boundary_wheel_inner_cone_distance == pytest.approx(
        expected_ri21, abs=1e-12
    )
    wheel_trace = _cutter_trace(gear, geo)
    assert pinion.outer_spiral_angle == pytest.approx(
        wheel_trace.spiral_angle_at(expected_re21)
        + math.asin(abs(geo.pitch_plane_offset) / expected_re21),
        abs=1e-12,
    )
    assert pinion.inner_spiral_angle == pytest.approx(
        wheel_trace.spiral_angle_at(expected_ri21)
        + math.asin(abs(geo.pitch_plane_offset) / expected_ri21),
        abs=1e-12,
    )
    assert math.degrees(pinion.inner_spiral_angle) == pytest.approx(44.881, abs=0.01)
    assert math.degrees(pinion.outer_spiral_angle) == pytest.approx(57.545, abs=0.01)
    assert math.degrees(gear.inner_spiral_angle) == pytest.approx(30.828, abs=0.01)
    assert math.degrees(gear.outer_spiral_angle) == pytest.approx(47.676, abs=0.01)

    for member in (pinion, gear):
        assert member.tooth_face_inner_cone_distance == pytest.approx(
            member.cone_distance - member.inner_face_width, abs=1e-12
        )
        assert member.tooth_face_outer_cone_distance == pytest.approx(
            member.cone_distance + member.outer_face_width, abs=1e-12
        )
        trace = _cutter_trace(member, geo)
        assert trace.mean_cone_dist == pytest.approx(member.cone_distance, abs=1e-12)
        assert trace.spiral_angle_at(member.cone_distance) == pytest.approx(
            member.mean_spiral_angle, abs=1e-12
        )


def test_longitudinal_face_limits_are_separate_from_loft_extensions():
    geo = compute_set(ANCHOR)
    for name in ("pinion", "gear"):
        member = geo.member(name)
        bounds = section_cone_bounds(geo, name)
        distances = section_cone_distances(geo, name)
        assert bounds.calculation_point == pytest.approx(member.cone_distance, abs=1e-12)
        assert bounds.tooth_face_inner == pytest.approx(
            member.tooth_face_inner_cone_distance, abs=1e-12
        )
        assert bounds.tooth_face_outer == pytest.approx(
            member.tooth_face_outer_cone_distance, abs=1e-12
        )
        assert bounds.tooth_face_inner <= bounds.calculation_point <= bounds.tooth_face_outer
        assert distances[0] == pytest.approx(bounds.loft_inner, abs=1e-11)
        assert distances[-1] == pytest.approx(bounds.loft_outer, abs=1e-11)
        assert any(
            value == pytest.approx(bounds.tooth_face_inner, abs=1e-11)
            for value in distances
        )
        assert any(
            value == pytest.approx(bounds.tooth_face_outer, abs=1e-11)
            for value in distances
        )
        assert any(
            value == pytest.approx(bounds.calculation_point, abs=1e-11)
            for value in distances
        )
        assert bounds.is_loft_extension(distances[0]) or bounds.is_loft_extension(distances[-1])


@pytest.mark.parametrize(
    "offset,cutter_radius",
    [(5.0, 40.0), (-10.0, 120.0)],
)
def test_longitudinal_boundaries_follow_method_1_for_other_offsets_and_cutters(
    offset, cutter_radius
):
    params = HypoidSetParams(**{
        **ANCHOR.__dict__,
        "offset": offset,
        "cutter_radius": cutter_radius,
    })
    geo = compute_set(params)
    assert geo.pinion.tooth_face_width != pytest.approx(geo.gear.tooth_face_width)
    assert geo.pinion.tooth_face_width == pytest.approx(
        geo.pinion.tooth_face_outer_cone_distance
        - geo.pinion.tooth_face_inner_cone_distance,
        abs=1e-12,
    )
    assert geo.gear.tooth_face_width == pytest.approx(
        geo.gear.tooth_face_outer_cone_distance
        - geo.gear.tooth_face_inner_cone_distance,
        abs=1e-12,
    )
    for member in (geo.pinion, geo.gear):
        trace = _cutter_trace(member, geo)
        assert trace.cutter_radius == pytest.approx(cutter_radius, abs=1e-12)
        assert trace.mean_cone_dist == pytest.approx(member.cone_distance, abs=1e-12)
        bounds = section_cone_bounds(geo, member.name)
        assert bounds.calculation_point == pytest.approx(member.cone_distance, abs=1e-12)
        assert bounds.tooth_face_inner <= bounds.calculation_point <= bounds.tooth_face_outer


def test_cutter_radius_changes_longitudinal_geometry_at_fixed_offset():
    smaller = compute_set(
        HypoidSetParams(**{**ANCHOR.__dict__, "cutter_radius": 40.0})
    )
    larger = compute_set(
        HypoidSetParams(**{**ANCHOR.__dict__, "cutter_radius": 120.0})
    )
    for name in ("pinion", "gear"):
        small, large = smaller.member(name), larger.member(name)
        assert small.tooth_face_inner_cone_distance != pytest.approx(
            large.tooth_face_inner_cone_distance
        )
        assert small.tooth_face_outer_cone_distance != pytest.approx(
            large.tooth_face_outer_cone_distance
        )


def test_method_1_anchor_outer_transverse_backlash_is_converted_once():
    geo = compute_set(ANCHOR_BACKLASH)
    p = ANCHOR_BACKLASH
    thickness = geo.thickness
    mn = geo.mean_normal_module
    alpha_n = p.alpha
    expected_mean_transverse = (
        p.backlash * geo.gear.cone_distance / geo.gear.outer_cone_distance
    )
    expected_mean_normal = expected_mean_transverse * abs(
        math.cos(geo.gear.mean_spiral_angle)
    )
    expected_backlash_x = expected_mean_normal / (
        4.0 * mn * math.cos(alpha_n)
    )
    x_smn = 0.5 * p.thickness_factor

    assert p.outer_transverse_backlash == pytest.approx(0.2)
    assert thickness.outer_transverse_backlash == pytest.approx(0.2)
    assert thickness.mean_transverse_backlash == pytest.approx(
        expected_mean_transverse, abs=1e-12
    )
    assert thickness.mean_normal_backlash == pytest.approx(
        expected_mean_normal, abs=1e-12
    )
    assert thickness.backlash_thickness_modification == pytest.approx(
        expected_backlash_x, abs=1e-12
    )
    assert geo.pinion.x_sm == pytest.approx(x_smn - expected_backlash_x, abs=1e-12)
    assert geo.gear.x_sm == pytest.approx(-x_smn - expected_backlash_x, abs=1e-12)
    # These are the published coefficient values to the stated precision.
    assert geo.pinion.x_sm == pytest.approx(0.038, abs=0.002)
    assert geo.gear.x_sm == pytest.approx(-0.062, abs=0.002)

    expected_pinion = 0.5 * mn * (
        math.pi + 2.0 * (geo.pinion.x_sm + geo.profile_shift_coefficient)
        * math.tan(alpha_n)
    )
    expected_gear = 0.5 * mn * (
        math.pi + 2.0 * (geo.gear.x_sm - geo.profile_shift_coefficient)
        * math.tan(alpha_n)
    )
    assert geo.pinion.mean_normal_tooth_thickness == pytest.approx(
        expected_pinion, abs=1e-12
    )
    assert geo.gear.mean_normal_tooth_thickness == pytest.approx(
        expected_gear, abs=1e-12
    )
    assert geo.pinion.mean_transverse_tooth_thickness == pytest.approx(
        expected_pinion / abs(math.cos(geo.pinion.mean_spiral_angle)), abs=1e-12
    )
    assert geo.gear.mean_transverse_tooth_thickness == pytest.approx(
        expected_gear / abs(math.cos(geo.gear.mean_spiral_angle)), abs=1e-12
    )


def test_hypoid_zero_backlash_has_no_backlash_thickness_correction():
    geo = compute_set(ANCHOR)
    assert geo.outer_transverse_backlash == 0.0
    assert geo.mean_transverse_backlash == 0.0
    assert geo.mean_normal_backlash == 0.0
    assert geo.thickness.backlash_thickness_modification == 0.0
    assert geo.pinion.x_sm == pytest.approx(0.5 * ANCHOR.thickness_factor)
    assert geo.gear.x_sm == pytest.approx(-0.5 * ANCHOR.thickness_factor)
    assert (
        geo.pinion.mean_normal_tooth_thickness
        + geo.gear.mean_normal_tooth_thickness
    ) == pytest.approx(math.pi * geo.mean_normal_module, abs=1e-12)


def _recomputed_limit_radius_from_public_fields(geo):
    """Re-evaluate ISO 23509 formulas 32 and 33 from public result fields."""
    a, b, method = geo.pinion, geo.gear, geo.method1
    beta1 = abs(a.mean_spiral_angle)
    beta2 = abs(b.mean_spiral_angle)
    zeta = abs(method.pinion_offset_angle_pitch)
    alpha_lim = math.atan(
        -math.tan(a.pitch_angle) * math.tan(b.pitch_angle)
        * (
            a.cone_distance * math.sin(beta1)
            - b.cone_distance * math.sin(beta2)
        )
        / (
            math.cos(zeta)
            * (
                a.cone_distance * math.tan(a.pitch_angle)
                + b.cone_distance * math.tan(b.pitch_angle)
            )
        )
    )
    denominator = (
        -math.tan(alpha_lim)
        * (
            math.tan(beta1) / (a.cone_distance * math.tan(a.pitch_angle))
            + math.tan(beta2) / (b.cone_distance * math.tan(b.pitch_angle))
        )
        + 1.0 / (a.cone_distance * math.cos(beta1))
        - 1.0 / (b.cone_distance * math.cos(beta2))
    )
    return alpha_lim, (
        1.0 / math.cos(alpha_lim)
        * (math.tan(beta1) - math.tan(beta2))
        / denominator
    )


def test_method_1_public_fields_close_recomputed_pitch_and_curvature_equations():
    geo = compute_set(ANCHOR)
    method = geo.method1
    assert (
        geo.gear.pitch_radius * math.cos(abs(geo.gear.mean_spiral_angle))
        / (geo.pinion.pitch_radius * math.cos(abs(geo.pinion.mean_spiral_angle)))
    ) == pytest.approx(ANCHOR.ratio, abs=1e-12)
    assert method.pitch_plane_offset == pytest.approx(
        method.wheel_mean_cone_distance
        * math.sin(method.pinion_offset_angle_pitch),
        abs=1e-12,
    )
    assert geo.mean_normal_module == pytest.approx(
        2.0 * method.wheel_mean_cone_distance
        * math.sin(method.wheel_pitch_angle)
        * math.cos(abs(geo.gear.mean_spiral_angle))
        / ANCHOR.z2,
        abs=1e-12,
    )
    alpha_lim, rho_lim = _recomputed_limit_radius_from_public_fields(geo)
    assert method.limit_pressure_angle == pytest.approx(alpha_lim, abs=1e-12)
    assert method.limit_radius_of_curvature == pytest.approx(rho_lim, abs=1e-9)
    assert method.limit_radius_of_curvature == pytest.approx(ANCHOR.cutter_radius, abs=1e-8)
    assert abs(method.curvature_residual) < 1e-8
    assert method.pinion_mean_cone_distance == pytest.approx(73.521, abs=0.002)
    assert method.wheel_mean_cone_distance == pytest.approx(76.337, abs=0.002)


@pytest.mark.parametrize(
    "module,z1,z2,offset,face_width,spiral,cutter,shaft_angle",
    [
        (2.0, 17, 43, 6.0, 8.0, 35.0, 30.0, 90.0),
        (3.0, 19, 57, -10.0, 12.0, 35.0, 50.0, 90.0),
        (2.0, 17, 43, 5.0, 6.0, 35.0, 25.0, 60.0),
    ],
)
def test_method_1_closes_non_anchor_valid_designs(
    module, z1, z2, offset, face_width, spiral, cutter, shaft_angle
):
    params = HypoidSetParams.with_defaults(
        module, z1, z2, offset=offset, face_width=face_width,
        spiral_angle=spiral, cutter_radius=cutter, shaft_angle=shaft_angle,
    )
    geo = compute_set(params)
    assert geo.method1.limit_radius_of_curvature == pytest.approx(cutter, abs=1e-8)
    assert geo.pinion.pitch_angle + geo.gear.pitch_angle < params.sigma + math.pi / 2.0
    assert validate(params).ok


def test_zero_offset_transverse_thicknesses_close_one_pitch_and_backlash_once():
    base = HypoidSetParams.with_defaults(2.0, 17, 43, offset=0.0, backlash=0.0)
    geo = compute_set(base)
    transverse_pitch = 2.0 * math.pi * geo.pinion.pitch_radius / geo.pinion.z
    assert (
        geo.pinion.transverse_tooth_thickness
        + geo.gear.transverse_tooth_thickness
    ) == pytest.approx(transverse_pitch, abs=1e-9)

    with_backlash = compute_set(
        HypoidSetParams(**{**base.__dict__, "backlash": 0.2})
    )
    normal_sum = (
        with_backlash.pinion.mean_normal_tooth_thickness
        + with_backlash.gear.mean_normal_tooth_thickness
    )
    expected_loss = (
        2.0 * with_backlash.mean_normal_module
        * with_backlash.thickness.backlash_thickness_modification
        * math.tan(with_backlash.thickness.mean_normal_pressure_angle)
    )
    assert normal_sum == pytest.approx(
        math.pi * with_backlash.mean_normal_module - expected_loss, abs=1e-9
    )
    assert with_backlash.mean_transverse_backlash == pytest.approx(
        0.2 * with_backlash.gear.cone_distance
        / with_backlash.gear.outer_cone_distance,
        abs=1e-12,
    )


def test_offset_is_signed_but_axis_distance_is_positive():
    positive = compute_set(ANCHOR)
    negative = compute_set(HypoidSetParams(**{**ANCHOR.__dict__, "offset": -15.0}))
    assert positive.offset_angle == pytest.approx(-negative.offset_angle)
    assert positive.pitch_plane_offset == pytest.approx(-negative.pitch_plane_offset)
    assert positive.pinion.pitch_angle == pytest.approx(negative.pinion.pitch_angle)
    assert positive.gear.pitch_angle == pytest.approx(negative.gear.pitch_angle)
    assert positive.method1.limit_radius_of_curvature == pytest.approx(
        negative.method1.limit_radius_of_curvature
    )
    assert positive.method1.wheel_offset_angle_axial == pytest.approx(
        -negative.method1.wheel_offset_angle_axial
    )
    for member in ("pinion", "gear"):
        a, b = positive.member(member), negative.member(member)
        assert a.face_width == pytest.approx(b.face_width, abs=1e-12)
        assert a.outer_face_width == pytest.approx(b.outer_face_width, abs=1e-12)
        assert a.inner_face_width == pytest.approx(b.inner_face_width, abs=1e-12)
        assert a.outside_dia == pytest.approx(b.outside_dia, abs=1e-12)
        assert a.outer_root_diameter == pytest.approx(b.outer_root_diameter, abs=1e-12)
        assert a.outer_tip_z == pytest.approx(b.outer_tip_z, abs=1e-12)
    assert gear_translation(positive)[1] == pytest.approx(15.0)
    assert gear_translation(negative)[1] == pytest.approx(-15.0)


def test_zero_offset_reduces_to_intersecting_pitch_cones():
    p = HypoidSetParams.with_defaults(2.0, 17, 43, offset=0.0)
    geo = compute_set(p)
    assert geo.offset_angle == 0.0
    assert geo.pinion.pitch_angle + geo.gear.pitch_angle == pytest.approx(p.sigma)
    assert geo.pinion.face_width == pytest.approx(geo.gear.face_width, abs=1e-12)
    assert geo.pinion.outer_face_width == pytest.approx(geo.gear.outer_face_width, abs=1e-12)
    assert geo.pinion.inner_face_width == pytest.approx(geo.gear.inner_face_width, abs=1e-12)
    changed_cutter = compute_set(
        HypoidSetParams(**{**p.__dict__, "cutter_radius": 100.0})
    )
    assert changed_cutter.pinion.pitch_angle == pytest.approx(geo.pinion.pitch_angle, abs=1e-12)
    assert changed_cutter.gear.pitch_angle == pytest.approx(geo.gear.pitch_angle, abs=1e-12)
    assert geo.method1.limit_radius_of_curvature is None
    assert validate(p).ok


def test_sections_are_closed_and_cover_both_face_ends():
    geo = compute_set(ANCHOR)
    for member in ("pinion", "gear"):
        distances = section_cone_distances(geo, member)
        assert len(distances) >= 2
        section = tooth_space_section(geo, member, distances[0], split_cap=True)
        assert len(section.loop_2d) > 20
        assert "cap_neg" in section.segments
        assert "cap_pos" in section.segments
        assert len(section.loop_3d()) == len(section.loop_2d)
        assert section.cone_apex_z == pytest.approx(
            section.cone_dist / math.cos(geo.member(member).pitch_angle)
        )
        inner = tooth_space_section(geo, member, distances[0])
        outer = tooth_space_section(geo, member, distances[-1])
        outline = blank_outline(geo, member)
        i_back = 4 if geo.params.min_root_thickness > 0.0 else 3
        assert max(z for _, _, z in inner.loop_3d()) < outline[0][1]
        assert min(z for _, _, z in outer.loop_3d()) > outline[i_back][1]


def test_production_loft_guide_uses_the_split_cap_vertex_for_every_station():
    """The pure-Python station/guide contract matches the SOLIDWORKS path.

    ``gears.sw.hypoid_part._guide_points`` samples exactly this
    ``tooth_space_section(..., split_cap=True)`` cap vertex and maps it with
    ``to_cone_3d``.  Keeping the assertion here avoids importing COM-bound
    SOLIDWORKS code while checking that physical and loft-only stations use
    the same production section geometry.
    """
    geo = compute_set(ANCHOR)
    for member in ("pinion", "gear"):
        for distance in section_cone_distances(geo, member):
            section = tooth_space_section(geo, member, distance, split_cap=True)
            cap = (section.r_cap, 0.0)
            assert section.segments["cap_neg"][-1] == pytest.approx(cap, abs=1e-12)
            assert section.segments["cap_pos"][0] == pytest.approx(cap, abs=1e-12)
            mapped = to_cone_3d(
                cap[0], cap[1], section.pitch_angle,
                section.cone_apex_z, section.phase,
            )
            assert mapped == pytest.approx(
                to_cone_3d(
                    section.segments["cap_pos"][0][0],
                    section.segments["cap_pos"][0][1],
                    section.pitch_angle,
                    section.cone_apex_z,
                    section.phase,
                ),
                abs=1e-12,
            )


def test_anchor_drive_and_coast_flanks_are_independently_generated():
    geo = compute_set(ANCHOR)
    section = tooth_space_section(geo, "pinion", geo.pinion.cone_distance)
    assert section.drive_flank
    assert section.coast_flank
    # A mirrored copy of the drive curve would have the same positive-y
    # magnitude at every corresponding point.  The generated normal angles
    # deliberately make the two involutes different.
    assert section.drive_flank[-1][1] != pytest.approx(
        -section.coast_flank[-1][1], abs=1e-6
    )


def test_independent_hypoid_flanks_preserve_mean_tooth_space_position():
    geo = compute_set(ANCHOR)
    member = geo.pinion
    section = tooth_space_section(
        geo, "pinion", member.cone_distance, n_flank=1001
    )
    half_pitch = math.pi / member.virtual_teeth
    tooth_half_angle = (
        member.mean_transverse_tooth_thickness / (2.0 * member.virtual_pitch_r)
    )
    expected_space_angle = half_pitch - tooth_half_angle

    for flank, pressure_angle in (
        (section.drive_flank, section.drive_transverse_pressure_angle),
        (section.coast_flank, section.coast_transverse_pressure_angle),
    ):
        pitch_point = min(
            flank,
            key=lambda point: abs(math.hypot(*point) - member.virtual_pitch_r),
        )
        actual_angle = math.atan2(pitch_point[1], pitch_point[0])
        assert abs(actual_angle) == pytest.approx(expected_space_angle, abs=2e-4)
        assert 0.0 < pressure_angle < math.pi / 2.0


@pytest.mark.parametrize(
    "params",
    [ANCHOR, NON_ANCHOR_SECTION],
    ids=["anchor", "non_anchor"],
)
def test_face_boundary_sections_use_local_method1_beta_for_angle_and_thickness(
    params,
):
    geo = compute_set(params)
    for name in ("pinion", "gear"):
        member = geo.member(name)
        for cone_dist in (
            member.tooth_face_inner_cone_distance,
            member.tooth_face_outer_cone_distance,
        ):
            section = tooth_space_section(geo, name, cone_dist, n_flank=1001)
            beta = _method1_spiral_angle_at(member, cone_dist, geo)
            scale = cone_dist / member.cone_distance
            expected_transverse_thickness = (
                member.mean_transverse_tooth_thickness * scale
            )
            assert section.spiral_angle == pytest.approx(beta, abs=1e-12)
            assert section.transverse_tooth_thickness == pytest.approx(
                expected_transverse_thickness, abs=1e-12
            )
            assert section.normal_tooth_thickness == pytest.approx(
                expected_transverse_thickness * abs(math.cos(beta)), abs=1e-12
            )
            assert section.drive_transverse_pressure_angle == pytest.approx(
                normal_to_transverse_pressure_angle(
                    member.generated_drive_normal_pressure_angle, beta
                ),
                abs=1e-12,
            )
            assert section.coast_transverse_pressure_angle == pytest.approx(
                normal_to_transverse_pressure_angle(
                    member.generated_coast_normal_pressure_angle, beta
                ),
                abs=1e-12,
            )

            pitch = member.virtual_pitch_r * scale
            expected_space_angle = math.pi / member.virtual_teeth - (
                section.transverse_tooth_thickness / (2.0 * pitch)
            )
            for flank in (section.drive_flank, section.coast_flank):
                pitch_point = min(
                    flank, key=lambda point: abs(math.hypot(*point) - pitch)
                )
                assert abs(math.atan2(pitch_point[1], pitch_point[0])) == pytest.approx(
                    expected_space_angle, abs=2e-4
                )


def test_hypoid_hand_mirrors_the_drive_and_coast_flanks():
    right = compute_set(ANCHOR)
    left = compute_set(HypoidSetParams(**{**ANCHOR.__dict__, "hand": "left"}))
    right_section = tooth_space_section(right, "pinion")
    left_section = tooth_space_section(left, "pinion")
    assert left_section.spiral_angle == pytest.approx(
        -right_section.spiral_angle, abs=1e-12
    )
    assert left_section.transverse_tooth_thickness == pytest.approx(
        right_section.transverse_tooth_thickness, abs=1e-12
    )
    assert left_section.drive_transverse_pressure_angle == pytest.approx(
        right_section.drive_transverse_pressure_angle, abs=1e-12
    )
    assert left_section.coast_transverse_pressure_angle == pytest.approx(
        right_section.coast_transverse_pressure_angle, abs=1e-12
    )
    assert left_section.drive_base_radius == pytest.approx(
        right_section.drive_base_radius, abs=1e-12
    )
    assert left_section.coast_base_radius == pytest.approx(
        right_section.coast_base_radius, abs=1e-12
    )

    for right_flank, left_flank in (
        (right_section.drive_flank, left_section.drive_flank),
        (right_section.coast_flank, left_section.coast_flank),
    ):
        assert len(right_flank) == len(left_flank)
        for right_point, left_point in zip(right_flank, left_flank):
            assert left_point[0] == pytest.approx(right_point[0], abs=1e-12)
            assert left_point[1] == pytest.approx(-right_point[1], abs=1e-12)


def test_independent_hypoid_flanks_keep_root_and_top_land_constraints():
    geo = compute_set(ANCHOR)
    section = tooth_space_section(geo, "gear", split_cap=True)
    m = geo.gear

    assert math.dist(section.loop_2d[0], section.segments["root"][-1]) < 1e-9
    for name in ("cap_neg", "cap_pos"):
        assert all(
            math.hypot(x, y) == pytest.approx(section.r_cap, abs=1e-9)
            for x, y in section.segments[name]
        )
    assert math.dist(
        section.segments["cap_neg"][-1], section.segments["cap_pos"][0]
    ) < 1e-12
    assert math.hypot(*section.segments["root"][0]) == pytest.approx(
        section.r_root, abs=1e-9
    )
    assert math.hypot(*section.segments["root"][-1]) == pytest.approx(
        section.r_root, abs=1e-9
    )
    for flank in (section.drive_flank, section.coast_flank):
        assert all(
            section.r_root - 1e-9
            <= math.hypot(x, y)
            <= section.r_tip + 1e-9
            for x, y in flank
        )
    left_tip = math.atan2(*section.segments["cap_neg"][0][::-1])
    right_tip = math.atan2(*section.segments["cap_pos"][-1][::-1])
    assert left_tip < 0.0 < right_tip
    assert m.generated_drive_normal_pressure_angle != pytest.approx(
        m.generated_coast_normal_pressure_angle
    )


def test_blank_edges_follow_the_face_and_back_cone_angles():
    geo = compute_set(ANCHOR)
    for member in ("pinion", "gear"):
        m = geo.member(member)
        outline = blank_outline(geo, member)

        def angle_to_axis(a, b):
            return math.atan2(abs(b[0] - a[0]), abs(b[1] - a[1]))

        assert angle_to_axis(outline[1], outline[2]) == pytest.approx(m.face_angle)
        assert angle_to_axis(outline[2], outline[3]) == pytest.approx(
            math.pi / 2.0 - m.pitch_angle
        )


def test_blank_spans_both_sides_of_mean_cone_and_uses_heel_diameter():
    geo = compute_set(ANCHOR)
    for member in ("pinion", "gear"):
        m = geo.member(member)
        outline = blank_outline(geo, member)
        assert outline[1] == pytest.approx((m.inner_tip_radius, m.inner_tip_z), abs=1e-9)
        assert outline[2] == pytest.approx((m.outer_tip_radius, m.outer_tip_z), abs=1e-9)
        assert outline[3] == pytest.approx((m.outer_root_radius, m.outer_root_z), abs=1e-9)
        assert m.inner_cone_distance == pytest.approx(
            m.cone_distance - m.inner_face_width, abs=1e-12
        )
        assert m.outer_cone_distance == pytest.approx(
            m.cone_distance + m.outer_face_width, abs=1e-12
        )
        assert m.outside_dia / 2.0 == pytest.approx(outline[2][0], abs=1e-9)
        assert m.outside_dia / 2.0 > m.outer_pitch_diameter / 2.0


def test_method_1_physical_root_is_separate_from_tredgold_root():
    geo = compute_set(ANCHOR)
    for member in (geo.pinion, geo.gear):
        assert member.outer_root_radius != pytest.approx(
            member.tredgold_outer_root_radius, abs=1e-6
        )
        assert member.root_r == pytest.approx(member.mean_root_radius, abs=1e-12)


def test_mesh_clocking_and_preview_are_available():
    geo = compute_set(ANCHOR)
    assert 0.0 <= gear_clocking(geo) < 2.0 * math.pi / geo.gear.z
    for key, _ in preview.SCENE_LABELS:
        scene = preview.build_scene(geo, "pinion", key)
        assert scene.polylines
    assert any(row.label == "hypoid offset" for row in preview.derived_rows(geo))
    assert preview.csv_lines(geo, "pinion")[0].startswith("member,section")


def test_skew_contact_points_and_axis_offset_agree():
    geo = compute_set(ANCHOR)
    assert gear_contact_point(geo) == pytest.approx(contact_point(geo), abs=1e-9)
    translation = gear_translation(geo)
    axis1 = (0.0, 0.0, 1.0)
    axis2 = (math.sin(geo.params.sigma), 0.0, math.cos(geo.params.sigma))
    assert skew_axis_distance((0.0, 0.0, 0.0), axis1, translation, axis2) == (
        pytest.approx(abs(geo.params.offset), abs=1e-9)
    )
    theta1, theta2 = contact_azimuths(geo)
    implied_offset = (
        geo.pinion.pitch_radius * math.sin(theta1)
        - geo.gear.pitch_radius * math.sin(theta2)
    )
    assert implied_offset == pytest.approx(geo.params.offset, abs=1e-9)


def test_member_traces_turn_in_opposite_senses_and_hand_mirrors_them():
    right = compute_set(ANCHOR)
    left = compute_set(HypoidSetParams(**{**ANCHOR.__dict__, "hand": "left"}))
    for geo in (right, left):
        pinion = tooth_space_section(
            geo, "pinion", geo.pinion.cone_distance + 5.0
        )
        gear = tooth_space_section(geo, "gear", geo.gear.cone_distance + 5.0)
        assert pinion.phase * gear.phase < 0.0
    assert left.pinion.mean_spiral_angle == pytest.approx(
        -right.pinion.mean_spiral_angle
    )
    assert left.gear.mean_spiral_angle == pytest.approx(
        -right.gear.mean_spiral_angle
    )


def _actual_3d_trace_spiral_angle(geo, member, cone_dist):
    """Numerically measure the pitch-trace tangent through production geometry."""
    m = geo.member(member)

    def point(distance):
        section = tooth_space_section(geo, member, distance)
        # The developed virtual pitch point is the point used by the
        # longitudinal trace.  Passing it through both production functions is
        # important: this measures the phase that the loft receives, rather
        # than measuring a private phase helper in isolation.
        radius = m.virtual_pitch_r * distance / m.cone_distance
        return to_cone_3d(
            radius, 0.0, m.pitch_angle, section.cone_apex_z, section.phase
        )

    h = 1e-4
    before, after = point(cone_dist - h), point(cone_dist + h)
    tangent = tuple((after[i] - before[i]) / (2.0 * h) for i in range(3))
    current = point(cone_dist)
    azimuth = math.atan2(current[1], current[0])
    circumferential = (-math.sin(azimuth), math.cos(azimuth), 0.0)
    generator = (
        math.sin(m.pitch_angle) * math.cos(azimuth),
        math.sin(m.pitch_angle) * math.sin(azimuth),
        math.cos(m.pitch_angle),
    )
    tangent_component = sum(tangent[i] * circumferential[i] for i in range(3))
    generator_component = sum(tangent[i] * generator[i] for i in range(3))
    return math.atan2(tangent_component, generator_component)


@pytest.mark.parametrize(
    "label, params",
    [
        ("anchor", ANCHOR),
        (
            "positive_offset",
            HypoidSetParams.with_defaults(
                2.0, 17, 43, offset=6.0, face_width=8.0,
                spiral_angle=35.0, cutter_radius=30.0,
            ),
        ),
        ("negative_offset", replace(ANCHOR, offset=-15.0)),
        ("zero_offset", replace(ANCHOR, offset=0.0)),
        ("left_hand", replace(ANCHOR, hand="left")),
    ],
)
def test_actual_3d_trace_reproduces_method1_mean_and_face_spirals(label, params):
    """The phase supplied to each Tredgold section has the Method 1 slope.

    The two local production phases have opposite rotational senses, so the
    signed 3D slope is ``+member`` on the pinion and ``-member`` on the wheel.
    The reported Method 1 spiral angles are the corresponding hand-signed
    magnitudes; both forms are asserted here at the calculation point and at
    the physical inner/outer tooth-face boundaries.
    """
    geo = compute_set(params)
    for name in ("pinion", "gear"):
        member = geo.member(name)
        expected_sense = 1.0 if name == "pinion" else -1.0
        for distance, expected in (
            (member.tooth_face_inner_cone_distance, member.inner_spiral_angle),
            (member.cone_distance, member.mean_spiral_angle),
            (member.tooth_face_outer_cone_distance, member.outer_spiral_angle),
        ):
            actual = _actual_3d_trace_spiral_angle(geo, name, distance)
            assert actual == pytest.approx(
                expected_sense * expected, abs=2e-6
            ), f"{label} {name} at A={distance}"
            assert abs(actual) == pytest.approx(abs(expected), abs=2e-6)


def test_cutter_radius_changes_off_mean_loft_sections():
    smaller = compute_set(ANCHOR)
    larger = compute_set(
        HypoidSetParams(**{**ANCHOR.__dict__, "cutter_radius": 120.0})
    )
    assert larger.pinion.pitch_angle != pytest.approx(smaller.pinion.pitch_angle)
    assert larger.gear.pitch_angle != pytest.approx(smaller.gear.pitch_angle)
    assert larger.method1.limit_radius_of_curvature == pytest.approx(120.0, abs=1e-8)
    assert smaller.method1.limit_radius_of_curvature == pytest.approx(63.5, abs=1e-8)
    for member in ("pinion", "gear"):
        small = smaller.member(member)
        large = larger.member(member)
        assert tooth_space_section(
            smaller, member, small.cone_distance - 10.0
        ).phase != pytest.approx(
            tooth_space_section(
                larger, member, large.cone_distance - 10.0
            ).phase
        )
        assert tooth_space_section(
            smaller, member, small.cone_distance
        ).phase == pytest.approx(
            tooth_space_section(larger, member, large.cone_distance).phase
        )


def test_cutter_radius_must_cover_both_member_faces():
    p = HypoidSetParams(**{**ANCHOR.__dict__, "cutter_radius": 10.0})
    result = validate(p)
    assert not result.ok
    assert any(issue.field == "cutter_radius" for issue in result.errors)


@pytest.mark.parametrize(
    "module,z1,z2,offset,face_width,spiral,cutter",
    [
        (2.0, 17, 43, 6.0, 8.0, 25.0, 30.0),
        (3.0, 19, 57, -10.0, 12.0, 48.0, 50.0),
    ],
)
def test_non_anchor_backlash_uses_each_member_spiral_angle(
    module, z1, z2, offset, face_width, spiral, cutter
):
    p = HypoidSetParams.with_defaults(
        module, z1, z2, offset=offset, face_width=face_width,
        spiral_angle=spiral, cutter_radius=cutter, backlash=0.2,
    )
    geo = compute_set(p)
    expected_mean_transverse = (
        p.backlash * geo.gear.cone_distance / geo.gear.outer_cone_distance
    )
    expected_mean_normal = expected_mean_transverse * abs(
        math.cos(geo.gear.mean_spiral_angle)
    )
    assert geo.mean_transverse_backlash == pytest.approx(
        expected_mean_transverse, abs=1e-12
    )
    assert geo.mean_normal_backlash == pytest.approx(
        expected_mean_normal, abs=1e-12
    )
    for member in (geo.pinion, geo.gear):
        assert member.mean_transverse_tooth_thickness == pytest.approx(
            member.mean_normal_tooth_thickness / abs(math.cos(member.mean_spiral_angle)),
            abs=1e-12,
        )
    assert geo.pinion.mean_spiral_angle != pytest.approx(geo.gear.mean_spiral_angle)


def test_excessive_outer_transverse_backlash_is_rejected_by_tooth_thickness():
    invalid = HypoidSetParams(**{**ANCHOR.__dict__, "backlash": 100.0})
    result = validate(invalid)
    assert not result.ok
    assert any(
        issue.field == "backlash" and "tooth thickness" in issue.message
        for issue in result.errors
    )
    assert not any(
        "5%" in issue.message for issue in result.warnings
    )


def test_method_1_reports_a_non_convergent_curvature_design():
    invalid = HypoidSetParams(**{**ANCHOR.__dict__, "cutter_radius": 1.0})
    with pytest.raises(ValueError, match="curvature"):
        compute_set(invalid)
    result = validate(invalid)
    assert not result.ok
    assert any(issue.field == "cutter_radius" for issue in result.errors)


def test_invalid_offset_and_cutter_are_reported():
    bad = HypoidSetParams.with_defaults(2.0, 13, 42, offset=100.0, cutter_radius=-1.0)
    result = validate(bad)
    assert not result.ok
    assert {issue.field for issue in result.errors} >= {"offset", "cutter_radius"}


def test_hypoid_is_selectable_from_the_cli(capsys):
    status = main([
        "--type", "hypoid", "--module", str(170.0 / 42.0),
        "--z1", "13", "--z2", "42", "--offset", "15",
        "--face-width", "30", "--spiral", "50", "--cutter-radius", "63.5",
        "--backlash", "0.2",
    ])
    assert status == 0
    output = capsys.readouterr().out
    assert "pitch-plane offset" in output
    assert "pitch cone angle" in output
    assert "outer transverse backlash" in output
    assert "mean normal backlash" in output
    assert "generated drive normal pressure angle" in output
    assert "generated coast normal pressure angle" in output
