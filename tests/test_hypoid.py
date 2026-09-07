"""Pure-Python acceptance tests for the ISO Method 1 hypoid support."""

import math

import pytest

from gears.__main__ import main
from gears.hypoid import preview
from gears.hypoid.geometry import (
    blank_outline,
    compute_set,
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


def test_method_1_anchor_depths_and_angles_use_independent_equations():
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

    assert geo.pinion.face_width == pytest.approx(31.910, abs=0.002)
    assert geo.gear.face_width == pytest.approx(30.0, abs=1e-12)
    method = geo.method1
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


def _independent_limit_radius(geo):
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


def test_method_1_public_fields_close_independent_pitch_and_curvature_equations():
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
    alpha_lim, rho_lim = _independent_limit_radius(geo)
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


def test_member_trace_tangents_agree_in_the_skew_axis_frame():
    geo = compute_set(ANCHOR)
    theta1, theta2 = contact_azimuths(geo)

    def point(member, cone_dist, theta):
        m = geo.member(member)
        phase = tooth_space_section(geo, member, cone_dist).phase
        local = (
            cone_dist * math.sin(m.pitch_angle) * math.cos(theta + phase),
            cone_dist * math.sin(m.pitch_angle) * math.sin(theta + phase),
            cone_dist * math.cos(m.pitch_angle),
        )
        if member == "pinion":
            return local
        rotated = apply(rot_y(geo.params.sigma), local)
        translation = gear_translation(geo)
        return tuple(rotated[i] + translation[i] for i in range(3))

    def tangent(member, theta):
        mean = geo.member(member).cone_distance
        h = 1e-4
        before, after = point(member, mean - h, theta), point(member, mean + h, theta)
        vector = tuple((after[i] - before[i]) / (2.0 * h) for i in range(3))
        length = math.sqrt(sum(value * value for value in vector))
        return tuple(value / length for value in vector)

    pinion_tangent = tangent("pinion", theta1)
    gear_tangent = tangent("gear", theta2)
    assert sum(pinion_tangent[i] * gear_tangent[i] for i in range(3)) == (
        pytest.approx(1.0, abs=1e-10)
    )


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
