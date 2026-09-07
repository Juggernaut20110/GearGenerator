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


def test_method_1_anchor_converges_to_published_macro_geometry():
    geo = compute_set(ANCHOR)
    assert geo.pinion.pitch_angle_deg == pytest.approx(21.288, abs=0.02)
    assert geo.gear.pitch_angle_deg == pytest.approx(68.324, abs=0.02)
    assert geo.offset_angle_deg == pytest.approx(11.39, abs=0.03)
    assert geo.pitch_plane_offset == pytest.approx(15.075, abs=0.002)
    assert geo.pinion.mean_spiral_angle_deg == pytest.approx(50.0, abs=1e-9)
    assert geo.gear.mean_spiral_angle_deg == pytest.approx(38.61, abs=0.03)
    assert geo.mean_normal_module == pytest.approx(2.64, abs=0.01)
    assert (
        geo.pinion.normal_tooth_thickness + geo.gear.normal_tooth_thickness
    ) == pytest.approx(math.pi * geo.mean_normal_module - ANCHOR.backlash)


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
        with_backlash.pinion.normal_tooth_thickness
        + with_backlash.gear.normal_tooth_thickness
    )
    assert normal_sum == pytest.approx(
        math.pi * with_backlash.mean_normal_module - 0.2, abs=1e-9
    )


def test_offset_is_signed_but_axis_distance_is_positive():
    positive = compute_set(ANCHOR)
    negative = compute_set(HypoidSetParams(**{**ANCHOR.__dict__, "offset": -15.0}))
    assert positive.offset_angle == pytest.approx(-negative.offset_angle)
    assert abs(positive.pitch_plane_offset) == pytest.approx(abs(negative.pitch_plane_offset))
    assert gear_translation(positive)[1] == pytest.approx(15.0)
    assert gear_translation(negative)[1] == pytest.approx(-15.0)


def test_zero_offset_reduces_to_intersecting_pitch_cones():
    p = HypoidSetParams.with_defaults(2.0, 17, 43, offset=0.0)
    geo = compute_set(p)
    assert geo.offset_angle == 0.0
    assert geo.pinion.pitch_angle + geo.gear.pitch_angle == pytest.approx(p.sigma)
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
        radial_per_cone_distance = (
            m.virtual_tip_r * math.cos(m.pitch_angle) / m.cone_distance
        )
        assert outline[1][0] / radial_per_cone_distance == pytest.approx(
            m.cone_distance - geo.params.face_width / 2.0, abs=1e-9
        )
        assert outline[2][0] / radial_per_cone_distance == pytest.approx(
            m.cone_distance + geo.params.face_width / 2.0, abs=1e-9
        )
        outer_pitch_radius = (
            m.cone_distance + geo.params.face_width / 2.0
        ) * math.sin(m.pitch_angle)
        assert m.outside_dia / 2.0 == pytest.approx(outline[2][0], abs=1e-9)
        assert m.outside_dia / 2.0 > outer_pitch_radius


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
    ])
    assert status == 0
    output = capsys.readouterr().out
    assert "pitch-plane offset" in output
    assert "pitch cone angle" in output
