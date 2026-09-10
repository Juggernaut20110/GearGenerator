"""Geometric invariants for the Tk-free 3D assembly preview."""

from __future__ import annotations

import math

import pytest

from gears.bevel.geometry import compute_set as compute_bevel
from gears.bevel.params import BevelSetParams
from gears.hypoid.geometry import compute_set as compute_hypoid
from gears.hypoid.params import HypoidSetParams
from gears.planetary.geometry import compute_set as compute_planetary
from gears.planetary.params import PlanetarySetParams
from gears.spur.geometry import compute_set as compute_spur
from gears.spur.params import SpurSetParams
from gears.preview3d import Camera, Scene3D, build_scene


def _axis_line(scene, member):
    return next(line for line in scene.polylines if line.style == "axis" and line.member == member)


def _vector(line):
    return tuple(line.points[1][i] - line.points[0][i] for i in range(3))


def _finite(scene):
    return all(math.isfinite(value) for line in scene.polylines for point in line.points for value in point)


def test_scene_bounds_have_six_coordinates_and_handle_empty_scenes():
    assert Scene3D().bounds() == (-1.0, -1.0, -1.0, 1.0, 1.0, 1.0)


def test_front_top_and_right_projection_conventions_are_stable():
    camera = Camera()

    camera.front()
    assert camera.project((1.0, 2.0, 3.0), 100.0, 80.0) == pytest.approx((51.0, 38.0))

    camera.top()
    assert camera.project((1.0, 2.0, 3.0), 100.0, 80.0) == pytest.approx((51.0, 43.0))

    camera.right()
    assert camera.project((1.0, 2.0, 3.0), 100.0, 80.0) == pytest.approx((53.0, 38.0))


def test_camera_isometric_pan_and_zoom_are_finite_and_effective():
    camera = Camera()
    camera.iso()
    before = camera.project((2.0, 3.0, 4.0), 200.0, 120.0)
    camera.orbit(18.0, -9.0)
    camera.pan(7.0, -4.0)
    camera.zoom_by(1.5)
    after = camera.project((2.0, 3.0, 4.0), 200.0, 120.0)
    assert _finite(Scene3D())
    assert all(math.isfinite(value) for value in after)
    assert after != before


def test_camera_fit_contains_axis_aligned_scene_bounds():
    camera = Camera()
    camera.iso()
    bounds = (-10.0, -5.0, -2.0, 20.0, 5.0, 8.0)
    camera.fit(bounds, 300.0, 200.0, margin=20.0)
    corners = [
        (x, y, z)
        for x in (bounds[0], bounds[3])
        for y in (bounds[1], bounds[4])
        for z in (bounds[2], bounds[5])
    ]
    for point in corners:
        x, y = camera.project(point, 300.0, 200.0)
        assert 20.0 - 1e-9 <= x <= 280.0 + 1e-9
        assert 20.0 - 1e-9 <= y <= 180.0 + 1e-9


def test_spur_pair_uses_working_distance_parallel_axes_and_real_teeth():
    geo = compute_spur(SpurSetParams.with_defaults(2.0, 17, 43))
    scene = build_scene(geo)
    pinion_axis = _axis_line(scene, "pinion")
    gear_axis = _axis_line(scene, "gear")
    separation = math.dist(pinion_axis.points[0], gear_axis.points[0])
    assert separation == pytest.approx(geo.working_centre_distance)
    assert _vector(pinion_axis) == pytest.approx(_vector(gear_axis))
    assert len({round(point[2], 6) for line in scene.polylines if line.member == "pinion" for point in line.points}) > 1
    assert {line.style for line in scene.polylines} >= {"pinion", "gear"}
    assert _finite(scene)


def test_internal_spur_ring_is_on_the_negative_x_side():
    geo = compute_spur(
        SpurSetParams.with_defaults(
            2.0, 18, 60, internal=True,
            backlash_mode="working_circumferential",
        )
    )
    scene = build_scene(geo)
    assert _axis_line(scene, "gear").points[0][0] < 0.0
    assert math.dist(_axis_line(scene, "pinion").points[0], _axis_line(scene, "gear").points[0]) == pytest.approx(geo.working_centre_distance)


def test_helical_spur_has_different_profile_phase_at_the_two_faces():
    geo = compute_spur(SpurSetParams.with_defaults(2.0, 17, 43, helix_angle=15.0))
    scene = build_scene(geo)
    pinion_loops = [line for line in scene.polylines if line.member == "pinion" and line.closed]
    front = pinion_loops[0].points[0]
    back = pinion_loops[4].points[0]
    assert math.atan2(back[1], back[0]) != pytest.approx(math.atan2(front[1], front[0]))


def test_mesh_position_moves_the_mating_pair_by_the_existing_ratio():
    geo = compute_spur(SpurSetParams.with_defaults(2.0, 17, 43))
    start = build_scene(geo, mesh_position=0.0)
    moved = build_scene(geo, mesh_position=0.25)
    start_gear = next(line for line in start.polylines if line.member == "gear" and line.closed)
    moved_gear = next(line for line in moved.polylines if line.member == "gear" and line.closed)
    start_angle = math.atan2(start_gear.points[0][1], start_gear.points[0][0] - geo.working_centre_distance)
    moved_angle = math.atan2(moved_gear.points[0][1], moved_gear.points[0][0] - geo.working_centre_distance)
    expected = -0.25 * geo.pinion.z / geo.gear.z
    assert (moved_angle - start_angle) == pytest.approx(expected)


def test_bevel_axes_match_calculated_shaft_angle_and_sections_are_3d():
    geo = compute_bevel(BevelSetParams.with_defaults(2.0, 17, 43))
    scene = build_scene(geo)
    pinion = _vector(_axis_line(scene, "pinion"))
    gear = _vector(_axis_line(scene, "gear"))
    dot = sum(pinion[i] * gear[i] for i in range(3))
    angle = math.acos(dot / math.sqrt(sum(v * v for v in pinion) * sum(v * v for v in gear)))
    assert angle == pytest.approx(geo.params.sigma)
    assert len({round(point[2], 6) for line in scene.polylines if line.member == "pinion" for point in line.points}) > 1
    assert _finite(scene)


def test_hypoid_scene_reflects_skew_offset_and_contains_both_members():
    geo = compute_hypoid(
        HypoidSetParams.with_defaults(
            170.0 / 42.0, 13, 42, offset=15.0, face_width=30.0,
            spiral_angle=50.0, cutter_radius=63.5,
        )
    )
    scene = build_scene(geo)
    gear_origin = _axis_line(scene, "gear").points[0]
    assert abs(gear_origin[1]) > 1e-6
    assert {line.style for line in scene.polylines} >= {"pinion", "gear"}
    assert _finite(scene)


def test_planetary_scene_contains_sun_every_planet_and_ring():
    geo = compute_planetary(PlanetarySetParams.with_defaults(2.0, 24, 18, n_planets=3))
    scene = build_scene(geo)
    members = {line.member for line in scene.polylines}
    assert {"sun", "ring", "planet 0", "planet 1", "planet 2"} <= members
    assert {line.style for line in scene.polylines} >= {"sun", "planet", "ring"}
    assert _finite(scene)
