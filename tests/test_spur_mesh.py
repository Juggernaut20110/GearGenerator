"""Spur placement and clocking maths. No SOLIDWORKS involved."""

from __future__ import annotations

import math

import pytest

from gears.spur import mesh
from gears.spur.geometry import compute_set, tooth_space_section
from gears.spur.params import SpurSetParams

ANCHOR = SpurSetParams.with_defaults(2.0, 17, 43)
TOOTH_COUNTS = [(17, 43), (20, 20), (12, 60), (21, 21), (13, 31)]


@pytest.fixture
def geo():
    return compute_set(ANCHOR)


def placed_gear(geo):
    """Where the gear's own frame ends up: rotation plus the centre distance."""
    clocking = mesh.gear_clocking(geo.gear.z)
    matrix = mesh.gear_placement(clocking)
    translation = mesh.gear_translation(geo)

    def place(point):
        x, y, z = mesh.apply(matrix, point)
        return x + translation[0], y + translation[1], z + translation[2]

    return place


# --- the arrangement -------------------------------------------------------


def test_the_pinion_keeps_the_frame_it_was_built_in(geo):
    assert mesh.pinion_placement() == (
        (1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)
    )


def test_the_gear_is_translated_along_x_by_the_centre_distance(geo):
    assert mesh.gear_translation(geo) == (geo.centre_distance, 0.0, 0.0)
    assert geo.centre_distance == pytest.approx(60.0)


def test_the_axes_stay_parallel(geo):
    """No tilt anywhere in a spur placement - that is the whole difference."""
    clocking = mesh.gear_clocking(geo.gear.z)
    assert mesh.axis_of(mesh.gear_placement(clocking)) == pytest.approx((0.0, 0.0, 1.0))
    assert mesh.angle_between(
        mesh.axis_of(mesh.pinion_placement()),
        mesh.axis_of(mesh.gear_placement(clocking)),
    ) == pytest.approx(0.0, abs=1e-12)


def test_the_gear_axis_lands_on_the_line_of_centres(geo):
    place = placed_gear(geo)
    origin = place((0.0, 0.0, 0.0))
    assert origin == pytest.approx((geo.centre_distance, 0.0, 0.0))


def test_a_shifted_external_pair_is_placed_at_the_working_distance():
    geo = compute_set(
        SpurSetParams.with_defaults(2.0, 17, 43, profile_shift_1=0.2)
    )
    assert geo.working_centre_distance != geo.reference_centre_distance
    assert mesh.gear_translation(geo) == pytest.approx(
        (geo.working_centre_distance, 0.0, 0.0)
    )
    assert geo.working_centre_distance == pytest.approx(
        geo.pinion.working_r + geo.gear.working_r
    )


# --- clocking --------------------------------------------------------------


@pytest.mark.parametrize("z1,z2", TOOTH_COUNTS)
def test_clocking_puts_a_gear_tooth_centre_on_the_line_of_centres(z1, z2):
    """The property that lets the spur pair reuse the bevel's clocking formula.

    The pinion's tooth *space* is centred on +X, so the contact point on the line
    of centres falls in a space. The gear must present a tooth *centre* there,
    which in its own frame - looking back toward the pinion - is angle pi.
    """
    geo = compute_set(SpurSetParams.with_defaults(2.0, z1, z2))
    clocking = mesh.gear_clocking(geo.gear.z)
    tau = geo.gear.angular_pitch

    centres = [clocking + (k + 0.5) * tau for k in range(geo.gear.z)]
    nearest = min(
        abs((angle - math.pi + math.pi) % (2.0 * math.pi) - math.pi)
        for angle in centres
    )
    assert nearest == pytest.approx(0.0, abs=1e-12)


@pytest.mark.parametrize("z", [17, 21, 43, 61])
def test_odd_tooth_counts_need_no_clocking(z):
    assert mesh.gear_clocking(z) == 0.0


@pytest.mark.parametrize("z", [12, 20, 30, 60])
def test_even_tooth_counts_need_half_a_pitch(z):
    assert mesh.gear_clocking(z) == pytest.approx(math.pi / z, rel=1e-12)


def test_clocking_never_returns_a_whole_pitch(geo):
    """The z = 21 trap: (pi - tau/2) % tau can return tau instead of 0."""
    for z in range(6, 200):
        tau = 2.0 * math.pi / z
        assert 0.0 <= mesh.gear_clocking(z) < tau - 1e-15


@pytest.mark.parametrize("z1,z2", TOOTH_COUNTS)
def test_a_gear_tooth_faces_a_pinion_space_where_they_touch(z1, z2):
    """End to end, on the real profiles rather than on the angles alone.

    Take the pinion's tooth-space section, place the gear's, and check that the
    gear's material reaches the contact point while the pinion's does not - which
    is what meshing means at the pitch point.
    """
    geo = compute_set(SpurSetParams.with_defaults(2.0, z1, z2))
    contact = (geo.pinion.pitch_r, 0.0)

    # The pinion's space is centred on +X, so the contact point sits inside it:
    # the space's angular half width at the pitch radius is positive.
    pinion_loop = tooth_space_section(geo, "pinion").loop_2d
    at_pitch = [
        math.atan2(y, x)
        for x, y in pinion_loop
        if abs(math.hypot(x, y) - geo.pinion.pitch_r) < 0.35
    ]
    assert at_pitch and min(at_pitch) < 0.0 < max(at_pitch)

    # The gear's space, once clocked and translated, must straddle a different
    # angle - the contact point falls on a tooth, not in a space.
    place = placed_gear(geo)
    gear_loop = [place((x, y, 0.0)) for x, y in tooth_space_section(geo, "gear").loop_2d]
    nearest_space_point = min(
        math.dist((x, y), contact) for x, y, _ in gear_loop
    )
    assert nearest_space_point > 0.05 * geo.transverse_module


# --- ratio -----------------------------------------------------------------


@pytest.mark.parametrize("z1,z2", TOOTH_COUNTS)
def test_the_pair_turns_in_opposite_senses(z1, z2):
    assert mesh.angular_velocity_ratio(z1, z2) == pytest.approx(-z2 / z1, rel=1e-15)


@pytest.mark.parametrize("z1,z2", TOOTH_COUNTS)
def test_the_gear_mate_gets_the_tooth_counts_unchanged(z1, z2):
    assert mesh.gear_mate_ratio(z1, z2) == (float(z1), float(z2))


# --- the transform ---------------------------------------------------------


def test_the_transform_is_written_column_major(geo):
    """Reading it row-major transposes the rotation and mirrors the part."""
    clocking = mesh.gear_clocking(geo.gear.z)
    matrix = mesh.gear_placement(clocking)
    data = mesh.to_array_data(matrix)
    assert data[0:3] == pytest.approx([matrix[0][0], matrix[1][0], matrix[2][0]])
    assert data[3:6] == pytest.approx([matrix[0][1], matrix[1][1], matrix[2][1]])
    assert data[6:9] == pytest.approx([matrix[0][2], matrix[1][2], matrix[2][2]])


def test_the_translation_column_carries_the_centre_distance(geo):
    """The bevel pair shares an apex and always passes zero here; this one does not."""
    metres = tuple(v * 0.001 for v in mesh.gear_translation(geo))
    data = mesh.to_array_data(mesh.gear_placement(0.0), metres)
    assert data[9:12] == pytest.approx([geo.centre_distance * 0.001, 0.0, 0.0])
    assert data[12] == 1.0
    assert data[13:16] == [0.0, 0.0, 0.0]


def test_an_unclocked_unmoved_gear_packs_to_the_identity():
    data = mesh.to_array_data(mesh.pinion_placement())
    assert data == pytest.approx(
        [1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0]
    )


# --- what the assembly builder relies on -----------------------------------


def test_the_gear_needs_its_own_axial_locator(geo):
    """Both members' fronts sit at z = 0, and nothing else puts them there.

    A bevel pair gets all three translations from putting both origins on the
    assembly origin, because it shares an apex. A spur pair does not: the gear's
    origin is `a` away, so its axial position has to be asked for separately.
    That is what the gear-origin-to-Front-plane mate is for, and this records
    that the number it should land on is zero.
    """
    assert mesh.gear_translation(geo)[2] == 0.0
    place = placed_gear(geo)
    assert place((0.0, 0.0, 0.0))[2] == pytest.approx(0.0, abs=1e-12)


def test_the_gear_axis_cannot_lie_in_the_front_plane(geo):
    """Why the gear is mated to Top and to a parallel, not to Front.

    Its axis runs along +Z and the Front plane is z = 0, so no rotation in the
    placement could put the axis in that plane - only the Top and Right planes
    contain a +Z direction, and Right would force the gear onto x = 0.
    """
    clocking = mesh.gear_clocking(geo.gear.z)
    axis = mesh.axis_of(mesh.gear_placement(clocking))
    assert abs(axis[2]) == pytest.approx(1.0)      # wholly along Z
    assert axis[0] == pytest.approx(0.0, abs=1e-12)
    assert axis[1] == pytest.approx(0.0, abs=1e-12)
    assert mesh.gear_translation(geo)[0] > 0.0     # and not on x = 0
