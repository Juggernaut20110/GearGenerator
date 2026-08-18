"""Placement and clocking maths. No SOLIDWORKS involved."""

from __future__ import annotations

import math

import pytest

from bevelgear import mesh
from bevelgear.geometry import compute_set
from bevelgear.params import BevelSetParams


# --- clocking --------------------------------------------------------------


@pytest.mark.parametrize("z", [12, 17, 20, 21, 43, 60, 97])
def test_clocking_puts_a_tooth_centre_at_the_contact_line(z):
    """After clocking, some tooth centre must land at pi in the gear's frame.

    Tooth centres sit at (k + 1/2) * tau; the contact line is at pi.
    """
    tau = 2.0 * math.pi / z
    phi = mesh.gear_clocking(z)
    offset = (math.pi - phi) / tau - 0.5
    assert offset == pytest.approx(round(offset), abs=1e-12)


@pytest.mark.parametrize("z", [12, 17, 20, 43])
def test_clocking_is_within_one_pitch(z):
    assert 0.0 <= mesh.gear_clocking(z) < 2.0 * math.pi / z


def test_odd_tooth_counts_need_no_clocking():
    """pi is already an odd multiple of half the pitch when z is odd."""
    for z in (17, 21, 43, 97):
        assert mesh.gear_clocking(z) == pytest.approx(0.0, abs=1e-12)


def test_even_tooth_counts_need_half_a_pitch():
    for z in (12, 20, 60):
        assert mesh.gear_clocking(z) == pytest.approx(math.pi / z, rel=1e-12)


# --- placement -------------------------------------------------------------


@pytest.mark.parametrize("sigma_deg", [45.0, 60.0, 90.0, 120.0])
def test_gear_axis_lands_at_the_shaft_angle(sigma_deg):
    sigma = math.radians(sigma_deg)
    m = mesh.gear_placement(sigma, 0.3)
    axis = mesh.axis_of(m)
    assert axis == pytest.approx((math.sin(sigma), 0.0, math.cos(sigma)), abs=1e-12)


@pytest.mark.parametrize("sigma_deg", [45.0, 60.0, 90.0])
@pytest.mark.parametrize("clocking", [0.0, 0.15, 1.0])
def test_clocking_does_not_disturb_the_axis(sigma_deg, clocking):
    """Spinning the gear about its own axis must not move that axis."""
    sigma = math.radians(sigma_deg)
    a = mesh.axis_of(mesh.gear_placement(sigma, 0.0))
    b = mesh.axis_of(mesh.gear_placement(sigma, clocking))
    assert a == pytest.approx(b, abs=1e-12)


@pytest.mark.parametrize("sigma_deg", [45.0, 60.0, 90.0, 135.0])
def test_angle_between_the_two_axes_is_the_shaft_angle(sigma_deg):
    sigma = math.radians(sigma_deg)
    pinion = mesh.axis_of(mesh.pinion_placement())
    gear = mesh.axis_of(mesh.gear_placement(sigma, 0.0))
    assert math.degrees(mesh.angle_between(pinion, gear)) == pytest.approx(
        sigma_deg, abs=1e-9
    )


@pytest.mark.parametrize("sigma_deg,z1,z2", [(90.0, 17, 43), (60.0, 20, 20)])
def test_pitch_cones_are_tangent_along_the_contact_line(sigma_deg, z1, z2):
    """The real meshing condition: one line lies on both pitch cones.

    The contact line must sit at delta1 from the pinion axis and delta2 from
    the gear axis, with both apexes at the origin.
    """
    g = compute_set(
        BevelSetParams.with_defaults(2.0, z1, z2, shaft_angle=sigma_deg)
    )
    sigma = g.params.sigma
    d1 = g.pinion.pitch_angle

    contact = (math.sin(d1), 0.0, math.cos(d1))
    pinion_axis = mesh.axis_of(mesh.pinion_placement())
    gear_axis = mesh.axis_of(mesh.gear_placement(sigma, 0.0))

    assert mesh.angle_between(contact, pinion_axis) == pytest.approx(d1, abs=1e-12)
    assert mesh.angle_between(contact, gear_axis) == pytest.approx(
        g.gear.pitch_angle, abs=1e-12
    )


def test_contact_line_sits_at_pi_in_the_gear_frame():
    """This is what the clocking formula is measured against, so pin it down."""
    g = compute_set(BevelSetParams.with_defaults(2.0, 17, 43))
    sigma, d1 = g.params.sigma, g.pinion.pitch_angle
    placement = mesh.gear_placement(sigma, 0.0)

    # Where does the gear's own +X end up?
    gear_x = mesh.apply(placement, (1.0, 0.0, 0.0))
    gear_axis = mesh.axis_of(placement)
    contact = (math.sin(d1), 0.0, math.cos(d1))

    # Strip the axial part of the contact line, then compare with gear +X.
    along = sum(c * a for c, a in zip(contact, gear_axis))
    perp = tuple(c - along * a for c, a in zip(contact, gear_axis))
    assert math.degrees(mesh.angle_between(perp, gear_x)) == pytest.approx(
        180.0, abs=1e-9
    )


# --- the gear mate ---------------------------------------------------------


@pytest.mark.parametrize("sigma_deg", [30.0, 45.0, 60.0, 90.0, 120.0, 135.0])
@pytest.mark.parametrize("z1,z2", [(17, 43), (20, 20), (12, 60)])
def test_velocity_ratio_is_the_tooth_ratio_at_every_shaft_angle(sigma_deg, z1, z2):
    """The closed form claims every shaft angle term cancels. Check it numerically.

    Solved here the long way round - from the rolling condition on the real
    pitch angles - so this is an independent check of the algebra in the
    docstring, not a restatement of it.
    """
    g = compute_set(
        BevelSetParams.with_defaults(2.0, z1, z2, shaft_angle=sigma_deg)
    )
    sigma, d1 = g.params.sigma, g.pinion.pitch_angle

    # w1 = w2 (cos S - sin S cot d1), from the two components of
    # w1*a1 - w2*a2 = k*u with w2 = 1.
    from_geometry = math.cos(sigma) - math.sin(sigma) / math.tan(d1)
    assert mesh.angular_velocity_ratio(z1, z2) == pytest.approx(
        from_geometry, rel=1e-12
    )


@pytest.mark.parametrize("z1,z2", [(17, 43), (20, 20), (12, 60)])
def test_the_members_always_turn_in_opposite_senses(z1, z2):
    assert mesh.angular_velocity_ratio(z1, z2) < 0.0


def test_mate_ratio_is_the_tooth_counts_in_selection_order():
    """Pinion entity is selected first, so its count is the numerator."""
    assert mesh.gear_mate_ratio(17, 43) == (17.0, 43.0)


@pytest.mark.parametrize("z1,z2", [(17, 43), (20, 20), (12, 60)])
def test_mate_ratio_and_velocity_ratio_agree_in_magnitude(z1, z2):
    """A gear mate holds w1*r1 = w2*r2, so w1/w2 must come back out as r2/r1."""
    r1, r2 = mesh.gear_mate_ratio(z1, z2)
    assert r2 / r1 == pytest.approx(abs(mesh.angular_velocity_ratio(z1, z2)))


# --- transform packing -----------------------------------------------------


def test_array_data_is_column_major_and_16_long():
    m = ((1.0, 2.0, 3.0), (4.0, 5.0, 6.0), (7.0, 8.0, 9.0))
    data = mesh.to_array_data(m, translation=(0.1, 0.2, 0.3), scale=1.0)
    assert len(data) == 16
    # First three entries are the image of the X axis, i.e. the first column.
    assert data[0:3] == [1.0, 4.0, 7.0]
    assert data[3:6] == [2.0, 5.0, 8.0]
    assert data[6:9] == [3.0, 6.0, 9.0]
    assert data[9:12] == [0.1, 0.2, 0.3]
    assert data[12] == 1.0


def test_identity_packs_to_the_identity_transform():
    data = mesh.to_array_data(mesh.pinion_placement())
    assert data[0:9] == [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    assert data[9:12] == [0.0, 0.0, 0.0]
    assert data[12] == 1.0
