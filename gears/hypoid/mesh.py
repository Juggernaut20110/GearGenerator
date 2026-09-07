"""Skew-axis placement and rolling arithmetic for a hypoid pair."""

from __future__ import annotations

import math

from ..placement import (
    Matrix3,
    angle_between,
    angular_velocity_ratio,
    apply,
    gear_mate_ratio,
    matmul,
    rot_y,
    rot_z,
)
from .geometry import HypoidSetGeometry, contact_azimuths

__all__ = [
    "Matrix3", "angle_between", "angular_velocity_ratio", "axis_offset",
    "contact_azimuths", "contact_point", "gear_clocking",
    "gear_contact_point", "gear_mate_ratio", "gear_placement",
    "gear_translation", "pinion_placement", "skew_axis_distance",
]


def pinion_placement(geo: HypoidSetGeometry) -> Matrix3:
    """Clock the pinion's mean tooth space onto the skew contact point."""
    theta1, _ = contact_azimuths(geo)
    return rot_z(theta1)


def gear_clocking(geo: HypoidSetGeometry) -> float:
    """Put a wheel tooth centre at its solved skew contact azimuth."""
    _, target = contact_azimuths(geo)
    tau = 2.0 * math.pi / geo.gear.z
    nearest = round(target / tau - 0.5)
    residual = target - (nearest + 0.5) * tau
    return 0.0 if abs(residual) < 1e-9 else residual % tau


def gear_placement(shaft_angle: float, clocking: float) -> Matrix3:
    return matmul(rot_y(shaft_angle), rot_z(clocking))


def gear_translation(geo: HypoidSetGeometry) -> tuple[float, float, float]:
    """Align both solved mean pitch points while retaining the shaft offset."""
    theta1, theta2 = contact_azimuths(geo)
    a, b = geo.pinion, geo.gear
    pinion_point = (
        a.pitch_radius * math.cos(theta1),
        a.pitch_radius * math.sin(theta1),
        a.cone_distance * math.cos(a.pitch_angle),
    )
    gear_local = (
        b.pitch_radius * math.cos(theta2),
        b.pitch_radius * math.sin(theta2),
        b.cone_distance * math.cos(b.pitch_angle),
    )
    gear_rotated = apply(rot_y(geo.params.sigma), gear_local)
    return tuple(pinion_point[i] - gear_rotated[i] for i in range(3))


def axis_offset(geo: HypoidSetGeometry) -> float:
    return abs(geo.params.offset)


def contact_point(geo: HypoidSetGeometry) -> tuple[float, float, float]:
    """Mean contact point in the pinion frame, useful for pure tests."""
    m = geo.pinion
    theta1, _ = contact_azimuths(geo)
    return (
        m.pitch_radius * math.cos(theta1),
        m.pitch_radius * math.sin(theta1),
        m.cone_distance * math.cos(m.pitch_angle),
    )


def gear_contact_point(geo: HypoidSetGeometry) -> tuple[float, float, float]:
    """The wheel's mean pitch point after skew placement."""
    _, theta2 = contact_azimuths(geo)
    m = geo.gear
    local = (
        m.pitch_radius * math.cos(theta2),
        m.pitch_radius * math.sin(theta2),
        m.cone_distance * math.cos(m.pitch_angle),
    )
    rotated = apply(rot_y(geo.params.sigma), local)
    translation = gear_translation(geo)
    return tuple(rotated[i] + translation[i] for i in range(3))


def skew_axis_distance(origin1, axis1, origin2, axis2) -> float:
    """Shortest distance between two non-parallel 3D axis lines."""
    cross = (
        axis1[1] * axis2[2] - axis1[2] * axis2[1],
        axis1[2] * axis2[0] - axis1[0] * axis2[2],
        axis1[0] * axis2[1] - axis1[1] * axis2[0],
    )
    norm = math.sqrt(sum(value * value for value in cross))
    if norm < 1e-12:
        delta = tuple(origin2[i] - origin1[i] for i in range(3))
        projection = sum(delta[i] * axis1[i] for i in range(3))
        radial = tuple(delta[i] - projection * axis1[i] for i in range(3))
        return math.sqrt(sum(value * value for value in radial))
    delta = tuple(origin2[i] - origin1[i] for i in range(3))
    return abs(sum(delta[i] * cross[i] for i in range(3))) / norm
