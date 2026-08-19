"""Where the members of a planetary train sit, and how each one is clocked.

Pure math - no COM. The sun and the ring are concentric on the origin; planet k
sits at carrier angle `phi_k = 2 pi k / N` on a circle of radius `a`.

The meshing condition, stated once
----------------------------------
Every clocking in this module comes out of one relation. Two gears mesh when the
arc from each one's reference tooth space to the contact point, measured on its
own pitch circle, adds up to half a pitch (plus any whole number of pitches):

    external   r_A (phi_A - c_A) + r_B (phi_B - c_B) = p/2 + n p
    internal   r_A (phi_A - c_A) - r_B (phi_B - c_B) = p/2 + n p

where `c_X` is that member's clocking, `phi_X` is the direction of the contact
point seen from X's own centre, and `p` is the circular pitch. The sign is the
whole difference: external surfaces roll in opposite senses, internal ones in
the same sense.

It is not a new rule. Substituting a plain spur pair - phi_A = 0, c_A = 0 -
gives back `placement.gear_clocking` exactly, and substituting the internal pair
gives back `spur.mesh.internal_gear_clocking`. Both are checked in the tests, so
this module's arithmetic is anchored to two answers that were already right.

What falls out
--------------
For the sun-planet mesh, with the sun unclocked at the origin and the planet at
carrier angle phi_k:

    c_planet(k) = phi_k * (z_s + z_p) / z_p + gear_clocking(z_p)

For the planet-ring mesh, substituting that in:

    c_ring(k) = phi_k * (z_r + z_s) / z_r + gear_clocking(z_p) * z_p / z_r
                + pi / z_r

The ring is one part and can only have one clocking, so `c_ring(k)` has to come
out the same for every planet - modulo the ring's own angular pitch, since a
whole pitch is no rotation at all. That requires

    phi_k * (z_r + z_s) / z_r  ==  0   (mod 2 pi / z_r)

and with `phi_k = 2 pi k / N` it reduces to `k (z_s + z_r) / N` being a whole
number for every k. Which is to say:

    **(z_sun + z_ring) must be divisible by the number of planets**

That is the assembly condition, and it is worth seeing it arrive as the
consequence of a phase having to agree rather than as a rule from a table. A set
that fails it does not have a planet slightly out of place - it has no
consistent ring clocking at all, and the third planet cannot be fitted.
"""

from __future__ import annotations

import math

from ..placement import (
    Matrix3,
    angle_between,
    apply,
    axis_of,
    gear_clocking,
    gear_mate_ratio,
    matmul,
    pinion_placement,
    rot_z,
    to_array_data,
)
from .geometry import PlanetarySetGeometry

__all__ = [
    "Matrix3",
    "angle_between",
    "apply",
    "axis_of",
    "carrier_angle",
    "gear_mate_ratio",
    "member_placement",
    "planet_clocking",
    "planet_translation",
    "pinion_placement",
    "ring_clocking",
    "sun_clocking",
    "to_array_data",
]


def carrier_angle(geo: PlanetarySetGeometry, k: int) -> float:
    """Where planet k sits around the sun, radians from +X."""
    return 2.0 * math.pi * k / geo.params.n_planets


def sun_clocking() -> float:
    """Zero. The sun keeps the frame it was built in.

    Stated as a function rather than left implicit because it is a choice - the
    sun is this train's equivalent of the spur pair's pinion, the one member
    every other clocking is measured against.
    """
    return 0.0


def planet_clocking(geo: PlanetarySetGeometry, k: int) -> float:
    """How far planet k is turned about its own axis, radians.

    `phi_k (z_s + z_p) / z_p + gear_clocking(z_p)` - see the module docstring.

    The two terms say different things and it is worth keeping them apart. The
    second is the clocking a lone spur gear would need against this sun, and is
    the same for every planet. The first is what carrying the planet round to
    its station does: the sun stays still while the planet's centre moves, so
    the planet has to roll along the sun's teeth to get there.
    """
    p = geo.params
    phi = carrier_angle(geo, k)
    return phi * (p.z_sun + p.z_planet) / p.z_planet + gear_clocking(p.z_planet)


def ring_clocking(geo: PlanetarySetGeometry, k: int = 0) -> float:
    """How far the ring is turned about its own axis, radians.

    Solved against planet `k`, and the answer is the same for every k modulo the
    ring's angular pitch **exactly when the assembly condition holds**. It is
    computed against planet 0 by default because that is the one whose carrier
    angle is zero, which drops the first term entirely.

    `validate` refuses a set whose assembly condition fails, so by the time a
    build reaches here the choice of k does not matter. The argument is kept so
    a test can ask the question for each planet and check they agree.
    """
    p = geo.params
    phi = carrier_angle(geo, k)
    return (
        phi * (p.z_ring + p.z_sun) / p.z_ring
        + gear_clocking(p.z_planet) * p.z_planet / p.z_ring
        + math.pi / p.z_ring
    )


def planet_translation(geo: PlanetarySetGeometry, k: int) -> tuple[float, float, float]:
    """Where planet k's origin goes, mm. On the orbit circle, at its carrier angle.

    Millimetres, like everything else upstream of `gears.sw`; the SOLIDWORKS
    layer converts at its own boundary.
    """
    phi = carrier_angle(geo, k)
    a = geo.centre_distance
    return (a * math.cos(phi), a * math.sin(phi), 0.0)


def member_placement(clocking: float) -> Matrix3:
    """Spin a member about its own axis. That is the whole rotation.

    Every axis in a planetary train is parallel to +Z, so no member is ever
    tilted - which is what makes this a spur assembly with more parts in it
    rather than a new kind of arrangement.
    """
    return rot_z(clocking)


def sun_planet_ratio(geo: PlanetarySetGeometry) -> tuple[float, float]:
    """The two numbers a gear mate wants for a sun-planet mesh, sun selected first."""
    return gear_mate_ratio(geo.params.z_sun, geo.params.z_planet)


def planet_ring_ratio(geo: PlanetarySetGeometry) -> tuple[float, float]:
    """The same, for a planet-ring mesh with the planet selected first."""
    return gear_mate_ratio(geo.params.z_planet, geo.params.z_ring)
