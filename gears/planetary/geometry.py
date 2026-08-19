"""Planetary gear train geometry. Pure math - no COM, no GUI, no file I/O.

Millimetres and radians throughout, like everything upstream of `gears.sw`.

This module owns the **train**, not the tooth. Every member is an involute spur
gear and every mesh is a spur mesh, so the three members are built by calling
`gears.spur.geometry.compute_set` twice and taking one member from each:

    sun + planet     an EXTERNAL pair,  a = m_t (z_s + z_p) / 2
    planet + ring    an INTERNAL pair,  a = m_t (z_r - z_p) / 2

Those two centre distances are the same distance - the planet sits on the line
from the sun's axis to the ring's - which is what forces `z_r = z_s + 2 z_p` and
why the ring's tooth count is not an input.

Coordinate system
-----------------
The sun and the ring are **concentric on the origin**; planet k sits at carrier
angle `2 pi k / N` on a circle of radius `a`. Every axis is parallel to +Z and
every front face is at z = 0, exactly as for a spur pair - so a planetary
assembly is a spur assembly with more components in it, and nothing here needs a
new idea about placement.

The hands
---------
One input settles all three. The sun and planet mesh externally, so they take
opposite hands; the planet and ring mesh internally, so they take the same. That
gives sun right / planet left / ring left, and it is not a convention - each
half of it is forced by `gears.spur.geometry` for the reasons stated there.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from ..spur.geometry import (
    SpurMemberGeometry,
    SpurSetGeometry,
    compute_set as compute_spur_set,
    rim_radius,
    transverse_contact_ratio,
)
from ..spur.params import SpurSetParams
from .params import PlanetarySetParams

MEMBERS = ("sun", "planet", "ring")


@dataclass(frozen=True)
class PlanetarySetGeometry:
    """Everything derived from a PlanetarySetParams."""

    params: PlanetarySetParams
    centre_distance: float          # sun axis to planet axis; the orbit radius
    transverse_module: float
    transverse_pressure_angle: float
    circular_pitch: float
    axial_pitch: float
    whole_depth: float

    sun: SpurMemberGeometry
    planet: SpurMemberGeometry
    ring: SpurMemberGeometry

    # The two meshes, kept whole. Everything a spur pair knows about a mesh -
    # its contact ratio, its section geometry, its blank outlines - is already
    # written against a SpurSetGeometry, so handing those two objects on is what
    # lets this module stay about the train.
    sun_planet: SpurSetGeometry
    planet_ring: SpurSetGeometry

    @property
    def ring_rim_radius(self) -> float:
        return rim_radius(self.planet_ring, "gear")

    @property
    def sun_planet_contact_ratio(self) -> float:
        return self.sun_planet.total_contact_ratio

    @property
    def planet_ring_contact_ratio(self) -> float:
        return self.planet_ring.total_contact_ratio

    @property
    def planet_angles(self) -> list[float]:
        """Carrier angle of each planet, radians, evenly spaced from +X."""
        n = self.params.n_planets
        return [2.0 * math.pi * k / n for k in range(n)]

    @property
    def neighbour_spacing(self) -> float:
        """Straight-line distance between adjacent planet axes, mm.

        `2 a sin(pi / N)`. What the planets' tip circles have to fit inside, and
        the constraint that bites first when someone asks for more of them.
        """
        n = self.params.n_planets
        if n < 2:
            return math.inf
        return 2.0 * self.centre_distance * math.sin(math.pi / n)

    def member(self, which: str) -> SpurMemberGeometry:
        if which == "sun":
            return self.sun
        if which == "planet":
            return self.planet
        if which == "ring":
            return self.ring
        raise ValueError(f"member must be one of {MEMBERS}, got {which!r}")

    def mesh_for(self, which: str) -> SpurSetGeometry:
        """The spur pair a member's *part* should be built from.

        The planet appears in both meshes and is the same gear in each, so
        either would do; the sun-planet pair is picked because that is the one
        whose `pinion`/`gear` roles put the planet second, which is the slot the
        part builder's `member` argument names.
        """
        return self.planet_ring if which == "ring" else self.sun_planet

    def role_of(self, which: str) -> str:
        """Which slot of `mesh_for(which)` this member occupies."""
        return {"sun": "pinion", "planet": "gear", "ring": "gear"}[which]


def _spur_params(p: PlanetarySetParams, z1: int, z2: int, internal: bool,
                 hand: str) -> SpurSetParams:
    """One of the two meshes, as the spur type's own parameter object."""
    return SpurSetParams(
        module=p.module,
        z1=z1,
        z2=z2,
        face_width=p.face_width,
        bore=p.bore,
        hub_thickness=p.hub_thickness,
        pressure_angle=p.pressure_angle,
        helix_angle=p.helix_angle,
        hand=hand,
        internal=internal,
        fillet_factor=p.fillet_factor,
        backlash=p.backlash,
        rim_thickness=p.rim_thickness,
    )


def sun_planet_params(p: PlanetarySetParams) -> SpurSetParams:
    """The external mesh, with the sun as pinion. The sun keeps the stated hand."""
    return _spur_params(p, p.z_sun, p.z_planet, internal=False, hand=p.hand)


def planet_ring_params(p: PlanetarySetParams) -> SpurSetParams:
    """The internal mesh, with the planet as pinion.

    The planet's hand is the **opposite** of the sun's, because that is what the
    external mesh gave it - so this pair is stated with that hand as its input,
    and the internal rule then hands the ring the same one again.
    """
    planet_hand = "left" if p.hand == "right" else "right"
    return _spur_params(
        p, p.z_planet, p.z_ring, internal=True, hand=planet_hand
    )


def compute_set(p: PlanetarySetParams) -> PlanetarySetGeometry:
    """Everything derived from the inputs, for all three members at once."""
    sun_planet = compute_spur_set(sun_planet_params(p))
    planet_ring = compute_spur_set(planet_ring_params(p))

    return PlanetarySetGeometry(
        params=p,
        centre_distance=sun_planet.centre_distance,
        transverse_module=sun_planet.transverse_module,
        transverse_pressure_angle=sun_planet.transverse_pressure_angle,
        circular_pitch=sun_planet.circular_pitch,
        axial_pitch=sun_planet.axial_pitch,
        whole_depth=sun_planet.whole_depth,
        sun=sun_planet.pinion,
        # The planet out of the sun-planet pair rather than the planet-ring one.
        # They are the same gear - same radii, same psi0, same twist magnitude -
        # but this one carries the hand the external mesh gave it, which is the
        # hand the part is actually cut with.
        planet=sun_planet.gear,
        ring=planet_ring.gear,
        sun_planet=sun_planet,
        planet_ring=planet_ring,
    )


def centre_distance_from_ring(p: PlanetarySetParams) -> float:
    """The planet-ring centre distance, solved the other way round.

    `m_t (z_r - z_p) / 2`. Equal to `params.centre_distance` by construction, and
    a test says so - which is the cheapest possible check that `z_ring` is right,
    because the two expressions share no terms.
    """
    return p.transverse_module * (p.z_ring - p.z_planet) / 2.0
