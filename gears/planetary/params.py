"""User-facing input parameters for a planetary (epicyclic) gear set.

Angles are in degrees here because this is what the GUI edits; everything
downstream of `geometry.compute_set` works in radians.

What is an input and what is not
--------------------------------
Three tooth counts describe a planetary set and only **two** of them are free.
The sun and the planets fix the ring:

    z_ring = z_sun + 2 * z_planet

because the sun-planet centre distance and the planet-ring centre distance are
the same distance measured twice - the planet sits on the line between them -
and with no profile shift that forces the relation exactly. So `z_ring` is a
derived property here rather than a field, and a set that "wants" a different
ring is asking for two different centre distances at once.

`n_planets` is free, but not entirely: the sun and ring have to present the same
tooth phase at every planet position, which needs `(z_sun + z_ring) % n == 0`.
That is the **assembly condition**, and it is checked rather than solved for -
the honest failure is to say which counts would work, not to quietly move a
planet off its station.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from ..params_io import JsonParams


@dataclass(frozen=True)
class PlanetarySetParams(JsonParams):
    """The editable inputs, plus the form factors kept out of the GUI."""

    module: float               # mm, NORMAL module
    z_sun: int
    z_planet: int
    n_planets: int = 3

    face_width: float = 20.0    # mm, along the axis; shared by all three members
    bore: float = 10.0          # mm, diameter - the SUN's; planets get their own
    hub_thickness: float = 5.0  # mm, boss behind the sun's back face
    rim_thickness: float = 5.0  # mm, standing outside the ring's root circle

    pressure_angle: float = 20.0   # degrees, NORMAL
    helix_angle: float = 0.0       # degrees; 0 is a straight-toothed train

    # Which way the SUN's teeth wind. The planets take the opposite hand
    # (external mesh) and the ring takes the planets' (internal mesh), so one
    # input settles all three - see `geometry.compute_set`.
    hand: str = "right"

    # Not exposed in the GUI, but part of the geometry.
    fillet_factor: float = 0.2
    backlash: float = 0.0

    @property
    def z_ring(self) -> int:
        """Forced by the two centre distances having to agree. Not an input."""
        return self.z_sun + 2 * self.z_planet

    @property
    def alpha_n(self) -> float:
        return math.radians(self.pressure_angle)

    @property
    def beta(self) -> float:
        """Helix angle in radians, signed by hand: right-hand is positive."""
        magnitude = math.radians(self.helix_angle)
        return magnitude if self.hand == "right" else -magnitude

    @property
    def transverse_module(self) -> float:
        return self.module / math.cos(self.beta)

    @property
    def alpha_t(self) -> float:
        return math.atan2(math.tan(self.alpha_n), math.cos(self.beta))

    @property
    def centre_distance(self) -> float:
        """Sun axis to planet axis, mm. Also the planets' orbit radius.

        The same number as the planet-to-ring centre distance, which is the
        whole reason `z_ring` is not free.
        """
        return self.transverse_module * (self.z_sun + self.z_planet) / 2.0

    @property
    def assembly_remainder(self) -> int:
        """Zero when the planets can all be fitted at equal spacing."""
        return (self.z_sun + self.z_ring) % self.n_planets

    @property
    def ratio_carrier_to_sun(self) -> float:
        """Sun turns per carrier turn with the **ring held**. 3.5 on the anchor set.

        `1 + z_ring / z_sun`, the textbook epicyclic reduction and the reason
        anyone builds one of these.
        """
        return 1.0 + self.z_ring / self.z_sun

    @property
    def ratio_ring_to_sun(self) -> float:
        """Ring turns per sun turn with the **carrier held**. Negative: they oppose.

        This is the configuration the v1 assembly is built in - every member
        spinning about a fixed axis - so it is the ratio a gear mate can express
        and the one worth reporting beside the other.
        """
        return -self.z_ring / self.z_sun

    @classmethod
    def with_defaults(cls, module: float, z_sun: int, z_planet: int, **overrides):
        """Build a set with sensible face width / bore / hub / rim for the size.

        The same rules of thumb the spur type uses, applied to the sun - it is
        the member with a bore and a hub, and the smallest of the three, so
        sizing against it is sizing against the tightest constraint.
        """
        beta = math.radians(overrides.get("helix_angle", 0.0))
        face_width = 10.0 * module
        if abs(math.sin(beta)) > 1e-9:
            face_width = max(face_width, math.pi * module / abs(math.sin(beta)))

        m_t = module / math.cos(beta)
        defaults = {
            "face_width": round(face_width, 2),
            "bore": round(max(0.25 * m_t * z_sun, 4.0), 1),
            "hub_thickness": round(2.5 * module, 2),
            "rim_thickness": round(2.5 * module, 2),
        }
        defaults.update(overrides)
        return cls(module=module, z_sun=z_sun, z_planet=z_planet, **defaults)
