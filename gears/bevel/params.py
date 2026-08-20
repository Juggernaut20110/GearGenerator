"""User-facing input parameters for a bevel gear set, straight or spiral.

Angles are in degrees here because this is what the GUI edits; everything
downstream of `geometry.compute_set` works in radians.

Straight or spiral?
-------------------
`spiral_angle` is the **mean** spiral angle - measured at the mean cone
distance Am, which is where the Gleason system quotes it and where the cutter
is set. Zero is a straight bevel gear, and it is not a special case anywhere
downstream: it makes the tooth trace a straight radial line, every section's
phase zero, and the same code path builds both.

`cutter_radius` is the radius of the face-milling cutter that swept the trace.
Left as None it defaults to Am, which is the nominal Gleason practice and the
value that makes the arc's curvature match the gear it is cutting. It matters
because the trace is a real circular arc rather than a curve of constant
spiral angle: the spiral angle it actually delivers varies along the face, and
the cutter radius is what decides by how much. On the anchor set at 35 degrees
mean it runs 30.074 at the toe to 40.599 at the heel.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from ..params_io import JsonParams


@dataclass(frozen=True)
class BevelSetParams(JsonParams):
    """The nine editable inputs, plus two form factors kept out of the GUI."""

    module: float               # mm, transverse module at the outer (large) end
    z1: int                     # pinion tooth count
    z2: int                     # gear tooth count
    face_width: float           # mm, along the pitch cone
    bore: float                 # mm, diameter
    hub_thickness: float        # mm, backing behind the outer root point

    # Axial rim kept behind the outer root point, before the flat back begins.
    #
    # Without it the flat back passes exactly through that point, so the material
    # under the tooth root tapers to nothing at the heel. The wedge it leaves has
    # an included angle of 90 - root_angle, which is a harmless 70 degrees on a
    # 17-tooth pinion but a 25 degree feather edge on its 43-tooth mate - the
    # bigger the ratio, the thinner the gear's heel. Setting this to zero
    # restores the old outline exactly.
    min_root_thickness: float = 0.5    # mm, measured along the axis

    pressure_angle: float = 20.0   # degrees
    shaft_angle: float = 90.0      # degrees

    spiral_angle: float = 0.0      # degrees, MEAN; 0 is a straight bevel gear

    # Which way the PINION's teeth curve, looking along +Z from the apex. The
    # gear always takes the opposite hand, and unlike the spur case that is not
    # a rule this module has to impose - both members map the *same* arc off the
    # generating crown gear, and two different pitch angles is all it takes to
    # make one left hand and the other right. See geometry.CrownTrace.
    hand: str = "right"

    # Face-milling cutter radius, mm. None means "the Gleason nominal", Am.
    cutter_radius: float | None = None

    # Not exposed in the GUI, but part of the geometry.
    fillet_factor: float = 0.2  # root fillet radius as a multiple of module

    # Circular backlash, mm, split evenly between the members so the mesh sees
    # the stated number once. Taken off the tooth rather than added to the
    # mounting distance, which keeps the cones where they are.
    #
    # **Quoted at the outer end**, like the module it is measured against. The
    # tooth thickness it comes off is the one on the outer back cone, and every
    # inner section is a uniform scaling of that one - so the backlash a section
    # actually carries tapers with k = A/Ao toward the toe, the same way every
    # other length on the tooth does. That is what a cutter leaves, not an
    # approximation of it.
    backlash: float = 0.0

    # --- radian accessors, so downstream code never repeats the conversion ---

    @property
    def alpha(self) -> float:
        """Pressure angle in radians."""
        return math.radians(self.pressure_angle)

    @property
    def sigma(self) -> float:
        """Shaft angle in radians."""
        return math.radians(self.shaft_angle)

    @property
    def psi_m(self) -> float:
        """Mean spiral angle in radians, signed by hand: right-hand is positive.

        Signed the same way `SpurSetParams.beta` is, and for the same reason -
        the sign is what the geometry needs and the hand is what a person says.
        """
        magnitude = math.radians(self.spiral_angle)
        return magnitude if self.hand == "right" else -magnitude

    @property
    def trace_sign(self) -> float:
        """Which way the trace bows: +1 right hand, -1 left. The hand, as a number.

        **Not `sign(psi_m)`, and the difference is exactly Zerol.** A Zerol set
        has `psi_m == 0.0` for both hands - and `-0.0 < 0.0` is False - so
        reading the bow off the angle landed every Zerol tooth on the same side
        whatever `hand` said, which made the input inert on the one kind of set
        where it is the only thing left to say. The hand string still carries a
        direction at zero, so at zero it is the one asked.

        Only at zero, though. A negative `spiral_angle` typed in directly is
        permitted by the validator and has to keep meaning what it already meant,
        so above zero the angle's own sign still wins - otherwise
        `spiral_angle_at` would hand back +35 degrees for a set built at -35.
        """
        if abs(self.spiral_angle) > 1e-12:
            return -1.0 if self.psi_m < 0.0 else 1.0
        return 1.0 if self.hand == "right" else -1.0

    @property
    def is_curved(self) -> bool:
        """Whether the tooth trace curves at all, which is the geometry's switch.

        Not the same question as "is the spiral angle zero". A **Zerol** gear has
        a mean spiral angle of exactly zero and a curved tooth all the same: the
        arc crosses Am radially and bends away from radial on either side of it.
        So a cutter radius given alongside a zero spiral angle is a Zerol set and
        gets the curved code path.

        `with_defaults` only fills in a cutter radius when there is a spiral, so
        a plain straight bevel set still arrives with None here and takes the
        straight path - which is what keeps its output identical to what this
        module produced before the trace existed.
        """
        return abs(self.spiral_angle) > 1e-12 or self.cutter_radius is not None

    @property
    def trace_kind(self) -> str:
        """What to call this tooth: "straight", "zerol" or "spiral".

        Three names off two booleans, and the pairing is the point: "spiral angle
        0" describes a straight gear and a Zerol one equally well, so a report
        that only prints the angle cannot tell them apart. Shared by the terminal
        report and the GUI status line so they cannot drift.
        """
        if not self.is_curved:
            return "straight"
        return "spiral" if abs(self.spiral_angle) > 1e-12 else "zerol"

    @property
    def ratio(self) -> float:
        return self.z2 / self.z1

    @classmethod
    def with_defaults(
        cls, module: float, z1: int, z2: int, *, zerol: bool = False, **overrides
    ):
        """Build a set with sensible face width / bore / hub for the given size.

        Face width follows the usual bevel limit of min(Ao/3, 10*m); the bore,
        hub and root rim are rules of thumb that keep the blank manufacturable.
        Any of them can be overridden.

        **A spiral set gets a narrower face**, 0.30*Ao rather than Ao/3 - which
        is the Gleason limit and is tighter for a reason worth stating: a spiral
        tooth's contact sweeps along the face as well as up the profile, so the
        toe end of a wide face carries load at a spiral angle well away from the
        one it was designed at. On the anchor set that is 13.87 mm against 15.41.

        The cutter radius defaults to Am, the mean cone distance. That is the
        Gleason nominal, and it is the choice that puts the arc's own curvature
        on the same order as the gear's - a much smaller cutter swings the spiral
        angle wildly across the face, and a much larger one approaches a straight
        tooth laid at an angle.

        `zerol=True` asks for a curved tooth at a zero mean spiral angle. It is a
        **sizing hint and not a field** - it never reaches the constructor. All it
        does is answer the `curved` question below the way a cutter radius would
        have, which then fills the cutter radius in with Am; the set that comes
        back reports `is_curved` on its own, off that radius, exactly as a set
        built by naming the radius directly does. That is deliberate: there is
        one definition of curved and it lives in `is_curved`.
        """
        # Ao needs the pitch angle, which needs the shaft angle - resolve it here
        # rather than importing geometry (which would be a circular import).
        sigma = math.radians(overrides.get("shaft_angle", 90.0))
        delta1 = math.atan2(math.sin(sigma), z2 / z1 + math.cos(sigma))
        outer_cone_dist = module * z1 / (2.0 * math.sin(delta1))

        # The same question `is_curved` asks, and it has to be the same question:
        # a Zerol set has a zero spiral angle and a curved tooth, so sizing it as
        # a straight one would hand it a face width the validator then complains
        # about.
        curved = (
            zerol
            or abs(overrides.get("spiral_angle", 0.0)) > 1e-12
            or overrides.get("cutter_radius") is not None
        )
        cone_fraction = 0.30 if curved else 1.0 / 3.0
        face_width = round(
            min(cone_fraction * outer_cone_dist, 10.0 * module), 2
        )

        defaults = {
            "face_width": face_width,
            "bore": round(max(0.25 * module * z1, 4.0), 1),
            "hub_thickness": round(2.5 * module, 2),
            "min_root_thickness": round(0.25 * module, 2),
        }
        defaults.update(overrides)
        if defaults.get("cutter_radius") is None and curved:
            # Am, measured against whatever face width survived the overrides.
            defaults["cutter_radius"] = round(
                outer_cone_dist - defaults["face_width"] / 2.0, 4
            )
        return cls(module=module, z1=z1, z2=z2, **defaults)

    # Preset save/load for the GUI comes from JsonParams.
