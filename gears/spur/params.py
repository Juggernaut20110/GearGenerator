"""User-facing input parameters for an involute spur gear set.

Angles are in degrees here because this is what the GUI edits; everything
downstream of `geometry.compute_set` works in radians.

Normal or transverse?
---------------------
`module` and `pressure_angle` are the **normal** values - measured in the plane
perpendicular to the tooth, which is the plane a hob or a cutter works in. That
is the convention every catalogue and every cutter is sold under, and it is what
makes a helical gear cuttable with the same tool as a straight one of the same
normal module.

The involute itself, though, lives in the **transverse** plane - the plane
perpendicular to the axis, which is where a section through the part is taken.
So the geometry works throughout in

    m_t     = m_n / cos(beta)
    alpha_t = atan(tan(alpha_n) / cos(beta))

and both are exposed here so nothing downstream repeats the conversion. At
beta = 0 they collapse to the normal values and a straight spur gear falls out
of the same code with no special case.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from ..params_io import JsonParams


# Basic-rack coefficients.  These are dimensionless ISO quantities; the
# actual lengths are obtained by multiplying by the normal module m_n.
#
# The root-radius value is carried by the data model and is consumed by the
# opt-in external straight rack-generated root. The legacy mode deliberately
# continues to use ``fillet_factor`` for compatibility.
BASIC_RACK_ADDENDUM_FACTOR = 1.0       # h_aP*
BASIC_RACK_CLEARANCE_FACTOR = 0.25     # c_P*
BASIC_RACK_ROOT_RADIUS_FACTOR = 0.38   # rho_fP*
ROOT_GEOMETRY_MODES = ("legacy", "rack_generated")
TIP_ALTERATION_MODES = ("legacy", "iso_clearance", "explicit")
BACKLASH_MODES = ("legacy_reference", "working_circumferential", "normal")


@dataclass(frozen=True)
class SpurSetParams(JsonParams):
    """User inputs for an external or internal spur/helical gear pair.

    The profile-shift and basic-rack fields are additive ISO 21771/ISO 53
    data-model inputs.  ``root_geometry="legacy"`` remains the default so
    existing presets and tooth-space sections retain their previous shape.
    ``root_geometry="rack_generated"`` opts an external straight member into
    the verified ISO 53 basic-rack root envelope; internal and helical members
    retain the legacy path until their cutter sections are independently
    verified.
    """

    module: float               # mm, NORMAL module
    z1: int                     # pinion tooth count
    z2: int                     # gear tooth count
    face_width: float           # mm, along the axis
    bore: float                 # mm, diameter
    hub_thickness: float        # mm, boss behind the back face

    pressure_angle: float = 20.0   # degrees, NORMAL
    helix_angle: float = 0.0       # degrees; 0 is a straight spur gear

    # Which way the PINION's teeth wind, looking along +Z.
    #
    # For an **external** pair the gear takes the opposite hand: the two are
    # placed on parallel axes by a pure translation with no flip, so same-hand
    # teeth would cross rather than mesh.
    #
    # For an **internal** pair both members take the *same* hand, and the reason
    # is the same reason: there is still no flip in the placement, but the ring
    # is not turned over either - the pinion runs inside it, both teeth face the
    # same way round the axis, and opposite hands would cross. See
    # `geometry.compute_set`, which signs each member's twist for this.
    hand: str = "right"

    # Whether z2 is an internal ring gear rather than an external one.
    internal: bool = False

    # Not exposed in the GUI, but part of the geometry.
    fillet_factor: float = 0.2  # root fillet radius as a multiple of module

    # Backlash input.  The meaning is explicit in `backlash_mode` below.
    # `legacy_reference` preserves the historical JSON/API behavior: the
    # requested j_t is removed from the two reference-circle tooth widths,
    # half from each member.  It remains the default so old presets are
    # deterministic and do not silently change geometry.
    backlash: float = 0.0

    # Rim standing outside a ring gear's root circle, mm. Ignored for an
    # external pair, which has no rim - its blank ends at the tip circle.
    rim_thickness: float = 5.0

    # ISO 21771 profile-shift coefficients x_1 and x_2.  They are
    # dimensionless and are measured in units of the normal module.  The
    # physical positive-radius convention used by this repository is kept;
    # internal ISO signed-count conversion belongs at the geometry boundary.
    profile_shift_1: float = 0.0
    profile_shift_2: float = 0.0

    # ISO 53 basic-rack coefficients.  The dedendum coefficient is derived as
    # h_fP* = h_aP* + c_P* so one source of truth cannot drift from the other.
    basic_rack_addendum_factor: float = BASIC_RACK_ADDENDUM_FACTOR  # h_aP*
    basic_rack_clearance_factor: float = BASIC_RACK_CLEARANCE_FACTOR  # c_P*
    basic_rack_root_radius_factor: float = BASIC_RACK_ROOT_RADIUS_FACTOR  # rho_fP*

    # Optional working centre distance a_w.  None means the working distance
    # is derived from x_1+x_2 externally or x_2-x_1 internally.  It is
    # represented here as an input so a later validator can detect a conflict
    # between an explicit a_w and the two profile-shift coefficients.
    working_centre_distance: float | None = None

    # Pair-level ISO 21771-1 tip alteration. New parameter sets use the
    # standards-based estimate from Clause 5.3.9. ``legacy`` keeps the old
    # k=0 geometry for presets written before this field existed; the JSON
    # migration below selects it explicitly for such files. ``explicit`` uses
    # the supplied coefficient and is intentionally separate from x_i.
    tip_alteration_mode: str = "iso_clearance"
    tip_alteration_coefficient: float | None = None

    # ``legacy`` preserves the existing radial-below-base/root-fillet
    # approximation.  ``rack_generated`` is an explicit opt-in for the
    # external straight-gear rack/cutter envelope.  It is intentionally not a
    # silent default: the helical transverse projection and internal cutter
    # have separate generation geometry that is not yet verified here.
    root_geometry: str = "legacy"

    # ISO 21771-1:2024 §5.6 / ISO 21771-2:2025 Clause 13 terminology.  The
    # `normal` mode accepts normal-base backlash j_bn; the conversion to the
    # working circumferential quantity is performed only after alpha_wt and
    # the working helix geometry are known.  `backlash_allocation` is the
    # fraction assigned to member 1 (the pinion) in the two-member tooth
    # thickness solution.  0.5 is an explicit equal split, not an ISO
    # requirement.
    backlash_mode: str = "legacy_reference"
    backlash_allocation: float = 0.5

    # --- radian and transverse accessors, so downstream code never repeats them ---

    @property
    def alpha_n(self) -> float:
        """Normal pressure angle in radians."""
        return math.radians(self.pressure_angle)

    @property
    def beta(self) -> float:
        """Helix angle in radians, signed by hand: right-hand is positive."""
        magnitude = math.radians(self.helix_angle)
        return magnitude if self.hand == "right" else -magnitude

    @property
    def transverse_module(self) -> float:
        """Transverse module m_t = m_n / cos(beta), in millimetres."""
        return normal_to_transverse_module(self.module, self.beta)

    @property
    def alpha_t(self) -> float:
        """Reference transverse pressure angle alpha_t, in radians."""
        return normal_to_transverse_pressure_angle(self.alpha_n, self.beta)

    @property
    def reference_pressure_angle(self) -> float:
        """Clear-name alias for the transverse reference pressure angle alpha_t."""
        return self.alpha_t

    @property
    def ratio(self) -> float:
        return self.z2 / self.z1

    @property
    def reference_centre_distance(self) -> float:
        """Reference centre distance a, in millimetres.

        This is fixed by the reference diameters d = m_t * |z|.  It is not a
        working distance when profile shift or an explicit working distance is
        present.
        """
        counts = self.z2 - self.z1 if self.internal else self.z1 + self.z2
        return self.transverse_module * counts / 2.0

    @property
    def profile_shift_combination(self) -> float:
        """The pair's ISO working shift combination X.

        External pairs use X = x_1 + x_2.  With the repository's positive
        ring tooth count, internal pairs use X = x_2 - x_1.
        """
        return (
            self.profile_shift_2 - self.profile_shift_1
            if self.internal
            else self.profile_shift_1 + self.profile_shift_2
        )

    @property
    def basic_rack_dedendum_factor(self) -> float:
        """Basic-rack dedendum coefficient h_fP* = h_aP* + c_P*."""
        return self.basic_rack_addendum_factor + self.basic_rack_clearance_factor

    @property
    def h_aP_star(self) -> float:
        """ISO symbol alias for ``basic_rack_addendum_factor``."""
        return self.basic_rack_addendum_factor

    @property
    def h_fP_star(self) -> float:
        """ISO symbol alias for the derived basic-rack dedendum h_fP*."""
        return self.basic_rack_dedendum_factor

    @property
    def c_P_star(self) -> float:
        """ISO symbol alias for ``basic_rack_clearance_factor``."""
        return self.basic_rack_clearance_factor

    @property
    def rho_fP_star(self) -> float:
        """ISO symbol alias for ``basic_rack_root_radius_factor``."""
        return self.basic_rack_root_radius_factor

    @property
    def centre_distance(self) -> float:
        """Compatibility alias for :attr:`reference_centre_distance`.

        The **difference** of the tooth counts for an internal pair, not the
        sum. The pinion runs inside the ring, so moving teeth from one to the
        other brings the axes together instead of pushing them apart, and at
        z1 = z2 the two axes coincide - which is why the validator wants a
        healthy gap between the counts.

        ``SpurSetGeometry.centre_distance`` is the working-distance alias;
        this parameter-level property is retained as the old reference-only
        quantity because parameters do not contain derived member radii.
        """
        return self.reference_centre_distance

    @classmethod
    def _migrate_json_data(cls, data: dict):
        """Map short/transitional ISO spellings to canonical JSON names."""
        migrated = dict(data)
        aliases = {
            "profile_shift1": "profile_shift_1",
            "profile_shift2": "profile_shift_2",
            "x1": "profile_shift_1",
            "x2": "profile_shift_2",
            "h_aP_star": "basic_rack_addendum_factor",
            "c_P_star": "basic_rack_clearance_factor",
            "rho_fP_star": "basic_rack_root_radius_factor",
        }
        for old_name, new_name in aliases.items():
            if new_name not in migrated and old_name in migrated:
                migrated[new_name] = migrated[old_name]
        # A pre-tip-alteration preset must remain reproducible. New callers of
        # ``with_defaults`` get automatic ISO clearance, while an old JSON
        # file opts into the explicitly named compatibility path.
        if "tip_alteration_mode" not in migrated:
            migrated["tip_alteration_mode"] = "legacy"
        # A pre-ISO-21771-2 backlash field was a reference-circle allowance.
        # Keep that meaning when loading old JSON rather than interpreting it
        # as working circumferential backlash.
        if "backlash_mode" not in migrated:
            migrated["backlash_mode"] = "legacy_reference"
        return migrated

    @classmethod
    def with_defaults(cls, module: float, z1: int, z2: int, **overrides):
        """Build a set with sensible face width / bore / hub for the given size.

        Face width is the usual 10 * m_n for straight teeth. For helical teeth it
        is at least one **axial pitch** as well, `pi * m_n / sin(beta)`, so the
        axial contact ratio reaches 1: below that the pair pays the thrust-load
        cost of a helix without buying the overlapping handover that is the whole
        point of one. The bore and hub are rules of thumb. Any of them can be
        overridden.
        """
        beta = math.radians(overrides.get("helix_angle", 0.0))
        face_width = 10.0 * module
        if abs(math.sin(beta)) > 1e-9:
            face_width = max(face_width, math.pi * module / abs(math.sin(beta)))

        # The bore is sized against the transverse pitch diameter, which is the
        # one the blank is actually as big as.
        m_t = module / math.cos(beta)

        defaults = {
            "face_width": round(face_width, 2),
            "bore": round(max(0.25 * m_t * z1, 4.0), 1),
            "hub_thickness": round(2.5 * module, 2),
            # The rim behind a ring gear's teeth. The same rule of thumb as the
            # hub, and comfortably over the one-module minimum the validator
            # asks for. Only read when `internal` is set.
            "rim_thickness": round(2.5 * module, 2),
        }
        defaults.update(overrides)
        return cls(module=module, z1=z1, z2=z2, **defaults)


def normal_to_transverse_module(module: float, beta: float) -> float:
    """Convert normal module m_n to transverse module m_t."""
    return module / math.cos(beta)


def normal_to_transverse_pressure_angle(alpha_n: float, beta: float) -> float:
    """Convert normal pressure angle alpha_n to transverse alpha_t."""
    return math.atan2(math.tan(alpha_n), math.cos(beta))
