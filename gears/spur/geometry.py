"""Involute spur gear geometry, straight and helical. Pure math - no COM, no GUI.

Units are millimetres and radians throughout. The SOLIDWORKS layer converts to
metres at its own boundary; nothing here knows about that.

3D coordinate system
--------------------
The gear axis is +Z and the **front face sits at z = 0**, with the body running
to z = face_width. A tooth **space** is centred on +X *at z = 0*, so z = 0 is the
clocking reference plane for the whole pair - the plane a mating pinion and gear
are phased against. Every other transverse section is that one rotated by the
twist that has accumulated by its own z.

Straight teeth are not a special case anywhere in this module. They are
helix_angle = 0, which makes the twist zero and every section identical; the
same code path builds both.

Tooth form
----------
A true involute in the **transverse** plane, from `gears.involute`. The inputs
that reach the flank are the transverse module and pressure angle, so a helical
gear differs from a straight one only in that those two are larger:

    m_t     = m_n / cos(beta)
    alpha_t = atan(tan(alpha_n) / cos(beta))

Proportions are the plain ISO ones on the **normal** module - addendum 1.0 m_n,
dedendum 1.25 m_n in the default rack. For an external pair, profile shift
changes those member-specific depths and the reference tooth thickness. In a
helical pair the tooth thickness is first calculated in the normal plane and
then projected into the transverse plane; the involute still consumes the
transverse result. An internal ring applies the same normal tooth-thickness
equation to its tooth, then gives the complementary transverse space width to
the internal involute generator.

The twist, and why the gear is wound the other way
--------------------------------------------------
Over the face width a helical tooth sweeps

    twist = face_width * tan(beta) / reference_r

about the axis. `beta` is signed by hand, so the twist is too. An external pair
carries opposite signs, while an internal pair carries the same sign; those are
the mesh hand rules for the two arrangements. The gear is placed on its parallel
axis by a pure translation with no flip, so using the wrong rule would make the
helical teeth cross instead of meshing.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from ..involute import (
    CUT_OVERSHOOT_FACTOR,
    FLANK_POINTS,
    MIN_TOP_LAND_FACTOR,
    inv,
    max_tip_radius,
    min_internal_tip_radius,
    rack_generated_root,
    solve_start_of_involute,
    tooth_space_loop,
)
from .params import (
    BASIC_RACK_ADDENDUM_FACTOR,
    BASIC_RACK_CLEARANCE_FACTOR,
    SpurSetParams,
)

Point2 = tuple[float, float]
Point3 = tuple[float, float, float]

# ISO standard proportions, as multiples of the NORMAL module.
ADDENDUM_FACTOR = BASIC_RACK_ADDENDUM_FACTOR
DEDENDUM_FACTOR = ADDENDUM_FACTOR + BASIC_RACK_CLEARANCE_FACTOR
WHOLE_DEPTH_FACTOR = ADDENDUM_FACTOR + DEDENDUM_FACTOR

# How far past each end of the face width the loft sections are pushed.
#
# Same reason as the bevel builder's: a cut that finishes tangent to a real face
# of the blank is rejected as zero-thickness geometry, and both ends of a spur
# blank are real faces. Unlike the bevel case this needs no search - the faces
# are planes perpendicular to the axis, so any positive overshoot clears them.
END_OVERSHOOT_FRACTION = 0.05
END_OVERSHOOT_MIN_MM = 0.5

# How far a lofted flank may fall inside the true helicoid, in mm.
#
# A loft carries each profile point from one section to the next along a
# straight chord, and a helix is an arc, so the swept surface sits inside the
# helicoid by the chord's sagitta - r * (1 - cos(delta / 2)) at radius r over a
# twist of delta. The cut comes out shallower than it should, and the flank is
# the wrong shape.
#
# Measured on the anchor helical pinion (m_n=2, 17 teeth, 15 deg) built from two
# sections and a guide curve: the mid-face profile stood 0.198 mm proud of the
# end profile at the same relative angle, against a predicted sagitta of 0.287
# mm at that radius. So the guide helps and does not fix it - it pins only the
# one point it passes through, and everything else is still interpolated.
#
# 0.02 mm is an order below any tooth tolerance that matters and costs only a
# few extra sketches.
MAX_SECTION_SAGITTA_MM = 0.02


# ---------------------------------------------------------------------------
# Derived geometry
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class SpurMemberGeometry:
    """Derived geometry for one member (pinion or gear) of the set."""

    name: str
    z: int
    beta: float                 # signed helix angle, radians; this member's hand
    reference_r: float          # d / 2, ISO reference-circle radius
    base_r: float               # d_b / 2, ISO base-circle radius
    working_r: float            # d_w / 2, ISO working pitch-circle radius
    tip_r: float                # SMALLEST radius of the teeth if internal
    root_r: float               # LARGEST radius of the teeth if internal
    addendum: float
    dedendum: float
    geometric_tooth_thickness: float  # s_t before deliberate backlash thinning
    reference_tooth_thickness: float  # s_t after the compatibility backlash split
    normal_geometric_tooth_thickness: float  # s_n before backlash
    normal_tooth_thickness: float           # s_n after backlash
    virtual_teeth: float        # z / cos(beta)**3, the equivalent spur gear
    twist: float                # total rotation over the face width, radians
    psi0: float                 # angular half-thickness constant at the base
    half_pitch: float           # pi / z
    internal: bool = False      # a ring gear: teeth pointing inward
    profile_shift: float = 0.0  # x_i, dimensionless ISO profile shift
    generated_root_r: float | None = None  # d_fE / 2, generated root boundary
    root_form_r: float | None = None  # d_Ff / 2, actual SOI/root-form radius
    start_of_involute_angle: float | None = None
    involute_roll_parameter: float | None = None
    undercut: bool | None = None

    @property
    def pitch_r(self) -> float:
        """Deprecated compatibility alias for ``reference_r`` (d / 2).

        This name has historically meant the reference pitch circle.  It must
        not be repurposed for ``working_r``.
        """
        return self.reference_r

    @property
    def working_pitch_r(self) -> float:
        """Clear-name alias for ``working_r`` (d_w / 2)."""
        return self.working_r

    @property
    def tooth_thickness(self) -> float:
        """Compatibility alias for actual transverse reference thickness s_t."""
        return self.reference_tooth_thickness

    @property
    def transverse_tooth_thickness(self) -> float:
        """Clear-name alias for actual transverse reference thickness s_t."""
        return self.reference_tooth_thickness

    @property
    def reference_d(self) -> float:
        """Reference diameter d = 2 * reference_r, in millimetres."""
        return 2.0 * self.reference_r

    @property
    def working_d(self) -> float:
        """Working pitch diameter d_w = 2 * working_r, in millimetres."""
        return 2.0 * self.working_r

    @property
    def base_d(self) -> float:
        """Base diameter d_b = 2 * base_r, in millimetres."""
        return 2.0 * self.base_r

    @property
    def tip_d(self) -> float:
        """Nominal physical tip diameter d_a = 2 * tip_r, in millimetres."""
        return 2.0 * self.tip_r

    @property
    def root_d(self) -> float:
        """Nominal physical root diameter d_f = 2 * root_r, in millimetres."""
        return 2.0 * self.root_r

    @property
    def generated_root_d(self) -> float | None:
        """Generated root diameter d_fE = 2 * generated_root_r.

        ``None`` means the selected profile is the legacy radial/root-fillet
        approximation or that no verified generated-root construction applies
        to this member.  It is intentionally separate from nominal ``root_d``
        and from ``root_form_d`` / ``d_Ff``.
        """
        return (
            None
            if self.generated_root_r is None
            else 2.0 * self.generated_root_r
        )

    @property
    def root_form_d(self) -> float | None:
        """Root-form diameter ``d_Ff = 2*r_Ff`` when generated geometry exists."""
        return None if self.root_form_r is None else 2.0 * self.root_form_r

    @property
    def start_of_involute_r(self) -> float | None:
        """ISO start-of-involute radius, separate from nominal ``root_r``."""
        return self.root_form_r

    @property
    def start_of_involute_d(self) -> float | None:
        return self.root_form_d

    @property
    def hand(self) -> str:
        if abs(self.beta) < 1e-12:
            return "none"
        return "right" if self.beta > 0.0 else "left"

    @property
    def helix_angle_deg(self) -> float:
        return math.degrees(self.beta)

    @property
    def twist_deg(self) -> float:
        return math.degrees(self.twist)

    @property
    def angular_pitch(self) -> float:
        return 2.0 * math.pi / self.z

    @property
    def outside_dia(self) -> float:
        """How wide the *teeth* reach, which for a ring gear is its root circle.

        Not the blank's outside diameter when the gear is internal - the rim
        stands outside the root - so `blank_outline` is the thing to ask about
        the part's overall size. This is the diameter the teeth occupy.
        """
        return 2.0 * (self.root_r if self.internal else self.tip_r)


@dataclass(frozen=True)
class SpurSetGeometry:
    params: SpurSetParams
    reference_centre_distance: float  # a, from reference diameters
    working_centre_distance: float    # a_w, from working pitch circles
    reference_pressure_angle: float   # alpha_t, transverse reference angle
    working_pressure_angle: float     # alpha_wt, transverse working angle
    transverse_module: float
    circular_pitch: float           # transverse, at the pitch circle
    axial_pitch: float              # inf for straight teeth
    whole_depth: float
    transverse_contact_ratio: float
    axial_contact_ratio: float
    pinion: SpurMemberGeometry
    gear: SpurMemberGeometry

    @property
    def centre_distance(self) -> float:
        """Compatibility alias for the working centre distance a_w."""
        return self.working_centre_distance

    @property
    def transverse_pressure_angle(self) -> float:
        """Compatibility alias for the transverse reference angle alpha_t."""
        return self.reference_pressure_angle

    @property
    def overlap_ratio(self) -> float:
        """ISO-named alias for the axial overlap ratio epsilon_beta."""
        return self.axial_contact_ratio

    @property
    def centre_distance_modification(self) -> float:
        """Centre-distance modification y = (a_w - a) / m_n."""
        return (
            self.working_centre_distance - self.reference_centre_distance
        ) / self.params.module

    @property
    def total_contact_ratio(self) -> float:
        return self.transverse_contact_ratio + self.axial_contact_ratio

    def member(self, which: str) -> SpurMemberGeometry:
        if which == "pinion":
            return self.pinion
        if which == "gear":
            return self.gear
        raise ValueError(f"member must be 'pinion' or 'gear', got {which!r}")


def compute_set(p: SpurSetParams) -> SpurSetGeometry:
    """Everything derived from the inputs, for both members at once."""
    m_n = p.module
    m_t = p.transverse_module
    alpha_t = p.alpha_t
    reference_centre_distance = p.reference_centre_distance
    working_pressure_angle, working_centre_distance = _working_geometry(
        p, reference_centre_distance, alpha_t
    )
    # Profile shift is supported for both pair arrangements. The coefficients
    # x_i are normal quantities; the transverse tooth thickness below is their
    # projection into the plane in which the involute is built. Internal and
    # external depth signs are handled explicitly in the member branch.
    rack_addendum = p.basic_rack_addendum_factor
    rack_dedendum = p.basic_rack_dedendum_factor

    # Circular tooth thickness at the reference circle, measured in the transverse
    # plane. Backlash is taken off the tooth, which is the convention that keeps
    # the centre distance nominal.
    standard_geometric_thickness = math.pi * m_t / 2.0
    # The gear's hand. An external pair is cut with opposite hands and an
    # internal pair with the same one - see `SpurSetParams.hand`. Neither is a
    # convention: the placement has no flip in it either way, and what differs
    # is whether the second member's teeth face the first from outside or from
    # inside.
    gear_beta = p.beta if p.internal else -p.beta

    members = []
    for name, z, beta, internal in (
        ("pinion", p.z1, p.beta, False),
        ("gear", p.z2, gear_beta, p.internal),
    ):
        reference_r = m_t * z / 2.0
        base_r = reference_r * math.cos(alpha_t)
        # The working circle is derived from the base circle and alpha_wt.  The
        # reference circle remains d/2 even when profile shift changes the
        # working centre distance.
        working_r = (
            reference_r
            if working_pressure_angle == alpha_t
            else base_r / math.cos(working_pressure_angle)
        )

        member_shift = p.profile_shift_2 if name == "gear" else p.profile_shift_1
        # ISO reference tooth thickness for either member. Profile shift x_i is
        # defined in the normal system, so the normal form is
        # s_n = m_n * (pi/2 + 2*x_i*tan(alpha_n)); the involute needs its
        # transverse projection s_t = s_n / cos(beta). Backlash is a separate
        # deliberate reduction, split symmetrically as before.
        if member_shift == 0.0:
            geometric_thickness = standard_geometric_thickness
        else:
            geometric_thickness = m_t * (
                math.pi / 2.0 + 2.0 * member_shift * math.tan(p.alpha_n)
            )
        reference_tooth_thickness = geometric_thickness - p.backlash / 2.0
        normal_geometric_thickness = geometric_thickness * math.cos(p.beta)
        normal_tooth_thickness = reference_tooth_thickness * math.cos(p.beta)

        if internal:
            # The ring uses the opposite physical depth signs from an
            # external member: positive x2 removes material from its inward
            # addendum and adds it to the outward dedendum.
            addendum = m_n * (rack_addendum - member_shift)
            dedendum = m_n * (rack_dedendum + member_shift)
        else:
            addendum = m_n * (rack_addendum + member_shift)
            dedendum = m_n * (rack_dedendum - member_shift)

        if internal:
            # The teeth point inward, so the addendum comes off the pitch radius
            # and the dedendum is added to it. Everything downstream keeps
            # calling the innermost radius the tip, because that is what it is -
            # the end of the tooth.
            tip_r = reference_r - addendum
            root_r = reference_r + dedendum
            # The ring involute generates a tooth SPACE, not a tooth. Its
            # reference space width is the complement of this ring tooth's
            # transverse thickness; that is the internal-specific angular
            # quantity which must be carried back to the base circle.
            space_width = math.pi * m_t - reference_tooth_thickness
            psi0 = space_width / (2.0 * reference_r) + inv(alpha_t)
        else:
            tip_r = reference_r + addendum
            root_r = reference_r - dedendum
            # psi0 is the angular half-thickness of the tooth extrapolated back
            # to the base circle; half_pitch is half the angular pitch. Together
            # they are all the flank generator needs, and both are scale-free.
            psi0 = reference_tooth_thickness / (2.0 * reference_r) + inv(alpha_t)

        half_pitch = math.pi / z
        generated_root_r = None
        root_form_r = None
        start_of_involute_angle = None
        involute_roll_parameter = None
        undercut = None
        if (
            p.root_geometry == "rack_generated"
            and not internal
            and abs(beta) <= 1e-12
        ):
            generated = rack_generated_root(
                reference_r=reference_r,
                r_base=base_r,
                r_root=root_r,
                psi0=psi0,
                half_pitch=half_pitch,
                cutter_tip_depth=dedendum,
                rack_root_radius=p.basic_rack_root_radius_factor * m_n,
            )
            if generated is not None:
                soi = solve_start_of_involute(generated)
                if soi is not None and tip_r > soi.start_of_involute_r + 1e-10:
                    generated_root_r = generated.generated_root_r
                    root_form_r = soi.start_of_involute_r
                    start_of_involute_angle = soi.start_of_involute_angle
                    involute_roll_parameter = soi.involute_roll_parameter
                    undercut = soi.undercut

        members.append(
            SpurMemberGeometry(
                name=name,
                z=z,
                beta=beta,
                reference_r=reference_r,
                base_r=base_r,
                working_r=working_r,
                tip_r=tip_r,
                root_r=root_r,
                addendum=addendum,
                dedendum=dedendum,
                geometric_tooth_thickness=geometric_thickness,
                reference_tooth_thickness=reference_tooth_thickness,
                normal_geometric_tooth_thickness=normal_geometric_thickness,
                normal_tooth_thickness=normal_tooth_thickness,
                virtual_teeth=z / math.cos(beta) ** 3,
                # The + 0.0 turns the gear's -0.0 into 0.0 when there is no
                # helix. Harmless arithmetically, but -0.0 prints as "-0.0000"
                # in the report and reads as a real, tiny, negative twist.
                twist=p.face_width * math.tan(beta) / reference_r + 0.0,
                psi0=psi0,
                half_pitch=half_pitch,
                internal=internal,
                profile_shift=member_shift,
                generated_root_r=generated_root_r,
                root_form_r=root_form_r,
                start_of_involute_angle=start_of_involute_angle,
                involute_roll_parameter=involute_roll_parameter,
                undercut=undercut,
            )
        )

    pinion, gear = members
    sin_beta = abs(math.sin(p.beta))

    return SpurSetGeometry(
        params=p,
        reference_centre_distance=reference_centre_distance,
        working_centre_distance=working_centre_distance,
        reference_pressure_angle=alpha_t,
        working_pressure_angle=working_pressure_angle,
        transverse_module=m_t,
        circular_pitch=math.pi * m_t,
        axial_pitch=(math.pi * m_n / sin_beta) if sin_beta > 1e-12 else math.inf,
        whole_depth=(rack_addendum + rack_dedendum) * m_n,
        transverse_contact_ratio=transverse_contact_ratio(
            pinion,
            gear,
            working_centre_distance,
            working_pressure_angle,
            math.pi * m_t,
            base_pitch_angle=alpha_t,
        ),
        axial_contact_ratio=(
            p.face_width * sin_beta / (math.pi * m_n) if sin_beta > 1e-12 else 0.0
        ),
        pinion=pinion,
        gear=gear,
    )


def _inverse_involute(value: float) -> float:
    """Return alpha in [0, pi/2) for ``inv(alpha) == value``.

    The working-pressure-angle equation is monotone on the physical domain.
    A bounded bisection keeps this foundation independent of numerical/scipy
    dependencies and gives a clear error for a non-physical requested shift.
    """
    if value < -1e-14:
        raise ValueError("working involute is below the physical alpha = 0 domain")
    if abs(value) <= 1e-14:
        return 0.0

    lo = 0.0
    hi = math.nextafter(math.pi / 2.0, 0.0)
    if inv(hi) < value:
        raise ValueError("working involute is outside the physical alpha domain")
    for _ in range(80):
        mid = (lo + hi) / 2.0
        if inv(mid) < value:
            lo = mid
        else:
            hi = mid
    return (lo + hi) / 2.0


def _working_geometry(
    p: SpurSetParams,
    reference_centre_distance: float,
    reference_pressure_angle: float,
) -> tuple[float, float]:
    """Resolve ``(alpha_wt, a_w)`` without changing tooth-space generation.

    With no explicit working distance, this uses the verified ISO-aligned
    involute relation ``inv(alpha_wt) = inv(alpha_t) +
    2*X*tan(alpha_n)/q`` and its corresponding base-circle distance.  An
    explicit working distance is treated as authoritative for this derived
    pair view; validation can compare it with the requested shifts.
    """
    if p.working_centre_distance is not None:
        working_distance = p.working_centre_distance
        if working_distance <= 0.0:
            raise ValueError("working centre distance must be greater than zero")
        # The base-circle distance is invariant: a_w*cos(alpha_wt) equals
        # a*cos(alpha_t).  Therefore the requested working distance gives
        # cos(alpha_wt) = a*cos(alpha_t)/a_w.
        cosine = (
            reference_centre_distance / working_distance
        ) * math.cos(reference_pressure_angle)
        if cosine <= 0.0 or cosine > 1.0 + 1e-12:
            raise ValueError("working centre distance gives a non-physical pressure angle")
        cosine = min(1.0, cosine)
        return math.acos(cosine), working_distance

    # External pairs use X = x1 + x2. Internal pairs use X = x2 - x1 because
    # the positive physical ring count is the negative signed ISO count. The
    # branches are explicit here so the internal sign is not a mechanical
    # mutation of the external equation.
    combination = p.profile_shift_combination
    if abs(combination) <= 1e-15:
        # Preserve the old zero-shift path exactly, including the reference
        # pressure angle and nominal centre-distance floating-point values.
        return reference_pressure_angle, reference_centre_distance

    q = p.z2 - p.z1 if p.internal else p.z1 + p.z2
    target = inv(reference_pressure_angle) + (
        2.0 * combination * math.tan(p.alpha_n) / q
    )
    working_pressure_angle = _inverse_involute(target)
    working_distance = (
        reference_centre_distance
        * math.cos(reference_pressure_angle)
        / math.cos(working_pressure_angle)
    )
    return working_pressure_angle, working_distance


def transverse_contact_ratio(
    pinion: SpurMemberGeometry,
    gear: SpurMemberGeometry,
    centre_distance: float,
    alpha_t: float,
    circular_pitch: float,
    *,
    base_pitch_angle: float | None = None,
) -> float:
    """How many tooth pairs are in contact on average, in the transverse plane.

    The standard length-of-action over base pitch. The length of action is the
    part of the line of action lying between the two tip circles:

        external   g = sqrt(ra1^2 - rb1^2) + sqrt(ra2^2 - rb2^2) - a sin(alpha_t)
        internal   g = sqrt(ra1^2 - rb1^2) - sqrt(ra2^2 - rb2^2) + a sin(alpha_t)

    and the base pitch it is divided by is `p_t * cos(alpha_t)`. For a shifted
    external pair, ``alpha_t`` is the working angle in the length-of-action
    term while ``base_pitch_angle`` remains the reference angle. Below 1.0 the
    pair loses contact between teeth and cannot transmit continuous motion,
    which is why the validator treats that as an error rather than a warning.

    **Two signs flip together for an internal pair, and they have to.** The ring
    curves the same way as the pinion rather than against it, so its tip circle
    cuts the line of action on the far side of the pitch point from where an
    external gear's would, and the centre-distance term is added rather than
    subtracted.

    Measured on the anchor ring pair, the four sign combinations give:

        both flipped (correct)                g =  11.431    ratio 1.936
        neither flipped                       g =   9.913    ratio 1.679
        only the branch flipped               g = -17.298    ratio 0
        only the centre distance flipped      g =  38.643    ratio 6.545

    Note which one is dangerous. Half-flipping is loud - a ratio of zero or of
    six is obviously not a gear pair. **Forgetting the flip entirely is quiet**:
    1.679 against a true 1.936 is an ordinary-looking number that passes every
    validator, and it is why the test for this solves the length of action
    longhand rather than re-running this function.

    The consequence is worth knowing: an internal pair has a **higher** contact
    ratio than an external pair of the same tooth counts and module. 1.9361 on
    the 18 x 60 anchor against 1.6572 external.

    The `max(0, ...)` guards a tip circle that has fallen inside its own base
    circle - possible with a badly undercut pinion - where the square root is of
    a negative number and the geometry is meaningless anyway.
    """
    def branch(m: SpurMemberGeometry) -> float:
        return math.sqrt(max(0.0, m.tip_r ** 2 - m.base_r ** 2))

    if gear.internal:
        length_of_action = (
            branch(pinion) - branch(gear) + centre_distance * math.sin(alpha_t)
        )
    else:
        length_of_action = (
            branch(pinion) + branch(gear) - centre_distance * math.sin(alpha_t)
        )
    if base_pitch_angle is None:
        base_pitch_angle = alpha_t
    base_pitch = circular_pitch * math.cos(base_pitch_angle)
    return max(0.0, length_of_action / base_pitch)


def min_internal_teeth(alpha_t: float, addendum_factor: float = ADDENDUM_FACTOR) -> float:
    """Fewest teeth a ring gear can have before its tip falls inside its base circle.

    A ring's tip radius is `m_t (z - 2 h_a) / 2` and its base radius is
    `m_t z cos(alpha_t) / 2`, so the tip clears the base circle only while

        z > 2 * h_a / (1 - cos(alpha_t))

    Below that the flank has no involute anywhere on the tooth and there is
    nothing honest to draw - unlike an external gear, which falls below its base
    circle at the *root* and gets a radial line there as a standard
    simplification. Here the whole flank is gone, not the bottom of it.

    The numbers are larger than people expect, and they run the wrong way:

        14.5 deg     62.8   ->  63 teeth
        20   deg     33.2   ->  34 teeth
        25   deg     21.3   ->  22 teeth

    A *lower* pressure angle needs *more* teeth, because it puts the base circle
    closer to the pitch circle where the tip has to fit between them. This is
    why the anchor ring has 60 teeth rather than something smaller and neater.
    """
    denominator = 1.0 - math.cos(alpha_t)
    if denominator <= 0.0:
        return math.inf
    return 2.0 * addendum_factor / denominator


def undercut_limit(
    alpha_t: float,
    beta: float,
    profile_shift: float = 0.0,
    addendum_factor: float = ADDENDUM_FACTOR,
    dedendum_factor: float | None = None,
    root_radius_factor: float = 0.0,
) -> float:
    """Fewest teeth before a rack-generated external flank is undercut.

    The no-undercut condition for a straight rack-generated external gear,
    written in this repository's normal-module/profile-shift convention, is

        z >= 2 * (h_fP* - x - rho_fP*(1 - sin(alpha_n))) /
            sin(alpha_t)^2.

    The default rack values make the bracket equal to one module, so the
    historical result ``2/sin(alpha_t)^2`` remains 17.1 teeth at 20 degrees.
    For nonzero helix angle this helper retains the existing conservative
    transverse warning; the analytical Clause 10 construction is enabled only
    for external straight gears.

    This remains a conservative design warning. ``root_geometry="legacy"``
    uses a radial below-base approximation, so it does not show the cutter-limited
    undercut; the opt-in external straight rack-envelope mode does show that
    generated root form. The limit itself is still reported rather than used to
    silently alter the selected tooth geometry.
    """
    if abs(beta) <= 1e-12 and dedendum_factor is not None:
        effective_depth = (
            dedendum_factor
            - profile_shift
            - root_radius_factor * (1.0 - math.sin(alpha_t))
        )
        return 2.0 * effective_depth / math.sin(alpha_t) ** 2
    return 2.0 * math.cos(beta) * (addendum_factor - profile_shift) / math.sin(
        alpha_t
    ) ** 2


# ---------------------------------------------------------------------------
# Tooth space profile, in the transverse plane
# ---------------------------------------------------------------------------


def end_overshoot(geo: SpurSetGeometry) -> float:
    """How far past each end face the cut profile is pushed, mm."""
    return max(END_OVERSHOOT_MIN_MM, END_OVERSHOOT_FRACTION * geo.params.face_width)


def section_count(
    geo: SpurSetGeometry,
    member: str,
    max_sagitta: float = MAX_SECTION_SAGITTA_MM,
) -> int:
    """How many sections the loft needs to follow the helix closely enough.

    Two is right for straight teeth and wrong for helical ones: the loft chords
    each profile point between consecutive sections, and over a twist of `delta`
    the chord falls `r * (1 - cos(delta / 2))` inside the true helicoid. Solving
    that for the tip radius - the worst case, being furthest out - and rounding
    up gives the count.

    A guide curve does not remove the need for this. It pins the one point it
    runs through and leaves the rest of the profile interpolated, which is
    exactly what the measurement recorded beside `MAX_SECTION_SAGITTA_MM` found.
    """
    m = geo.member(member)
    twist = abs(m.twist)
    if twist <= 0.0 or max_sagitta <= 0.0:
        return 2

    # Largest twist per interval that keeps the sagitta inside the tolerance.
    ratio = 1.0 - max_sagitta / m.tip_r
    if ratio <= -1.0:
        return 2
    step = 2.0 * math.acos(max(-1.0, min(1.0, ratio)))
    return max(2, math.ceil(twist / step) + 1)


def section_heights(
    geo: SpurSetGeometry,
    member: str,
    max_sagitta: float = MAX_SECTION_SAGITTA_MM,
) -> list[float]:
    """The axial positions of the loft sections, ends included.

    Evenly spaced from below the front face to above the back one, both pushed
    out by `end_overshoot` so the cut never finishes tangent to a real face.
    """
    overshoot = end_overshoot(geo)
    z_lo = -overshoot
    z_hi = geo.params.face_width + overshoot
    n = section_count(geo, member, max_sagitta)
    return [z_lo + (z_hi - z_lo) * i / (n - 1) for i in range(n)]


@dataclass(frozen=True)
class ToothSpaceSection:
    """One transverse section of a single tooth space.

    `segments` keeps the boundary broken into named pieces so the SOLIDWORKS
    layer can build splines and arcs separately rather than forcing one spline
    through sharp corners. `loop_2d` is the same boundary flattened into a
    closed polyline, which is what the preview and DXF export want.

    `z` is the axial position and `phase` the rotation the twist has accumulated
    by it - zero at z = 0 by definition, since that is the clocking reference.
    """

    member: str
    z: float
    phase: float
    r_root: float
    r_tip: float
    r_cap: float
    filleted: bool
    generated_root_r: float | None = None
    root_form_r: float | None = None
    start_of_involute_angle: float | None = None
    involute_roll_parameter: float | None = None
    undercut: bool | None = None
    segments: dict[str, list[Point2]] = field(default_factory=dict)
    loop_2d: list[Point2] = field(default_factory=list)

    @property
    def rack_generated(self) -> bool:
        """Whether this section contains the verified rack-root envelope."""
        return bool(self.segments.get("generated_root_pos"))

    def loop_3d(self) -> list[Point3]:
        return [to_axial_3d(x, y, self.phase, self.z) for x, y in self.loop_2d]

    def segments_3d(self) -> dict[str, list[Point3]]:
        return {
            name: [to_axial_3d(x, y, self.phase, self.z) for x, y in pts]
            for name, pts in self.segments.items()
        }


def to_axial_3d(x: float, y: float, phase: float, z: float) -> Point3:
    """Lift a transverse profile point to its place on the real gear.

    Rotate by the phase the twist has reached at this z, then set the height.
    For straight teeth the phase is zero and this is a plain lift.
    """
    c, s = math.cos(phase), math.sin(phase)
    return x * c - y * s, x * s + y * c, z


def phase_at(geo: SpurSetGeometry, member: str, z: float) -> float:
    """How far the section at height `z` has twisted, radians.

    Linear in z, which is exactly what a helix is. Measured from z = 0, so it is
    negative below the front face and beyond the total twist above the back one -
    both of which happen, because the cut profile is pushed past each end.
    """
    b = geo.params.face_width
    if b <= 0.0:
        return 0.0
    return geo.member(member).twist * z / b


def tooth_space_section(
    geo: SpurSetGeometry,
    member: str,
    z: float = 0.0,
    n_flank: int = FLANK_POINTS,
    split_cap: bool = False,
) -> ToothSpaceSection:
    """Build one transverse section of a tooth space, centred on angle 0 at z=0.

    The loop runs counter-clockwise: up the negative flank, out past the tip,
    across the cap, back down the positive flank, then round the root.

    Every section of a spur gear is the same shape - only its phase differs -
    because nothing about the profile depends on z. That is the whole difference
    from the bevel case, where each section is a different size and the radii
    have to be rebuilt for every one.
    """
    p = geo.params
    m = geo.member(member)

    if m.internal:
        # The clamp runs the other way for a ring gear. Its tooth is narrowest
        # at the tip - the innermost radius - and widens outward, so what the
        # top land constrains is a *minimum* tip radius rather than a maximum.
        # The cap then clears the blank inward, past the tip, toward the axis.
        r_tip = max(
            m.tip_r,
            min_internal_tip_radius(
                m.base_r, m.psi0, m.half_pitch, MIN_TOP_LAND_FACTOR * p.module
            ),
        )
        r_cap = max(0.05 * m.base_r, r_tip - CUT_OVERSHOOT_FACTOR * p.module)
    else:
        # Clamp the tip so the tooth never goes pointed. Rarely binds for a spur
        # gear at standard proportions - it takes a very small tooth count - but
        # it costs nothing and it is the same guard the bevel side relies on.
        r_tip = min(
            m.tip_r,
            max_tip_radius(m.base_r, m.psi0, MIN_TOP_LAND_FACTOR * p.module),
        )
        r_cap = r_tip + CUT_OVERSHOOT_FACTOR * p.module

    # ISO 53 defines the rack in its normal section.  The direct analytical
    # envelope currently verified here is therefore limited to an external
    # straight gear, where that section is also the transverse plane.  Internal
    # and helical members deliberately retain the existing path until their
    # cutter geometry has its own verified construction.
    rack_root = None
    generated_root_r = None
    root_form_r = None
    start_of_involute_angle = None
    involute_roll_parameter = None
    undercut = None
    if (
        p.root_geometry == "rack_generated"
        and not m.internal
        and abs(p.beta) <= 1e-12
    ):
        generated = rack_generated_root(
            reference_r=m.reference_r,
            r_base=m.base_r,
            r_root=m.root_r,
            psi0=m.psi0,
            half_pitch=m.half_pitch,
            cutter_tip_depth=m.dedendum,
            rack_root_radius=p.basic_rack_root_radius_factor * p.module,
        )
        if generated is not None:
            soi = solve_start_of_involute(generated)
            if soi is not None and r_tip > soi.start_of_involute_r + 1e-10:
                rack_root = (
                    generated.sample_to(
                        soi.trochoid_parameter, max(24, n_flank // 2)
                    ),
                    soi.start_of_involute_r,
                )
                generated_root_r = generated.generated_root_r
                root_form_r = soi.start_of_involute_r
                start_of_involute_angle = soi.start_of_involute_angle
                involute_roll_parameter = soi.involute_roll_parameter
                undercut = soi.undercut

    segments, loop, filleted = tooth_space_loop(
        m.base_r, m.root_r, r_tip, r_cap, m.psi0, m.half_pitch,
        p.fillet_factor * p.module, n_flank, split_cap=split_cap,
        internal=m.internal,
        rack_root=rack_root,
    )
    if not segments.get("generated_root_pos"):
        generated_root_r = None
        root_form_r = None
        start_of_involute_angle = None
        involute_roll_parameter = None
        undercut = None

    return ToothSpaceSection(
        member=member,
        z=z,
        phase=phase_at(geo, member, z),
        r_root=m.root_r,
        r_tip=r_tip,
        r_cap=r_cap,
        filleted=filleted,
        generated_root_r=generated_root_r,
        root_form_r=root_form_r,
        start_of_involute_angle=start_of_involute_angle,
        involute_roll_parameter=involute_roll_parameter,
        undercut=undercut,
        segments=segments,
        loop_2d=loop,
    )


def guide_helix(
    geo: SpurSetGeometry,
    member: str,
    z_lo: float,
    z_hi: float,
    points: int = 61,
) -> list[Point3]:
    """The loft guide curve: the helix traced by the cap's centreline vertex.

    A guided loft needs its guide to touch a point that *exists* in every
    profile. The cap of the tooth-space section is symmetric about the space
    centreline, so the point at (r_cap, angle 0) in section coordinates traces a
    pure helix of radius r_cap as the section twists - and `split_cap` puts a
    real vertex there rather than leaving it somewhere along a spline.

    Sampled rather than handed to `InsertHelix`, for the same reason the tooth
    sections are: the builder already knows how to write a 3D sketch through
    points, and a sampled curve needs no agreement with SOLIDWORKS about pitch,
    start angle or hand.
    """
    section = tooth_space_section(geo, member, z=0.0)
    r_cap = section.r_cap
    return [
        to_axial_3d(r_cap, 0.0, phase_at(geo, member, z), z)
        for z in (
            z_lo + (z_hi - z_lo) * i / (points - 1) for i in range(points)
        )
    ]


# ---------------------------------------------------------------------------
# Blank
# ---------------------------------------------------------------------------


def rim_radius(geo: SpurSetGeometry, member: str) -> float:
    """Outside radius of a ring gear's rim, mm. The blank's real outer size.

    The root circle is where the teeth stop and the rim begins, so the rim
    stands `rim_thickness` outside it. Meaningless for an external member,
    whose blank ends at the tip circle.
    """
    return geo.member(member).root_r + max(0.0, geo.params.rim_thickness)


def blank_outline(geo: SpurSetGeometry, member: str) -> list[Point2]:
    """Meridian half-section of the gear blank, as (R, z) with the front face at 0.

    Revolving this about the Z axis gives the un-toothed body.

    **External** - a plain stepped cylinder, and deliberately so. The profile
    runs: bore at the front face, out across the front face to the tip radius,
    back along the outside to the back face, in across the back face, then home
    along the bore, with an optional hub boss standing behind the back face. The
    bevel blank ends on the back cone and needs a root rim under the teeth at
    the heel; a spur blank ends on planes perpendicular to the axis, both faces
    carry the full tooth depth uniformly, and there is no wedge of vanishing
    material to protect.

    **Internal** - a plain annulus. It runs from the tip circle (the *inner*
    face, because the teeth point inward) out to the rim, across the back, and
    home. Four corners, every dimension still linear, so the SOLIDWORKS builder
    needs no new machinery - only different numbers and one different variable
    name.

    A ring gear has no bore and no hub, and that is not an omission. Its inner
    surface is the toothed one, so there is nothing for a bore to be; anything
    it is fastened by belongs on the rim, and a boss standing off the back face
    would be a second feature rather than part of the gear. `bore` and
    `hub_thickness` are simply not read for an internal member - the validator
    says so rather than letting them look effective.
    """
    p = geo.params
    m = geo.member(member)
    b = p.face_width

    if m.internal:
        r_outer = rim_radius(geo, member)
        return [
            (m.tip_r, 0.0),
            (r_outer, 0.0),
            (r_outer, b),
            (m.tip_r, b),
        ]

    r_bore = p.bore / 2.0
    r_tip = m.tip_r
    r_hub = max(r_bore, min(m.root_r, r_bore + 2.0 * p.module))

    outline: list[Point2] = [
        (r_bore, 0.0),
        (r_tip, 0.0),
        (r_tip, b),
    ]

    if p.hub_thickness > 0.0 and r_hub > r_bore + 1e-9:
        outline += [
            (r_hub, b),
            (r_hub, b + p.hub_thickness),
            (r_bore, b + p.hub_thickness),
        ]
    else:
        outline.append((r_bore, b))

    return outline
