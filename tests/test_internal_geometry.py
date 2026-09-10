"""Closed-form checks on internal ring gears. SOLIDWORKS is not involved.

The anchor internal pair is **m=2, 18 x 60**, which is also the planet-and-ring
mesh of the anchor planetary set - so the two sit beside each other and any
number checked here is a number the planetary train relies on.

The property this file exists for is `test_the_placed_pair_interlocks...`. It
samples the pinion's metal and asks whether any of it is also inside the ring's
metal, which is the whole meshing condition stated as something a computer can
check. It discriminates: clocking the ring wrong by half a pitch takes it from
zero overlapping samples to 2232.
"""

from __future__ import annotations

import math

import pytest

from gears.involute import (
    internal_space_width,
    internal_tooth_width,
    inv,
    min_internal_tip_radius,
    top_land,
)
from gears.placement import angular_velocity_ratio
from gears.spur import mesh
from gears.spur.geometry import (
    compute_set,
    blank_outline,
    min_internal_teeth,
    rim_radius,
    tooth_space_section,
)
from gears.spur.params import SpurSetParams
from gears.spur.validate import MIN_INTERNAL_TOOTH_DIFFERENCE, validate

ANCHOR_INTERNAL = SpurSetParams.with_defaults(2.0, 18, 60, internal=True)
ANCHOR_EXTERNAL = SpurSetParams.with_defaults(2.0, 18, 60)


@pytest.fixture
def internal():
    return compute_set(ANCHOR_INTERNAL)


@pytest.fixture
def external():
    return compute_set(ANCHOR_EXTERNAL)


def _independent_inv(angle: float) -> float:
    """Independent involute copy for the internal shifted fixtures."""
    return math.tan(angle) - angle


def _inverse_independent_inv(value: float) -> float:
    """Independent bounded inverse used only to form expected test values."""
    lo, hi = 0.0, math.nextafter(math.pi / 2.0, 0.0)
    for _ in range(100):
        mid = (lo + hi) / 2.0
        if _independent_inv(mid) < value:
            lo = mid
        else:
            hi = mid
    return (lo + hi) / 2.0


def _independent_internal_member(
    normal_module, teeth, shift, alpha_n, beta, internal, tip_alteration=0.0
):
    """Expected positive-radius internal geometry without production helpers."""
    transverse_module = normal_module / math.cos(beta)
    alpha_t = math.atan2(math.tan(alpha_n), math.cos(beta))
    reference_r = transverse_module * teeth / 2.0
    base_r = reference_r * math.cos(alpha_t)
    shift_sign = -1.0 if internal else 1.0
    normal_thickness = normal_module * (
        math.pi / 2.0 + shift_sign * 2.0 * shift * math.tan(alpha_n)
    )
    tooth_thickness = normal_thickness / math.cos(beta)
    if internal:
        addendum = normal_module * (1.0 - shift + tip_alteration)
        dedendum = normal_module * (1.25 + shift)
        tip_r = reference_r - addendum
        root_r = reference_r + dedendum
        space_width = math.pi * transverse_module - tooth_thickness
        psi0 = space_width / (2.0 * reference_r) + _independent_inv(alpha_t)
    else:
        addendum = normal_module * (1.0 + shift + tip_alteration)
        dedendum = normal_module * (1.25 - shift)
        tip_r = reference_r + addendum
        root_r = reference_r - dedendum
        space_width = None
        psi0 = tooth_thickness / (2.0 * reference_r) + _independent_inv(alpha_t)
    return {
        "reference_r": reference_r,
        "base_r": base_r,
        "tip_r": tip_r,
        "root_r": root_r,
        "tooth_thickness": tooth_thickness,
        "space_width": space_width,
        "psi0": psi0,
    }


INTERNAL_SHIFT_CASES = (
    ("zero", 0.0, 0.0),
    ("positive_pinion", 0.3, 0.0),
    ("balanced_negative_ring", 0.3, -0.3),
    ("positive_ring", 0.0, 0.3),
)


@pytest.mark.parametrize("beta_deg", [0.0, 15.0])
@pytest.mark.parametrize("case,shift_1,shift_2", INTERNAL_SHIFT_CASES)
def test_internal_profile_shift_geometry_matches_independent_equations(
    beta_deg, case, shift_1, shift_2
):
    """Check internal signs, dimensions, ring space, and operating contact."""
    normal_module = 2.0
    z1, z2 = 18, 60
    alpha_n = math.radians(20.0)
    beta = math.radians(beta_deg)
    p = SpurSetParams.with_defaults(
        normal_module,
        z1,
        z2,
        internal=True,
        helix_angle=beta_deg,
        profile_shift_1=shift_1,
        profile_shift_2=shift_2,
    )
    geo = compute_set(p)
    m_t = normal_module / math.cos(beta)
    alpha_t = math.atan2(math.tan(alpha_n), math.cos(beta))
    q = z2 - z1
    expected_reference_distance = m_t * q / 2.0
    expected_working_inv = _independent_inv(alpha_t) + (
        2.0 * (shift_2 - shift_1) * math.tan(alpha_n) / q
    )
    expected_working_angle = _inverse_independent_inv(expected_working_inv)
    expected_working_distance = (
        expected_reference_distance
        * math.cos(alpha_t)
        / math.cos(expected_working_angle)
    )
    tip_alteration = (
        (expected_reference_distance - expected_working_distance) / normal_module
        + (shift_2 - shift_1)
    )
    expected_members = (
        _independent_internal_member(
            normal_module, z1, shift_1, alpha_n, beta, internal=False,
            tip_alteration=tip_alteration,
        ),
        _independent_internal_member(
            normal_module, z2, shift_2, alpha_n, beta, internal=True,
            tip_alteration=tip_alteration,
        ),
    )

    assert geo.transverse_module == pytest.approx(m_t, abs=1e-12)
    assert geo.reference_pressure_angle == pytest.approx(alpha_t, abs=1e-12)
    assert geo.reference_centre_distance == pytest.approx(
        expected_reference_distance, abs=1e-12
    )
    assert geo.working_pressure_angle == pytest.approx(
        expected_working_angle, abs=1e-12
    )
    assert geo.working_centre_distance == pytest.approx(
        expected_working_distance, abs=1e-12
    )
    assert geo.working_centre_distance == pytest.approx(
        geo.gear.working_r - geo.pinion.working_r, abs=1e-12
    )

    expected_action = 0.0
    for member, expected in zip((geo.pinion, geo.gear), expected_members):
        assert member.reference_d == pytest.approx(2.0 * expected["reference_r"])
        assert member.base_d == pytest.approx(2.0 * expected["base_r"])
        assert member.tip_d == pytest.approx(2.0 * expected["tip_r"])
        assert member.root_d == pytest.approx(2.0 * expected["root_r"])
        assert member.reference_tooth_thickness == pytest.approx(
            expected["tooth_thickness"], abs=1e-12
        )
        assert member.working_r == pytest.approx(
            expected["base_r"] / math.cos(expected_working_angle), abs=1e-12
        )
        assert member.psi0 == pytest.approx(expected["psi0"], abs=1e-12)

        if member.internal:
            assert internal_space_width(
                member.reference_r, member.base_r, member.psi0
            ) == pytest.approx(expected["space_width"], abs=1e-12)
            assert internal_tooth_width(
                member.reference_r,
                member.base_r,
                member.psi0,
                member.half_pitch,
            ) == pytest.approx(expected["tooth_thickness"], abs=1e-12)
        else:
            assert top_land(member.reference_r, member.base_r, member.psi0) == pytest.approx(
                expected["tooth_thickness"], abs=1e-12
            )

        expected_action += math.sqrt(
            max(0.0, expected["tip_r"] ** 2 - expected["base_r"] ** 2)
        ) * (-1.0 if member.internal else 1.0)

    expected_action += expected_working_distance * math.sin(expected_working_angle)
    expected_contact = expected_action / (math.pi * m_t * math.cos(alpha_t))
    assert geo.transverse_contact_ratio == pytest.approx(expected_contact, abs=1e-12)
    expected_overlap = p.face_width * abs(math.sin(beta)) / (math.pi * normal_module)
    assert geo.axial_contact_ratio == pytest.approx(expected_overlap, abs=1e-12)


def test_internal_profile_shift_uses_ring_minus_pinion_not_external_sum():
    """The wrong but plausible external sign predicts the opposite distance."""
    p = SpurSetParams.with_defaults(
        2.0, 18, 60, internal=True, profile_shift_1=0.3, profile_shift_2=0.0
    )
    geo = compute_set(p)
    alpha_n = math.radians(20.0)
    alpha_t = math.atan2(math.tan(alpha_n), 1.0)
    q = 60 - 18
    wrong_inv = _independent_inv(alpha_t) + 2.0 * 0.3 * math.tan(alpha_n) / q
    wrong_angle = _inverse_independent_inv(wrong_inv)
    wrong_distance = 42.0 * math.cos(alpha_t) / math.cos(wrong_angle)

    assert geo.working_pressure_angle < geo.reference_pressure_angle
    assert geo.working_centre_distance < geo.reference_centre_distance
    assert wrong_distance > geo.reference_centre_distance
    assert geo.working_centre_distance != pytest.approx(wrong_distance, abs=1e-9)


def test_positive_ring_shift_reduces_inward_addendum_and_increases_outer_root():
    standard = compute_set(SpurSetParams.with_defaults(2.0, 18, 60, internal=True))
    shifted = compute_set(
        SpurSetParams.with_defaults(
            2.0, 18, 60, internal=True, profile_shift_2=0.3
        )
    )
    assert shifted.gear.tip_r > standard.gear.tip_r
    assert shifted.gear.root_r > standard.gear.root_r


# --- radii -----------------------------------------------------------------


def test_the_ring_puts_its_tip_inside_its_pitch_circle_and_its_root_outside(internal):
    """The whole of what "internal" means to the geometry, in two lines."""
    r = internal.gear
    assert r.internal
    assert r.pitch_r == pytest.approx(60.0)
    assert r.tip_r == pytest.approx(58.0)      # pitch - 1.00 * m_n
    assert r.root_r == pytest.approx(62.5)     # pitch + 1.25 * m_n
    assert r.tip_r < r.pitch_r < r.root_r


def test_the_pinion_of_an_internal_pair_is_an_ordinary_external_gear(internal, external):
    """Only the second member changes. A test because it would be easy not to."""
    for field in ("z", "pitch_r", "base_r", "tip_r", "root_r", "psi0", "half_pitch"):
        assert getattr(internal.pinion, field) == pytest.approx(
            getattr(external.pinion, field)
        )
    assert not internal.pinion.internal


def test_the_ring_shares_the_pinions_base_circle_formula(internal):
    """Same involute, same base radius rule - it is the same curve family."""
    r = internal.gear
    assert r.base_r == pytest.approx(
        r.pitch_r * math.cos(internal.transverse_pressure_angle)
    )
    # And the tip must clear it, or there is no involute flank to cut.
    assert r.tip_r > r.base_r


def test_the_working_centre_distance_is_the_difference_not_the_sum(internal, external):
    """At zero shift, a_w = m_t (z2 - z1) / 2 internally and is a sum externally."""
    assert internal.working_centre_distance == pytest.approx(42.0)
    assert external.working_centre_distance == pytest.approx(78.0)
    assert internal.working_centre_distance == pytest.approx(
        internal.gear.pitch_r - internal.pinion.pitch_r
    )


def test_the_reference_pitch_circles_are_internally_tangent(internal):
    """At zero shift, the reference radii and working distance agree."""
    assert (
        internal.pinion.reference_r + internal.working_centre_distance
        == pytest.approx(internal.gear.pitch_r)
    )


# --- the tooth form --------------------------------------------------------


def test_the_rings_space_at_its_pitch_circle_is_half_the_circular_pitch(internal):
    """No profile shift and no backlash, so tooth and space are equal there.

    This is the check that the space-width substitution in `compute_set` is the
    right one. Build psi0 from the tooth thickness instead - the external
    convention - and this comes out wrong immediately.
    """
    r = internal.gear
    width = internal_space_width(r.pitch_r, r.base_r, r.psi0)
    assert width == pytest.approx(math.pi * internal.transverse_module / 2.0)


@pytest.mark.parametrize("backlash", [0.0, 0.05, 0.2])
def test_the_internal_mesh_sees_the_backlash_once_and_not_twice(backlash):
    """The same pair property as the external case, read the other way round.

    The ring's `psi0` is a **space** half-width, so what `internal_space_width`
    returns off it is the space and the tooth is what is left of the pitch. The
    backlash therefore reaches the ring by *widening* its space rather than by
    thinning its tooth directly - the same half-millimetre either way, and the
    check that both descriptions agree is that the pair still loses exactly one
    backlash between them.
    """
    from gears.involute import top_land

    g = compute_set(SpurSetParams.with_defaults(2.0, 18, 60, internal=True,
                                                backlash=backlash))
    pinion = top_land(g.pinion.pitch_r, g.pinion.base_r, g.pinion.psi0)
    ring_space = internal_space_width(g.gear.pitch_r, g.gear.base_r, g.gear.psi0)
    ring_tooth = g.circular_pitch - ring_space

    assert ring_space == pytest.approx(
        g.circular_pitch / 2.0 + backlash / 2.0, rel=1e-12
    )
    assert pinion + ring_tooth == pytest.approx(g.circular_pitch - backlash, rel=1e-12)


@pytest.mark.parametrize("shift_1,shift_2", [(0.3, 0.0), (0.3, -0.3)])
def test_shifted_internal_mesh_applies_backlash_once_to_tooth_and_space(
    shift_1, shift_2
):
    p = SpurSetParams.with_defaults(
        2.0,
        18,
        60,
        internal=True,
        profile_shift_1=shift_1,
        profile_shift_2=shift_2,
        backlash=0.12,
    )
    g = compute_set(p)
    pinion, ring = g.pinion, g.gear
    ring_space = internal_space_width(ring.pitch_r, ring.base_r, ring.psi0)

    assert pinion.geometric_tooth_thickness - pinion.reference_tooth_thickness == pytest.approx(
        0.06, abs=1e-12
    )
    assert ring.geometric_tooth_thickness - ring.reference_tooth_thickness == pytest.approx(
        0.06, abs=1e-12
    )
    assert top_land(pinion.pitch_r, pinion.base_r, pinion.psi0) == pytest.approx(
        pinion.reference_tooth_thickness, abs=1e-12
    )
    assert ring_space == pytest.approx(
        g.circular_pitch - ring.reference_tooth_thickness, abs=1e-12
    )


def test_the_rings_flank_is_a_true_involute_of_its_base_circle(internal):
    """Sampled off the generated profile, checked against the closed form.

    The angular position of the space boundary at radius r must be exactly
    psi0 - inv(alpha_r). Nothing about the sampling or the fillet trimming is
    allowed to move it.
    """
    r = internal.gear
    section = tooth_space_section(internal, "gear")
    for x, y in section.segments["flank_pos"]:
        radius = math.hypot(x, y)
        alpha_r = math.acos(min(1.0, r.base_r / radius))
        assert math.atan2(y, x) == pytest.approx(r.psi0 - inv(alpha_r), abs=1e-5)


def test_a_ring_tooth_is_narrowest_at_its_tip(internal):
    """The reverse of an external tooth, because the whole tooth is inverted.

    Measured on the anchor ring: 1.8331 mm at the tip against 5.4308 at the
    root. It matters because it decides which end the pointed-tooth check has to
    be made at.
    """
    r = internal.gear
    at_tip = internal_tooth_width(r.tip_r, r.base_r, r.psi0, r.half_pitch)
    at_root = internal_tooth_width(r.root_r, r.base_r, r.psi0, r.half_pitch)
    assert at_tip == pytest.approx(1.8331, abs=1e-3)
    assert at_root == pytest.approx(5.4308, abs=1e-3)
    assert at_tip < at_root


def test_a_ring_space_is_narrowest_at_its_root(internal):
    """And the space is the other way about, for the same reason."""
    r = internal.gear
    assert internal_space_width(r.root_r, r.base_r, r.psi0) < internal_space_width(
        r.tip_r, r.base_r, r.psi0
    )


def test_the_generated_section_spans_tip_to_root_with_the_cap_innermost(internal):
    """A ring's cut clears the blank *inward*, toward the axis."""
    r = internal.gear
    section = tooth_space_section(internal, "gear")
    radii = [math.hypot(x, y) for x, y in section.loop_2d]
    assert min(radii) < r.tip_r          # the cap overshoots past the tip
    assert max(radii) == pytest.approx(r.root_r, abs=1e-6)
    assert section.filleted


def test_the_minimum_tip_radius_clamps_at_the_base_circle_when_it_does_not_bind(
    internal,
):
    """`min_internal_tip_radius` bisects the opposite way from `max_tip_radius`.

    On the anchor ring the constraint does not bind at all: the tooth is already
    1.2715 mm wide at the base circle, well over any sane top land, so the
    function returns the base radius rather than a smaller number that would
    name a radius where the involute does not exist.
    """
    r = internal.gear
    assert internal_tooth_width(
        r.base_r, r.base_r, r.psi0, r.half_pitch
    ) == pytest.approx(1.2715, abs=1e-3)
    assert min_internal_tip_radius(r.base_r, r.psi0, r.half_pitch, 0.1) == r.base_r


def test_the_minimum_tip_radius_solves_the_top_land_when_it_does_bind():
    """It binds on a ring with many teeth - past about 105 at 20 degrees.

    `half_pitch - psi0` is `pi / (2z) - inv(alpha_t)`, which goes negative once
    z passes `pi / (2 inv(alpha_t))` = 105.4 at 20 degrees. Past there the tooth
    is already pointed at the base circle and the tip has to stand outside it.
    """
    geo = compute_set(SpurSetParams.with_defaults(2.0, 18, 140, internal=True))
    r = geo.gear
    assert internal_tooth_width(r.base_r, r.base_r, r.psi0, r.half_pitch) <= 0.1

    limit = min_internal_tip_radius(r.base_r, r.psi0, r.half_pitch, 0.1)
    assert limit > r.base_r
    assert internal_tooth_width(limit, r.base_r, r.psi0, r.half_pitch) == pytest.approx(
        0.1, abs=1e-6
    )


# --- the contact ratio -----------------------------------------------------


def test_the_internal_contact_ratio_matches_the_length_of_action_longhand(internal):
    """Solved here rather than by re-running the function under test.

    Forgetting the sign flip is the quiet failure - 1.679 against a true 1.936
    is an ordinary-looking number - so this test refuses to share any arithmetic
    with the code it is checking.
    """
    p, r = internal.pinion, internal.gear
    alpha_t = internal.transverse_pressure_angle
    g = (
        math.sqrt(p.tip_r ** 2 - p.base_r ** 2)
        - math.sqrt(r.tip_r ** 2 - r.base_r ** 2)
        + internal.centre_distance * math.sin(alpha_t)
    )
    base_pitch = math.pi * internal.transverse_module * math.cos(alpha_t)
    assert internal.transverse_contact_ratio == pytest.approx(g / base_pitch)
    assert internal.transverse_contact_ratio == pytest.approx(1.9361, abs=1e-3)


def test_an_internal_pair_has_a_higher_contact_ratio_than_the_external_one(
    internal, external
):
    """Same teeth, same module - the ring's concave flank is what buys it."""
    assert external.transverse_contact_ratio == pytest.approx(1.6572, abs=1e-3)
    assert internal.transverse_contact_ratio > external.transverse_contact_ratio


def test_the_external_formula_is_still_used_for_an_external_pair(external):
    """The regression guard on the branch in `transverse_contact_ratio`."""
    p, g_ = external.pinion, external.gear
    alpha_t = external.transverse_pressure_angle
    g = (
        math.sqrt(p.tip_r ** 2 - p.base_r ** 2)
        + math.sqrt(g_.tip_r ** 2 - g_.base_r ** 2)
        - external.centre_distance * math.sin(alpha_t)
    )
    base_pitch = math.pi * external.transverse_module * math.cos(alpha_t)
    assert external.transverse_contact_ratio == pytest.approx(g / base_pitch)


# --- placement and meshing -------------------------------------------------


def test_the_ring_is_placed_on_the_negative_x_side(internal, external):
    """Internally tangent circles touch on the far side of the small one."""
    assert mesh.gear_translation(internal)[0] == pytest.approx(-42.0)
    assert mesh.gear_translation(external)[0] == pytest.approx(+78.0)


def test_a_shifted_internal_pair_is_placed_at_the_working_distance():
    geo = compute_set(
        SpurSetParams.with_defaults(
            2.0,
            18,
            60,
            internal=True,
            profile_shift_1=0.3,
            profile_shift_2=0.0,
        )
    )
    assert geo.working_centre_distance < geo.reference_centre_distance
    assert mesh.gear_translation(geo) == pytest.approx(
        (-geo.working_centre_distance, 0.0, 0.0)
    )
    assert geo.working_centre_distance == pytest.approx(
        geo.gear.working_r - geo.pinion.working_r
    )


def test_the_contact_point_lands_on_both_pitch_circles(internal):
    """The placement's own consistency check, and the reason for the -a_w sign.

    With the ring at -a_w the contact sits at (+r_p1, 0), which is angle 0 in the
    pinion's frame - where its tooth space already is. At +a_w it would land at
    (-r_p1, 0), and the pinion would need clocking too.
    """
    contact = (internal.pinion.pitch_r, 0.0)
    ring_centre = mesh.gear_translation(internal)[:2]
    assert math.dist(contact, (0.0, 0.0)) == pytest.approx(internal.pinion.pitch_r)
    assert math.dist(contact, ring_centre) == pytest.approx(internal.gear.pitch_r)
    assert math.atan2(contact[1], contact[0]) == pytest.approx(0.0)


def test_the_internal_clocking_is_exactly_half_an_angular_pitch(internal):
    """No parity case: the ring's tooth centres are half a pitch off its spaces."""
    assert mesh.clocking_for(internal) == pytest.approx(
        math.pi / internal.gear.z
    )
    assert mesh.clocking_for(internal) == pytest.approx(math.radians(3.0))


def test_the_external_clocking_is_left_alone(external):
    from gears.placement import gear_clocking

    assert mesh.clocking_for(external) == pytest.approx(gear_clocking(external.gear.z))


def _in_material(m, radius: float, theta: float, clock: float) -> bool:
    """Whether a point in a member's own frame is inside its metal.

    Written out longhand from the involute relation rather than by testing the
    generated polygon, so it does not inherit any error the profile builder has.
    """
    if m.internal:
        if radius >= m.root_r:
            return True
        if radius <= m.tip_r:
            return False
    else:
        if radius <= m.root_r:
            return True
        if radius >= m.tip_r:
            return False

    alpha_r = math.acos(min(1.0, m.base_r / max(radius, m.base_r)))
    half_space = (
        m.psi0 - inv(alpha_r) if m.internal
        else m.half_pitch - m.psi0 + inv(alpha_r)
    )
    tau = m.angular_pitch
    offset = (theta - clock) % tau
    return min(offset, tau - offset) > half_space


def _overlapping_samples(geo, clocking_error: float = 0.0, steps=360, radial=40) -> int:
    """How many samples of the pinion's metal are also inside the gear's metal.

    Zero is the meshing condition. Anything else is the two parts occupying the
    same space, which no amount of correct radii will fix.
    """
    clock = mesh.clocking_for(geo) + clocking_error
    tx = mesh.gear_translation(geo)
    pinion, gear = geo.pinion, geo.gear

    overlaps = 0
    for i in range(steps):
        theta = 2.0 * math.pi * i / steps
        for j in range(radial):
            radius = pinion.root_r + (pinion.tip_r - pinion.root_r) * j / (radial - 1)
            if not _in_material(pinion, radius, theta, 0.0):
                continue
            x = radius * math.cos(theta) - tx[0]
            y = radius * math.sin(theta) - tx[1]
            if _in_material(gear, math.hypot(x, y), math.atan2(y, x), clock):
                overlaps += 1
    return overlaps


@pytest.mark.parametrize("kind", ["internal", "external"])
def test_the_placed_pair_interlocks_without_the_two_bodies_overlapping(kind, request):
    """The meshing condition, checked as a solid-body question.

    Every other test here checks one number. This one places the whole pair the
    way the assembly builder does and asks whether any of the pinion's metal is
    inside the ring's - which is what "it meshes" actually means, and what no
    amount of individually-correct radii guarantees.
    """
    geo = request.getfixturevalue(kind)
    assert _overlapping_samples(geo) == 0


@pytest.mark.parametrize("kind", ["internal", "external"])
def test_the_interlock_check_would_catch_a_clocking_error(kind, request):
    """A test that cannot fail proves nothing, so make it fail on purpose.

    Half an angular pitch is the smallest error that matters and the one most
    likely to be made - it is exactly the difference between aiming a tooth
    centre and a tooth space at the contact.
    """
    geo = request.getfixturevalue(kind)
    wrong = _overlapping_samples(geo, clocking_error=geo.gear.angular_pitch / 2.0)
    assert wrong > 100


def test_an_internal_pair_turns_the_same_way_and_an_external_one_does_not():
    assert angular_velocity_ratio(18, 60, internal=True) == pytest.approx(60 / 18)
    assert angular_velocity_ratio(18, 60) == pytest.approx(-60 / 18)


# --- hands -----------------------------------------------------------------


def test_an_internal_helical_pair_is_cut_with_the_same_hand():
    """The reverse of the external rule, and forced by the same argument.

    There is no flip in the placement either way. An external gear faces its
    pinion, so its helix has to run the other way; a ring wraps around its
    pinion and faces the same way, so its helix has to match.
    """
    geo = compute_set(
        SpurSetParams.with_defaults(2.0, 18, 60, internal=True, helix_angle=15.0)
    )
    assert geo.pinion.hand == geo.gear.hand == "right"
    assert geo.pinion.beta == pytest.approx(geo.gear.beta)


def test_an_external_helical_pair_is_still_cut_with_opposite_hands():
    geo = compute_set(SpurSetParams.with_defaults(2.0, 18, 60, helix_angle=15.0))
    assert geo.pinion.hand == "right"
    assert geo.gear.hand == "left"
    assert geo.pinion.beta == pytest.approx(-geo.gear.beta)


def test_the_axial_pitch_still_matches_between_an_internal_pairs_members():
    """What has to agree between helical members is the axial pitch, not the twist.

    True of an internal pair as well, and worth its own check because the twists
    now have the *same* sign rather than opposite ones - which makes a bug that
    simply compares them look correct.

    Three lengths are easy to confuse here and only one of them is the axial
    pitch. The **lead** is the axial distance a tooth takes to go once round,
    `2*pi * face_width / twist`; the **axial pitch** is the axial distance
    between consecutive teeth, which is the lead divided by the tooth count. It
    is the second that has to match between the members - the leads differ by a
    factor of z2/z1, 436.97 mm against 1456.6 on this pair.
    """
    p = SpurSetParams.with_defaults(2.0, 18, 60, internal=True, helix_angle=15.0)
    geo = compute_set(p)
    expected = math.pi * p.module / abs(math.sin(p.beta))

    assert geo.axial_pitch == pytest.approx(expected)
    leads = []
    for member in (geo.pinion, geo.gear):
        lead = 2.0 * math.pi * p.face_width / abs(member.twist)
        leads.append(lead)
        assert lead / member.z == pytest.approx(expected)

    assert leads[0] == pytest.approx(436.97, abs=0.01)
    assert leads[1] == pytest.approx(1456.58, abs=0.01)

    # The twists themselves differ, because the radii do - and they now share a
    # sign, which is the internal pair's own rule.
    assert geo.pinion.twist != pytest.approx(geo.gear.twist)
    assert geo.pinion.twist * geo.gear.twist > 0.0


# --- the blank -------------------------------------------------------------


def test_a_ring_blank_is_an_annulus_from_the_tip_circle_out_to_the_rim(internal):
    outline = blank_outline(internal, "gear")
    assert len(outline) == 4
    radii = sorted({r for r, _ in outline})
    assert radii == pytest.approx([58.0, 67.5])       # tip, and root + rim
    heights = sorted({z for _, z in outline})
    assert heights == pytest.approx([0.0, internal.params.face_width])


def test_the_rim_stands_outside_the_root_circle(internal):
    assert rim_radius(internal, "gear") == pytest.approx(
        internal.gear.root_r + internal.params.rim_thickness
    )
    assert rim_radius(internal, "gear") > internal.gear.root_r


def test_the_pinion_blank_of_an_internal_pair_is_unchanged(internal, external):
    assert blank_outline(internal, "pinion") == pytest.approx(
        blank_outline(external, "pinion")
    )


def test_a_ring_reports_the_diameter_its_teeth_occupy_not_its_rim(internal):
    """`outside_dia` means "how far the teeth reach", which for a ring is its root."""
    assert internal.gear.outside_dia == pytest.approx(2.0 * internal.gear.root_r)
    assert internal.gear.outside_dia < 2.0 * rim_radius(internal, "gear")


# --- validation ------------------------------------------------------------


def test_the_anchor_internal_pair_is_buildable():
    assert validate(ANCHOR_INTERNAL).ok


def test_too_few_teeth_between_the_two_is_refused():
    """The pinion and the ring have to differ by enough to clear each other."""
    for z2 in range(19, 18 + MIN_INTERNAL_TOOTH_DIFFERENCE):
        result = validate(SpurSetParams.with_defaults(2.0, 18, z2, internal=True))
        assert not result.ok
        assert any(
            "more teeth than the pinion" in issue.message for issue in result.errors
        )


def test_a_ring_needs_a_minimum_tooth_count_of_its_own():
    """A separate limit from the difference, and a larger one than people expect.

    Below `min_internal_teeth` the ring's tip radius falls inside its own base
    circle and the flank has no involute anywhere - not just near the root, the
    way an external gear runs out of involute. 34 teeth at 20 degrees.

    This is why the anchor ring has 60 teeth. A 28-tooth ring clears the
    difference rule against an 18-tooth pinion and is still unbuildable.
    """
    alpha_t = compute_set(ANCHOR_INTERNAL).transverse_pressure_angle
    assert min_internal_teeth(alpha_t) == pytest.approx(33.16, abs=0.01)

    result = validate(SpurSetParams.with_defaults(2.0, 18, 28, internal=True))
    assert not result.ok
    assert any("no involute" in issue.message for issue in result.errors)

    assert validate(SpurSetParams.with_defaults(2.0, 18, 34, internal=True)).ok


def test_a_larger_pressure_angle_lets_a_ring_have_fewer_teeth():
    """The limit runs the opposite way from every other tooth-count rule.

    A lower pressure angle puts the base circle nearer the pitch circle, leaving
    the tip less room to fit between them - so 14.5 degrees needs 63 teeth where
    25 degrees needs 22. Worth a test because the instinct is backwards.
    """
    limits = [
        min_internal_teeth(math.radians(a)) for a in (14.5, 20.0, 25.0)
    ]
    assert [math.ceil(v) for v in limits] == [63, 34, 22]
    assert limits == sorted(limits, reverse=True)


def test_a_ring_is_not_warned_about_undercut():
    """Undercut is what a rack does to a convex flank; a ring's is concave."""
    result = validate(SpurSetParams.with_defaults(2.0, 18, 60, internal=True))
    assert not any(
        "undercut" in issue.message and "gear" in issue.message
        for issue in result.warnings
    )


def test_a_ring_with_no_rim_is_refused():
    p = SpurSetParams.with_defaults(2.0, 18, 60, internal=True, rim_thickness=0.0)
    result = validate(p)
    assert not result.ok
    assert any(issue.field == "rim_thickness" for issue in result.errors)


def test_the_bore_belongs_to_the_pinion_when_the_gear_is_a_ring():
    """A ring has no bore - its inner surface is the toothed one.

    So `bore` still means something in an internal pair, but it means only the
    pinion's, and a bore too wide for the pinion is still a fault. What must
    *not* happen is the ring being measured against it: its root circle is at
    62.5 mm, so a 200 mm bore would look like a comfortable fit.
    """
    p = SpurSetParams.with_defaults(2.0, 18, 60, internal=True, bore=200.0)
    errors = [issue for issue in validate(p).errors if issue.field == "bore"]
    assert len(errors) == 1
    assert "pinion" in errors[0].message
    assert "gear" not in errors[0].message


# --- the CLI and the report ------------------------------------------------


def test_the_internal_flag_reaches_the_geometry_from_the_command_line(capsys):
    from gears.__main__ import main

    assert main(
        ["--type", "spur", "--module", "2", "--z1", "18", "--z2", "60", "--internal"]
    ) == 0
    out = capsys.readouterr().out
    assert "arrangement" in out
    assert "internal" in out
    assert "RING" in out          # the members column is relabelled
    assert "rim radius" in out


def test_without_the_flag_the_report_still_says_external(capsys):
    from gears.__main__ import main

    main(["--type", "spur", "--module", "2", "--z1", "18", "--z2", "60"])
    out = capsys.readouterr().out
    assert "external" in out
    assert "RING" not in out
    assert "rim radius" not in out


def test_internal_is_refused_on_a_bevel_set(capsys):
    """`--internal` belongs to the spur type, so the shared guard has to catch it."""
    import pytest as _pytest
    from gears.__main__ import main

    with _pytest.raises(SystemExit) as exit_info:
        main(["--type", "bevel", "--module", "2", "--z1", "17", "--z2", "43",
              "--internal"])
    assert exit_info.value.code == 2
    assert "--internal" in capsys.readouterr().err


def test_the_ring_and_pinion_parts_get_different_filenames():
    """Otherwise a ring 18x60 overwrites an external 18x60 built the same day."""
    from gears.sw.assembly_common import part_filename

    internal = compute_set(ANCHOR_INTERNAL)
    external = compute_set(ANCHOR_EXTERNAL)
    assert part_filename(internal, "gear", prefix="internal_") != part_filename(
        external, "gear", prefix="spur_"
    )
