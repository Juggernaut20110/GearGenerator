"""Closed-form checks on the spur geometry engine. SOLIDWORKS is not involved."""

from __future__ import annotations

import json
import math

import pytest

from gears.involute import inv, top_land
from gears.spur.geometry import (
    ADDENDUM_FACTOR,
    DEDENDUM_FACTOR,
    blank_outline,
    compute_set,
    end_overshoot,
    guide_helix,
    MAX_SECTION_SAGITTA_MM,
    phase_at,
    section_count,
    section_heights,
    to_axial_3d,
    tooth_space_section,
    undercut_limit,
)
from gears.spur.params import SpurSetParams

# The anchor cases, chosen to sit beside the bevel one so the numbers compare:
# m=2, 17 x 43 teeth, 20 deg. Straight, and the same set given a 15 deg helix.
ANCHOR = SpurSetParams.with_defaults(2.0, 17, 43)
ANCHOR_HELICAL = SpurSetParams.with_defaults(2.0, 17, 43, helix_angle=15.0)

MEMBERS = ["pinion", "gear"]
BETAS = [0.0, 10.0, 15.0, 30.0]
TOOTH_COUNTS = [(17, 43), (20, 20), (12, 60)]


@pytest.fixture
def geo():
    return compute_set(ANCHOR)


@pytest.fixture
def helical():
    return compute_set(ANCHOR_HELICAL)


def sets(beta=0.0, z1=17, z2=43, **kw):
    return compute_set(SpurSetParams.with_defaults(2.0, z1, z2, helix_angle=beta, **kw))


# --- the transverse plane --------------------------------------------------


def test_straight_teeth_leave_the_normal_values_alone(geo):
    """At beta = 0 the transverse and normal quantities must coincide exactly."""
    assert geo.transverse_module == pytest.approx(2.0, rel=1e-15)
    assert geo.transverse_pressure_angle == pytest.approx(math.radians(20.0), rel=1e-15)
    assert geo.axial_contact_ratio == 0.0
    assert geo.axial_pitch == math.inf


def test_anchor_transverse_conversions(helical):
    assert helical.transverse_module == pytest.approx(2.0 / math.cos(math.radians(15)))
    assert helical.transverse_module == pytest.approx(2.07055, abs=1e-5)
    assert math.degrees(helical.transverse_pressure_angle) == pytest.approx(
        20.6469, abs=1e-4
    )


def test_straight_gear_keeps_normal_and_transverse_module_equal(geo):
    assert geo.params.module == geo.transverse_module
    assert geo.params.module == geo.params.transverse_module


def test_reference_and_working_quantities_are_distinctly_named_in_the_default_case(
    geo,
):
    assert geo.reference_centre_distance == geo.working_centre_distance
    assert geo.centre_distance == geo.working_centre_distance
    assert geo.reference_pressure_angle == geo.transverse_pressure_angle
    assert geo.working_pressure_angle == geo.reference_pressure_angle

    for member in (geo.pinion, geo.gear):
        assert member.reference_r == member.working_r
        assert member.reference_d == pytest.approx(2.0 * member.reference_r)
        assert member.working_d == pytest.approx(2.0 * member.working_r)
        assert member.base_d == pytest.approx(2.0 * member.base_r)
        assert member.tip_d == pytest.approx(2.0 * member.tip_r)
        assert member.root_d == pytest.approx(2.0 * member.root_r)
        assert member.pitch_r == member.reference_r


def test_zero_profile_shift_is_the_exact_legacy_geometry_case():
    legacy = compute_set(SpurSetParams.with_defaults(2.0, 17, 43))
    explicit = compute_set(
        SpurSetParams.with_defaults(
            2.0,
            17,
            43,
            profile_shift_1=0.0,
            profile_shift_2=0.0,
            basic_rack_addendum_factor=1.0,
            basic_rack_clearance_factor=0.25,
            basic_rack_root_radius_factor=0.38,
            working_centre_distance=None,
            root_geometry="legacy",
        )
    )

    assert explicit == legacy
    for member in ("pinion", "gear"):
        assert tooth_space_section(explicit, member).loop_2d == tooth_space_section(
            legacy, member
        ).loop_2d


def test_profile_shift_and_basic_rack_coefficients_are_represented():
    p = SpurSetParams.with_defaults(
        2.0,
        17,
        43,
        profile_shift_1=0.2,
        profile_shift_2=-0.1,
        basic_rack_addendum_factor=1.05,
        basic_rack_clearance_factor=0.3,
        basic_rack_root_radius_factor=0.4,
        root_geometry="rack_generated",
    )
    assert p.profile_shift_1 == 0.2
    assert p.profile_shift_2 == -0.1
    assert p.profile_shift_combination == pytest.approx(0.1)
    assert p.basic_rack_dedendum_factor == pytest.approx(1.35)
    assert p.h_aP_star == p.basic_rack_addendum_factor
    assert p.h_fP_star == p.basic_rack_dedendum_factor
    assert p.c_P_star == p.basic_rack_clearance_factor
    assert p.rho_fP_star == p.basic_rack_root_radius_factor

    geo = compute_set(p)
    expected_inv = inv(p.alpha_t) + (
        2.0 * p.profile_shift_combination * math.tan(p.alpha_n) / (p.z1 + p.z2)
    )
    assert inv(geo.working_pressure_angle) == pytest.approx(expected_inv, abs=1e-12)
    assert geo.pinion.profile_shift == p.profile_shift_1
    assert geo.gear.profile_shift == p.profile_shift_2
    # The selected root mode is still represented by the legacy circular root
    # approximation; the external straight involute itself is shifted.
    assert tooth_space_section(geo, "pinion").filleted


def _independent_rack_root_point(
    reference_r, alpha, depth, rho, psi0, half_pitch, phi
):
    """One direct rack-envelope evaluation for the generated-root tests.

    This intentionally duplicates the rolling/envelope equation in the test
    rather than calling the production helper.  ``u`` is tangent to the rack,
    ``v`` points away from the gear, and the inverted rack tooth has its tip at
    ``v = -depth``.
    """
    tangent = math.tan(alpha)
    centre_v = -depth + rho
    centre_u = tangent * centre_v - rho / math.cos(alpha)
    a = reference_r + centre_v
    b = centre_u - reference_r * phi
    c, s = math.cos(phi), math.sin(phi)
    cx = c * a - s * b
    cy = s * a + c * b
    dcx = -s * a - c * b + s * reference_r
    dcy = c * a - s * b - c * reference_r
    speed = math.hypot(dcx, dcy)
    raw = (cx + rho * dcy / speed, cy - rho * dcx / speed)
    pitch_space_angle = half_pitch - psi0 + math.tan(alpha) - alpha
    cp, sp = math.cos(pitch_space_angle), math.sin(pitch_space_angle)
    return raw[0] * cp - raw[1] * sp, raw[0] * sp + raw[1] * cp


def test_rack_generated_root_is_the_rolling_envelope_not_a_radial_flank():
    p = SpurSetParams.with_defaults(
        2.0, 12, 43, root_geometry="rack_generated"
    )
    geo = compute_set(p)
    member = geo.pinion
    section = tooth_space_section(geo, "pinion")
    generated = section.segments["generated_root_pos"]

    assert section.rack_generated
    assert member.generated_root_r == pytest.approx(member.root_r, abs=1e-12)
    assert section.generated_root_r == pytest.approx(member.root_r, abs=1e-12)

    # The midpoint is independently evaluated from the transformed cutter
    # corner.  It is not an interpolation through measured gear points.
    alpha = math.acos(member.base_r / member.reference_r)
    depth = member.dedendum
    rho = p.basic_rack_root_radius_factor * p.module
    centre_v = -depth + rho
    centre_u = math.tan(alpha) * centre_v - rho / math.cos(alpha)
    phi_root = centre_u / member.reference_r
    tangent_v = centre_v - rho * math.sin(alpha)
    phi_transition = tangent_v * (1.0 + math.tan(alpha) ** 2) / (
        math.tan(alpha) * member.reference_r
    )
    i = len(generated) // 2
    # The positive side is traversed from the tip back to the root, so its
    # generated-root segment is transition-to-root.
    phi = phi_transition + (phi_root - phi_transition) * i / (
        len(generated) - 1
    )
    assert generated[i] == pytest.approx(
        _independent_rack_root_point(
            member.reference_r,
            alpha,
            depth,
            rho,
            member.psi0,
            member.half_pitch,
            phi,
        ),
        abs=1e-11,
    )

    # The old below-base construction held the flank at the base-circle ray;
    # the generated root leaves that ray immediately and therefore detects the
    # undercut/root form instead of hiding it behind a radial segment.
    old_radial = (
        member.root_r * math.cos(member.half_pitch - member.psi0 + inv(alpha)),
        member.root_r * math.sin(member.half_pitch - member.psi0 + inv(alpha)),
    )
    assert math.dist(generated[0], old_radial) > 1e-3


@pytest.mark.parametrize("shift", [-0.5, 0.0, 0.5])
def test_rack_root_depth_and_angular_width_follow_profile_shift(shift):
    p = SpurSetParams.with_defaults(
        2.0, 12, 43, profile_shift_1=shift, root_geometry="rack_generated"
    )
    geo = compute_set(p)
    section = tooth_space_section(geo, "pinion")
    root = section.segments["generated_root_pos"][-1]
    assert math.hypot(*root) == pytest.approx(
        2.0 * (12 / 2.0 - (1.25 - shift)), abs=1e-12
    )

    # Positive shift moves the cutter away from the root: the generated curve
    # is shallower and the space is narrower. Negative shift does the reverse.
    angle = math.atan2(root[1], root[0])
    zero_geo = compute_set(
        SpurSetParams.with_defaults(
            2.0, 12, 43, root_geometry="rack_generated"
        )
    )
    zero_root = tooth_space_section(zero_geo, "pinion").segments[
        "generated_root_pos"
    ][-1]
    zero_angle = math.atan2(zero_root[1], zero_root[0])
    if shift < 0.0:
        assert angle > zero_angle
    elif shift > 0.0:
        assert angle < zero_angle


def test_rack_root_is_continuous_at_the_theoretical_undercut_boundary():
    # Independent sharp-rack limit for a straight 20 degree gear:
    # z_min = 2 / sin(alpha)^2.
    limit = 2.0 / math.sin(math.radians(20.0)) ** 2
    assert 17.0 < limit < 18.0

    below = tooth_space_section(
        compute_set(
            SpurSetParams.with_defaults(2.0, 17, 43, root_geometry="rack_generated")
        ),
        "pinion",
    )
    above = tooth_space_section(
        compute_set(
            SpurSetParams.with_defaults(2.0, 18, 43, root_geometry="rack_generated")
        ),
        "pinion",
    )
    for section in (below, above):
        root = section.segments["generated_root_pos"]
        flank = section.segments["flank_pos"]
        assert root[0] == pytest.approx(flank[-1], abs=1e-12)
        assert math.dist(root[0], flank[-1]) < 1e-12
        assert len(root) >= 9
    assert below.segments["generated_root_pos"] != above.segments["generated_root_pos"]


def test_rack_generated_mode_does_not_touch_internal_or_unverified_helical_roots():
    internal = tooth_space_section(
        compute_set(
            SpurSetParams.with_defaults(
                2.0, 18, 60, internal=True, root_geometry="rack_generated"
            )
        ),
        "gear",
    )
    helical = tooth_space_section(
        compute_set(
            SpurSetParams.with_defaults(
                2.0, 12, 43, helix_angle=15.0, root_geometry="rack_generated"
            )
        ),
        "pinion",
    )
    assert not internal.rack_generated
    assert not helical.rack_generated
    assert internal.generated_root_r is None
    assert helical.generated_root_r is None


def _proper_segment_cross(a, b, c, d):
    def side(p, q, r):
        return (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (
            r[0] - p[0]
        )

    ab_c = side(a, b, c)
    ab_d = side(a, b, d)
    cd_a = side(c, d, a)
    cd_b = side(c, d, b)
    return ab_c * ab_d < -1e-10 and cd_a * cd_b < -1e-10


@pytest.mark.parametrize("z", [8, 12, 17, 18, 30])
def test_rack_generated_section_is_a_cad_safe_simple_loop(z):
    geo = compute_set(
        SpurSetParams.with_defaults(2.0, z, 43, root_geometry="rack_generated")
    )
    section = tooth_space_section(geo, "pinion")
    loop = section.loop_2d

    assert section.rack_generated
    assert loop[0] != loop[-1]
    area = 0.5 * sum(
        loop[i][0] * loop[(i + 1) % len(loop)][1]
        - loop[(i + 1) % len(loop)][0] * loop[i][1]
        for i in range(len(loop))
    )
    assert area > 0.0
    assert all(math.dist(a, b) > 1e-12 for a, b in zip(loop, loop[1:]))

    n = len(loop)
    for i in range(n):
        for j in range(i + 1, n):
            if j in (i - 1, i + 1) or (i == 0 and j == n - 1):
                continue
            assert not _proper_segment_cross(
                loop[i], loop[(i + 1) % n], loop[j], loop[(j + 1) % n]
            )

    # Existing preview, DXF, and SOLIDWORKS consumers use these names.  The
    # old names remain exact aliases while the new names identify the source.
    assert section.segments["fillet_neg"] == section.segments[
        "generated_root_neg"
    ]
    assert section.segments["fillet_pos"] == section.segments[
        "generated_root_pos"
    ]


def _reference_inv(angle: float) -> float:
    """Independent copy of inv(alpha) for the shifted-pair fixtures."""
    return math.tan(angle) - angle


def _inverse_reference_inv(value: float) -> float:
    """Independent bounded inverse used only to form expected test values."""
    lo, hi = 0.0, math.nextafter(math.pi / 2.0, 0.0)
    for _ in range(100):
        mid = (lo + hi) / 2.0
        if _reference_inv(mid) < value:
            lo = mid
        else:
            hi = mid
    return (lo + hi) / 2.0


def _independent_shifted_values(module, teeth, shift, alpha, backlash):
    """Return expected straight external values without production helpers."""
    reference_r = module * teeth / 2.0
    base_r = reference_r * math.cos(alpha)
    geometric_thickness = module * (
        math.pi / 2.0 + 2.0 * shift * math.tan(alpha)
    )
    reference_thickness = geometric_thickness - backlash / 2.0
    tip_r = reference_r + module * (1.0 + shift)
    root_r = reference_r - module * (1.25 - shift)
    psi0 = reference_thickness / (2.0 * reference_r) + _reference_inv(alpha)
    return {
        "reference_r": reference_r,
        "base_r": base_r,
        "tip_r": tip_r,
        "root_r": root_r,
        "geometric_thickness": geometric_thickness,
        "reference_thickness": reference_thickness,
        "normal_geometric_thickness": geometric_thickness,
        "normal_thickness": reference_thickness,
        "psi0": psi0,
    }


def _independent_helical_shifted_values(
    normal_module, teeth, shift, alpha_n, beta, backlash=0.0
):
    """Return ISO helical values using only normal/transverse equations.

    This deliberately does not call ``SpurSetParams`` conversion properties or
    any geometry helper.  Profile shift x is normal, while the involute and
    reference tooth thickness used by the transverse section are transverse.
    """
    transverse_module = normal_module / math.cos(beta)
    alpha_t = math.atan2(math.tan(alpha_n), math.cos(beta))
    reference_r = transverse_module * teeth / 2.0
    base_r = reference_r * math.cos(alpha_t)
    normal_geometric_thickness = normal_module * (
        math.pi / 2.0 + 2.0 * shift * math.tan(alpha_n)
    )
    geometric_thickness = normal_geometric_thickness / math.cos(beta)
    reference_thickness = geometric_thickness - backlash / 2.0
    normal_thickness = reference_thickness * math.cos(beta)
    tip_r = reference_r + normal_module * (1.0 + shift)
    root_r = reference_r - normal_module * (1.25 - shift)
    psi0 = reference_thickness / (2.0 * reference_r) + _reference_inv(alpha_t)
    return {
        "reference_r": reference_r,
        "base_r": base_r,
        "tip_r": tip_r,
        "root_r": root_r,
        "geometric_thickness": geometric_thickness,
        "reference_thickness": reference_thickness,
        "normal_geometric_thickness": normal_geometric_thickness,
        "normal_thickness": normal_thickness,
        "psi0": psi0,
    }


SHIFT_CASES = (
    ("zero", 0.0, 0.0),
    ("positive_pinion", 0.2, 0.0),
    ("negative_pinion", -0.2, 0.0),
    ("balanced_individual", 0.5, -0.5),
    ("positive_total", 0.2, 0.3),
)


@pytest.mark.parametrize("case,shift_1,shift_2", SHIFT_CASES)
def test_external_straight_profile_shift_geometry_against_independent_equations(
    case, shift_1, shift_2
):
    """The complete A-E fixture set is written out independently here."""
    module = 2.0
    z1, z2 = 17, 43
    alpha = math.radians(20.0)
    backlash = 0.0
    p = SpurSetParams.with_defaults(
        module,
        z1,
        z2,
        profile_shift_1=shift_1,
        profile_shift_2=shift_2,
        backlash=backlash,
    )
    geo = compute_set(p)
    q = z1 + z2
    total_shift = shift_1 + shift_2
    expected_working_inv = _reference_inv(alpha) + (
        2.0 * total_shift * math.tan(alpha) / q
    )
    expected_working_angle = _inverse_reference_inv(expected_working_inv)
    expected_reference_distance = module * q / 2.0
    expected_working_distance = (
        expected_reference_distance
        * math.cos(alpha)
        / math.cos(expected_working_angle)
    )
    expected_contact_denominator = math.pi * module * math.cos(alpha)

    assert geo.reference_centre_distance == pytest.approx(
        expected_reference_distance, abs=1e-12
    )
    assert geo.working_centre_distance == pytest.approx(
        expected_working_distance, abs=1e-12
    )
    assert geo.centre_distance == pytest.approx(expected_working_distance, abs=1e-12)
    assert geo.working_pressure_angle == pytest.approx(
        expected_working_angle, abs=1e-12
    )
    assert geo.centre_distance_modification == pytest.approx(
        (expected_working_distance - expected_reference_distance) / module,
        abs=1e-12,
    )

    expected_members = (
        _independent_shifted_values(module, z1, shift_1, alpha, backlash),
        _independent_shifted_values(module, z2, shift_2, alpha, backlash),
    )
    expected_action = 0.0
    for member, expected in zip((geo.pinion, geo.gear), expected_members):
        assert member.reference_d == pytest.approx(2.0 * expected["reference_r"])
        assert member.base_d == pytest.approx(2.0 * expected["base_r"])
        assert member.tip_d == pytest.approx(2.0 * expected["tip_r"])
        assert member.root_d == pytest.approx(2.0 * expected["root_r"])
        assert member.geometric_tooth_thickness == pytest.approx(
            expected["geometric_thickness"], abs=1e-12
        )
        assert member.reference_tooth_thickness == pytest.approx(
            expected["reference_thickness"], abs=1e-12
        )
        assert member.normal_geometric_tooth_thickness == pytest.approx(
            expected["normal_geometric_thickness"], abs=1e-12
        )
        assert member.normal_tooth_thickness == pytest.approx(
            expected["normal_thickness"], abs=1e-12
        )
        assert member.psi0 == pytest.approx(expected["psi0"], abs=1e-12)
        assert member.working_r == pytest.approx(
            expected["base_r"] / math.cos(expected_working_angle), abs=1e-12
        )
        expected_action += math.sqrt(
            max(0.0, expected["tip_r"] ** 2 - expected["base_r"] ** 2)
        )

    expected_action -= expected_working_distance * math.sin(expected_working_angle)
    expected_contact = expected_action / expected_contact_denominator
    assert geo.transverse_contact_ratio == pytest.approx(expected_contact, abs=1e-12)


HELICAL_SHIFT_CASES = (
    ("zero", 0.0, 0.0),
    ("balanced_individual", 0.3, -0.3),
    ("positive_total", 0.4, 0.2),
)


@pytest.mark.parametrize("beta_deg", [0.0, 15.0, 30.0])
@pytest.mark.parametrize("case,shift_1,shift_2", HELICAL_SHIFT_CASES)
def test_external_helical_profile_shift_geometry_against_independent_equations(
    beta_deg, case, shift_1, shift_2
):
    """Independent ISO normal/transverse equations cover the full pair view."""
    normal_module = 2.0
    z1, z2 = 17, 43
    alpha_n = math.radians(20.0)
    beta = math.radians(beta_deg)
    p = SpurSetParams.with_defaults(
        normal_module,
        z1,
        z2,
        helix_angle=beta_deg,
        profile_shift_1=shift_1,
        profile_shift_2=shift_2,
    )
    geo = compute_set(p)

    # These are written directly from the normal/transverse definitions.
    expected_m_t = normal_module / math.cos(beta)
    expected_alpha_t = math.atan2(math.tan(alpha_n), math.cos(beta))
    expected_reference_distance = expected_m_t * (z1 + z2) / 2.0
    expected_working_inv = _reference_inv(expected_alpha_t) + (
        2.0 * (shift_1 + shift_2) * math.tan(alpha_n) / (z1 + z2)
    )
    expected_working_angle = _inverse_reference_inv(expected_working_inv)
    expected_working_distance = (
        expected_reference_distance
        * math.cos(expected_alpha_t)
        / math.cos(expected_working_angle)
    )

    assert geo.transverse_module == pytest.approx(expected_m_t, abs=1e-12)
    assert geo.reference_pressure_angle == pytest.approx(expected_alpha_t, abs=1e-12)
    assert geo.reference_centre_distance == pytest.approx(
        expected_reference_distance, abs=1e-12
    )
    assert geo.working_pressure_angle == pytest.approx(
        expected_working_angle, abs=1e-12
    )
    assert geo.working_centre_distance == pytest.approx(
        expected_working_distance, abs=1e-12
    )

    expected_members = (
        _independent_helical_shifted_values(
            normal_module, z1, shift_1, alpha_n, beta
        ),
        _independent_helical_shifted_values(
            normal_module, z2, shift_2, alpha_n, beta
        ),
    )
    expected_action = 0.0
    for member, expected in zip((geo.pinion, geo.gear), expected_members):
        assert member.reference_d == pytest.approx(2.0 * expected["reference_r"])
        assert member.base_d == pytest.approx(2.0 * expected["base_r"])
        assert member.tip_d == pytest.approx(2.0 * expected["tip_r"])
        assert member.root_d == pytest.approx(2.0 * expected["root_r"])
        assert member.geometric_tooth_thickness == pytest.approx(
            expected["geometric_thickness"], abs=1e-12
        )
        assert member.reference_tooth_thickness == pytest.approx(
            expected["reference_thickness"], abs=1e-12
        )
        assert top_land(member.pitch_r, member.base_r, member.psi0) == pytest.approx(
            expected["reference_thickness"], abs=1e-12
        )
        assert member.normal_geometric_tooth_thickness == pytest.approx(
            expected["normal_geometric_thickness"], abs=1e-12
        )
        assert member.normal_tooth_thickness == pytest.approx(
            expected["normal_thickness"], abs=1e-12
        )
        assert member.psi0 == pytest.approx(expected["psi0"], abs=1e-12)
        assert member.working_r == pytest.approx(
            expected["base_r"] / math.cos(expected_working_angle), abs=1e-12
        )
        expected_action += math.sqrt(
            max(0.0, expected["tip_r"] ** 2 - expected["base_r"] ** 2)
        )

    expected_action -= expected_working_distance * math.sin(expected_working_angle)
    expected_contact = expected_action / (
        math.pi * expected_m_t * math.cos(expected_alpha_t)
    )
    expected_overlap = p.face_width * abs(math.sin(beta)) / (math.pi * normal_module)
    assert geo.transverse_contact_ratio == pytest.approx(expected_contact, abs=1e-12)
    assert geo.axial_contact_ratio == pytest.approx(expected_overlap, abs=1e-12)
    assert geo.total_contact_ratio == pytest.approx(
        expected_contact + expected_overlap, abs=1e-12
    )


@pytest.mark.parametrize("beta_deg", [15.0, 30.0])
def test_helical_profile_shift_uses_normal_module_and_pressure_angle(beta_deg):
    """A transverse-module/angle substitution would produce a different tooth."""
    normal_module = 2.0
    shift = 0.25
    alpha_n = math.radians(20.0)
    beta = math.radians(beta_deg)
    geo = compute_set(
        SpurSetParams.with_defaults(
            normal_module,
            17,
            43,
            helix_angle=beta_deg,
            profile_shift_1=shift,
        )
    )
    expected_normal = normal_module * (
        math.pi / 2.0 + 2.0 * shift * math.tan(alpha_n)
    )
    expected_transverse = expected_normal / math.cos(beta)
    assert geo.pinion.normal_geometric_tooth_thickness == pytest.approx(
        expected_normal, abs=1e-12
    )
    assert geo.pinion.geometric_tooth_thickness == pytest.approx(
        expected_transverse, abs=1e-12
    )
    assert geo.pinion.geometric_tooth_thickness != pytest.approx(
        (normal_module / math.cos(beta))
        * (math.pi / 2.0 + 2.0 * shift * math.tan(geo.params.alpha_t)),
        abs=1e-8,
    )


def test_helical_profile_shift_converges_to_straight_profile_shift():
    straight = compute_set(
        SpurSetParams.with_defaults(
            2.0, 17, 43, profile_shift_1=0.4, profile_shift_2=0.2
        )
    )
    nearly_straight = compute_set(
        SpurSetParams.with_defaults(
            2.0,
            17,
            43,
            helix_angle=math.degrees(1e-7),
            face_width=20.0,
            profile_shift_1=0.4,
            profile_shift_2=0.2,
        )
    )

    assert nearly_straight.transverse_module == pytest.approx(
        straight.transverse_module, abs=1e-13
    )
    assert nearly_straight.reference_pressure_angle == pytest.approx(
        straight.reference_pressure_angle, abs=1e-13
    )
    assert nearly_straight.working_pressure_angle == pytest.approx(
        straight.working_pressure_angle, abs=1e-13
    )
    assert nearly_straight.working_centre_distance == pytest.approx(
        straight.working_centre_distance, abs=1e-11
    )
    assert nearly_straight.transverse_contact_ratio == pytest.approx(
        straight.transverse_contact_ratio, abs=1e-11
    )
    for straight_member, helical_member in zip(
        (straight.pinion, straight.gear), (nearly_straight.pinion, nearly_straight.gear)
    ):
        for field in (
            "reference_r",
            "base_r",
            "tip_r",
            "root_r",
            "geometric_tooth_thickness",
            "reference_tooth_thickness",
            "psi0",
        ):
            assert getattr(helical_member, field) == pytest.approx(
                getattr(straight_member, field), abs=1e-11
            )
    assert nearly_straight.axial_contact_ratio < 1e-6


def test_profile_shift_changes_actual_involute_space_orientation():
    standard = compute_set(SpurSetParams.with_defaults(2.0, 17, 43))
    shifted = compute_set(
        SpurSetParams.with_defaults(2.0, 17, 43, profile_shift_1=0.2)
    )
    standard_section = tooth_space_section(standard, "pinion")
    shifted_section = tooth_space_section(shifted, "pinion")

    assert shifted.pinion.psi0 != pytest.approx(standard.pinion.psi0)
    assert shifted_section.loop_2d != standard_section.loop_2d
    # At the base circle the external tooth half-angle is psi0.  The space
    # flank therefore starts at half_pitch - psi0, proving the changed psi0
    # reaches the involute placement rather than stopping at the dimensions.
    expected_space_angle = shifted.pinion.half_pitch - shifted.pinion.psi0
    base_point = min(
        shifted_section.segments["flank_neg"],
        key=lambda point: abs(math.hypot(*point) - shifted.pinion.base_r),
    )
    assert math.atan2(-base_point[1], base_point[0]) == pytest.approx(
        expected_space_angle, abs=1e-12
    )


def test_profile_shift_uses_module_and_normal_pressure_angle_exactly():
    p = SpurSetParams.with_defaults(2.0, 17, 43, profile_shift_1=0.25)
    geo = compute_set(p)
    expected_delta = 2.0 * p.module * 0.25 * math.tan(math.radians(20.0))
    assert geo.pinion.geometric_tooth_thickness - math.pi * p.module / 2.0 == pytest.approx(
        expected_delta, abs=1e-12
    )
    assert geo.pinion.geometric_tooth_thickness != pytest.approx(
        math.pi / 2.0 + 2.0 * 0.25 * math.tan(math.radians(20.0))
    )


def test_backlash_is_separate_from_shifted_geometric_tooth_thickness():
    p = SpurSetParams.with_defaults(
        2.0, 17, 43, profile_shift_1=0.2, backlash=0.12
    )
    geo = compute_set(p)
    for member in (geo.pinion, geo.gear):
        assert member.geometric_tooth_thickness - member.reference_tooth_thickness == pytest.approx(
            0.12 / 2.0, abs=1e-12
        )
        assert member.normal_geometric_tooth_thickness - member.normal_tooth_thickness == pytest.approx(
            0.12 / 2.0, abs=1e-12
        )
    assert sum(
        member.reference_tooth_thickness for member in (geo.pinion, geo.gear)
    ) == pytest.approx(
        sum(
            member.geometric_tooth_thickness
            for member in (geo.pinion, geo.gear)
        ) - p.backlash,
        abs=1e-12,
    )


def test_internal_profile_shift_is_applied_to_working_geometry():
    p = SpurSetParams.with_defaults(
        2.0,
        18,
        60,
        internal=True,
        profile_shift_1=0.15,
        profile_shift_2=0.35,
    )
    geo = compute_set(p)
    assert p.profile_shift_combination == pytest.approx(0.2)
    assert geo.pinion.profile_shift == p.profile_shift_1
    assert geo.gear.profile_shift == p.profile_shift_2
    assert geo.working_pressure_angle > geo.reference_pressure_angle
    assert geo.working_centre_distance > geo.reference_centre_distance


def test_old_json_without_iso_fields_loads_with_legacy_defaults(tmp_path):
    path = tmp_path / "old-spur.json"
    path.write_text(
        json.dumps(
            {
                "module": 2.0,
                "z1": 17,
                "z2": 43,
                "face_width": 20.0,
                "bore": 8.5,
                "hub_thickness": 5.0,
                "pressure_angle": 20.0,
                "helix_angle": 0.0,
                "hand": "right",
                "internal": False,
                "fillet_factor": 0.2,
                "backlash": 0.0,
                "rim_thickness": 5.0,
            }
        ),
        encoding="utf-8",
    )

    loaded = SpurSetParams.from_json(path)
    assert loaded.profile_shift_1 == 0.0
    assert loaded.profile_shift_2 == 0.0
    assert loaded.basic_rack_addendum_factor == 1.0
    assert loaded.basic_rack_clearance_factor == 0.25
    assert loaded.basic_rack_root_radius_factor == 0.38
    assert loaded.basic_rack_dedendum_factor == 1.25
    assert loaded.working_centre_distance is None
    assert loaded.root_geometry == "legacy"
    assert compute_set(loaded) == compute_set(
        SpurSetParams.with_defaults(2.0, 17, 43)
    )


def test_json_round_trip_writes_canonical_iso_field_names(tmp_path):
    params = SpurSetParams.with_defaults(
        2.0,
        17,
        43,
        profile_shift_1=0.1,
        profile_shift_2=0.2,
        basic_rack_addendum_factor=1.1,
    )
    path = tmp_path / "new-spur.json"
    params.to_json(path)
    raw = json.loads(path.read_text(encoding="utf-8"))

    assert raw["profile_shift_1"] == pytest.approx(0.1)
    assert raw["profile_shift_2"] == pytest.approx(0.2)
    assert "x1" not in raw
    assert SpurSetParams.from_json(path) == params


def test_transitional_json_profile_shift_aliases_are_migrated(tmp_path):
    path = tmp_path / "transitional-spur.json"
    path.write_text(
        json.dumps(
            {
                "module": 2.0,
                "z1": 17,
                "z2": 43,
                "face_width": 20.0,
                "bore": 8.5,
                "hub_thickness": 5.0,
                "x1": 0.1,
                "x2": -0.2,
            }
        ),
        encoding="utf-8",
    )
    loaded = SpurSetParams.from_json(path)
    assert loaded.profile_shift_1 == pytest.approx(0.1)
    assert loaded.profile_shift_2 == pytest.approx(-0.2)


@pytest.mark.parametrize("beta", BETAS)
def test_transverse_pressure_angle_matches_its_definition(beta):
    g = sets(beta)
    p = g.params
    assert math.tan(g.transverse_pressure_angle) == pytest.approx(
        math.tan(p.alpha_n) / math.cos(p.beta), rel=1e-14
    )


# --- radii -----------------------------------------------------------------


@pytest.mark.parametrize("member", MEMBERS)
@pytest.mark.parametrize("beta", BETAS)
def test_base_radius_is_the_pitch_radius_times_cos_alpha_t(member, beta):
    g = sets(beta)
    m = g.member(member)
    assert m.base_r == pytest.approx(
        m.pitch_r * math.cos(g.transverse_pressure_angle), rel=1e-14
    )


@pytest.mark.parametrize("member", MEMBERS)
@pytest.mark.parametrize("beta", BETAS)
def test_tooth_depths_are_measured_on_the_normal_module(member, beta):
    """The depths are normal quantities even though the radii are transverse ones."""
    g = sets(beta)
    m = g.member(member)
    assert m.tip_r - m.pitch_r == pytest.approx(ADDENDUM_FACTOR * 2.0, rel=1e-14)
    assert m.pitch_r - m.root_r == pytest.approx(DEDENDUM_FACTOR * 2.0, rel=1e-14)


@pytest.mark.parametrize("beta", BETAS)
@pytest.mark.parametrize("z1,z2", TOOTH_COUNTS)
def test_centre_distance_is_the_sum_of_the_pitch_radii(beta, z1, z2):
    g = sets(beta, z1, z2)
    assert g.centre_distance == pytest.approx(g.pinion.pitch_r + g.gear.pitch_r, rel=1e-14)


@pytest.mark.parametrize("beta", BETAS)
@pytest.mark.parametrize("z1,z2", TOOTH_COUNTS)
def test_centre_distance_solved_back_out_of_the_base_radii(beta, z1, z2):
    """a = (rb1 + rb2) / cos(alpha_t) - the involute's own statement of the same fact.

    Two independent routes to the centre distance agreeing is what says the base
    circles and the pressure angle belong to each other.
    """
    g = sets(beta, z1, z2)
    from_base = (g.pinion.base_r + g.gear.base_r) / math.cos(
        g.transverse_pressure_angle
    )
    assert from_base == pytest.approx(g.centre_distance, rel=1e-13)


# --- tooth thickness -------------------------------------------------------


@pytest.mark.parametrize("member", MEMBERS)
@pytest.mark.parametrize("beta", BETAS)
def test_tooth_thickness_at_the_pitch_circle_is_half_the_pitch(member, beta):
    """The defining property of a standard, unshifted tooth."""
    g = sets(beta)
    m = g.member(member)
    assert top_land(m.pitch_r, m.base_r, m.psi0) == pytest.approx(
        math.pi * g.transverse_module / 2.0, rel=1e-12
    )


@pytest.mark.parametrize("backlash", [0.0, 0.05, 0.2])
def test_backlash_comes_straight_off_the_tooth_thickness(backlash):
    g = compute_set(SpurSetParams.with_defaults(2.0, 17, 43, backlash=backlash))
    m = g.pinion
    assert top_land(m.pitch_r, m.base_r, m.psi0) == pytest.approx(
        math.pi * g.transverse_module / 2.0 - backlash / 2.0, rel=1e-12
    )


@pytest.mark.parametrize("backlash", [0.0, 0.05, 0.2])
@pytest.mark.parametrize("beta", BETAS)
def test_the_mesh_sees_the_backlash_once_and_not_twice(backlash, beta):
    """Both members are thinned by half of it, so the pair loses exactly it.

    The quiet failure this catches is applying the whole backlash to each
    member, which doubles the play at the mesh while every single-member check
    still passes. Stated as the pair's own property - the two tooth thicknesses
    at the pitch circles plus the backlash come to one circular pitch - so it
    cannot be satisfied by halving the number twice either.
    """
    g = compute_set(
        SpurSetParams.with_defaults(2.0, 17, 43, backlash=backlash, helix_angle=beta)
    )
    thicknesses = [
        top_land(m.pitch_r, m.base_r, m.psi0) for m in (g.pinion, g.gear)
    ]
    assert sum(thicknesses) == pytest.approx(g.circular_pitch - backlash, rel=1e-12)


@pytest.mark.parametrize("member", MEMBERS)
def test_psi0_is_the_half_thickness_carried_back_to_the_base_circle(geo, member):
    m = geo.member(member)
    assert m.psi0 == pytest.approx(
        top_land(m.pitch_r, m.base_r, m.psi0) / (2.0 * m.pitch_r)
        + inv(geo.transverse_pressure_angle),
        rel=1e-14,
    )


# --- contact ratio ---------------------------------------------------------


def test_anchor_contact_ratios(geo, helical):
    assert geo.transverse_contact_ratio == pytest.approx(1.6211, abs=1e-4)
    assert helical.transverse_contact_ratio == pytest.approx(1.5480, abs=1e-4)
    # with_defaults sizes a helical face width at exactly one axial pitch
    assert helical.axial_contact_ratio == pytest.approx(1.0, abs=5e-4)


@pytest.mark.parametrize("beta", BETAS)
@pytest.mark.parametrize("z1,z2", TOOTH_COUNTS)
def test_a_standard_pair_always_keeps_a_tooth_pair_in_contact(beta, z1, z2):
    assert sets(beta, z1, z2).transverse_contact_ratio > 1.0


def test_axial_contact_ratio_is_the_face_width_in_axial_pitches(helical):
    assert helical.axial_contact_ratio == pytest.approx(
        helical.params.face_width / helical.axial_pitch, rel=1e-12
    )


def test_undercut_limit_is_17_teeth_at_20_degrees_straight():
    assert undercut_limit(math.radians(20.0), 0.0) == pytest.approx(17.0973, abs=1e-4)


@pytest.mark.parametrize("beta", [10.0, 15.0, 30.0])
def test_a_helix_lets_a_gear_carry_fewer_teeth_before_undercutting(beta):
    g = sets(beta)
    assert undercut_limit(g.transverse_pressure_angle, g.params.beta) < undercut_limit(
        math.radians(20.0), 0.0
    )


# --- hand and twist --------------------------------------------------------


@pytest.mark.parametrize("beta", BETAS)
def test_the_two_members_are_cut_with_opposite_hands(beta):
    """Forced, not conventional: the placement has no flip in it."""
    g = sets(beta)
    assert g.pinion.beta == pytest.approx(-g.gear.beta, rel=1e-15)
    if beta != 0.0:
        assert {g.pinion.hand, g.gear.hand} == {"right", "left"}
    else:
        assert g.pinion.hand == g.gear.hand == "none"


@pytest.mark.parametrize("hand", ["right", "left"])
def test_the_pinion_takes_the_hand_that_was_asked_for(hand):
    g = compute_set(
        SpurSetParams.with_defaults(2.0, 17, 43, helix_angle=15.0, hand=hand)
    )
    assert g.pinion.hand == hand


@pytest.mark.parametrize("member", MEMBERS)
@pytest.mark.parametrize("beta", BETAS)
def test_twist_is_the_face_width_over_the_lead(member, beta):
    g = sets(beta)
    m = g.member(member)
    assert m.twist == pytest.approx(
        g.params.face_width * math.tan(m.beta) / m.pitch_r, rel=1e-14
    )


@pytest.mark.parametrize("member", MEMBERS)
def test_both_members_share_one_axial_pitch(helical, member):
    """What actually has to match for two helical gears to mesh on parallel axes.

    Not the twist angles - those differ, because the radii do. The axial pitch is
    the distance along the axis for one full tooth pitch of twist, and it is the
    same for both members precisely when the helix angles are equal and opposite.
    """
    m = helical.member(member)
    axial_pitch = m.angular_pitch / abs(m.twist / helical.params.face_width)
    assert axial_pitch == pytest.approx(helical.axial_pitch, rel=1e-12)


def test_straight_teeth_do_not_twist(geo):
    assert geo.pinion.twist == 0.0
    assert geo.gear.twist == 0.0
    assert phase_at(geo, "pinion", 999.0) == 0.0


# --- the tooth space section -----------------------------------------------


@pytest.mark.parametrize("member", MEMBERS)
@pytest.mark.parametrize("beta", BETAS)
def test_section_loop_is_closed_and_non_degenerate(beta, member):
    loop = tooth_space_section(sets(beta), member).loop_2d
    assert len(loop) > 20
    assert math.dist(loop[0], loop[-1]) > 1e-6      # closing point dropped, not repeated
    for a, b in zip(loop, loop[1:]):
        assert math.dist(a, b) > 1e-12


@pytest.mark.parametrize("member", MEMBERS)
@pytest.mark.parametrize("beta", BETAS)
def test_section_runs_counter_clockwise(beta, member):
    loop = tooth_space_section(sets(beta), member).loop_2d
    area = 0.5 * sum(
        loop[i][0] * loop[(i + 1) % len(loop)][1]
        - loop[(i + 1) % len(loop)][0] * loop[i][1]
        for i in range(len(loop))
    )
    assert area > 0.0


@pytest.mark.parametrize("member", MEMBERS)
@pytest.mark.parametrize("beta", BETAS)
def test_section_is_symmetric_about_the_x_axis(beta, member):
    """The space is centred on angle 0, so every point has a mirror partner."""
    loop = tooth_space_section(sets(beta), member).loop_2d
    ys = sorted(round(y, 9) for _, y in loop)
    assert all(a == pytest.approx(-b, abs=1e-9) for a, b in zip(ys, reversed(ys)))


@pytest.mark.parametrize("member", MEMBERS)
@pytest.mark.parametrize("beta", BETAS)
def test_section_spans_exactly_root_to_cap(beta, member):
    s = tooth_space_section(sets(beta), member)
    radii = [math.hypot(x, y) for x, y in s.loop_2d]
    assert min(radii) == pytest.approx(s.r_root, abs=1e-9)
    assert max(radii) == pytest.approx(s.r_cap, abs=1e-9)
    assert s.r_cap > s.r_tip


@pytest.mark.parametrize("member", MEMBERS)
def test_the_cut_cap_clears_the_blank_radially(geo, member):
    """The cut must reach past the tip or it leaves a ring of uncut material."""
    s = tooth_space_section(geo, member)
    assert s.r_cap > geo.member(member).tip_r


@pytest.mark.parametrize("member", MEMBERS)
def test_the_root_fillet_is_fitted_for_the_anchor_case(geo, member):
    assert tooth_space_section(geo, member).filleted


@pytest.mark.parametrize("member", MEMBERS)
@pytest.mark.parametrize("beta", BETAS)
def test_every_section_is_the_same_shape(beta, member):
    """The one real simplification over the bevel case: only the phase changes.

    A bevel section is a different size at every cone distance. A spur section
    is not, so the builder can generate the profile once and place it twice.
    """
    g = sets(beta)
    a = tooth_space_section(g, member, z=0.0)
    b = tooth_space_section(g, member, z=g.params.face_width)
    assert a.loop_2d == b.loop_2d
    assert (a.r_root, a.r_tip, a.r_cap) == (b.r_root, b.r_tip, b.r_cap)


@pytest.mark.parametrize("member", MEMBERS)
@pytest.mark.parametrize("beta", BETAS)
def test_the_back_section_is_the_front_one_rotated_by_the_twist(beta, member):
    g = sets(beta)
    m = g.member(member)
    b = g.params.face_width
    front = tooth_space_section(g, member, z=0.0).loop_3d()
    back = tooth_space_section(g, member, z=b).loop_3d()
    c, s = math.cos(m.twist), math.sin(m.twist)
    for (x, y, z), q in zip(front, back):
        assert z == pytest.approx(0.0, abs=1e-12)
        assert math.dist((x * c - y * s, x * s + y * c, b), q) < 1e-12


@pytest.mark.parametrize("member", MEMBERS)
def test_the_clocking_reference_plane_is_unrotated(geo, member):
    """z = 0 is where a pair is phased, so nothing may have twisted by it."""
    for g in (geo, compute_set(ANCHOR_HELICAL)):
        assert tooth_space_section(g, member, z=0.0).phase == 0.0


# --- overshoot and the guide curve -----------------------------------------


@pytest.mark.parametrize("beta", BETAS)
def test_sections_are_pushed_clear_of_both_end_faces(beta):
    """A cut finishing tangent to a real face is rejected as zero-thickness."""
    g = sets(beta)
    ov = end_overshoot(g)
    assert ov > 0.0
    assert -ov < 0.0
    assert g.params.face_width + ov > g.params.face_width


@pytest.mark.parametrize("member", MEMBERS)
@pytest.mark.parametrize("beta", [10.0, 15.0, 30.0])
def test_the_guide_helix_touches_the_cap_vertex_of_both_end_sections(beta, member):
    """The property a guided loft cut depends on.

    A guide curve that merely passes *near* a spline is the classic way such a
    loft fails, so `split_cap` puts a real vertex on the space centreline and the
    guide is generated to pass through it. If this ever stops holding, the loft
    stops being buildable - which is why it is asserted here rather than
    discovered in SOLIDWORKS.
    """
    g = sets(beta)
    ov = end_overshoot(g)
    z_lo, z_hi = -ov, g.params.face_width + ov
    guide = guide_helix(g, member, z_lo, z_hi)

    for z in (z_lo, z_hi):
        segments = tooth_space_section(g, member, z=z, split_cap=True).segments_3d()
        vertex = segments["cap_neg"][-1]
        assert segments["cap_pos"][0] == vertex
        assert min(math.dist(vertex, point) for point in guide) < 1e-9


@pytest.mark.parametrize("member", MEMBERS)
@pytest.mark.parametrize("beta", [10.0, 15.0, 30.0])
def test_the_guide_is_a_helix_of_constant_radius(beta, member):
    g = sets(beta)
    guide = guide_helix(g, member, 0.0, g.params.face_width)
    r_cap = tooth_space_section(g, member).r_cap
    for x, y, _ in guide:
        assert math.hypot(x, y) == pytest.approx(r_cap, abs=1e-12)


@pytest.mark.parametrize("member", MEMBERS)
def test_splitting_the_cap_does_not_change_the_loop(geo, member):
    plain = tooth_space_section(geo, member, split_cap=False)
    split = tooth_space_section(geo, member, split_cap=True)
    assert plain.loop_2d == split.loop_2d
    assert "cap" not in split.segments
    assert split.segments["cap_neg"] + split.segments["cap_pos"][1:] == (
        plain.segments["cap"]
    )


def test_to_axial_3d_is_a_plain_lift_when_nothing_has_twisted():
    assert to_axial_3d(3.0, 4.0, 0.0, 7.0) == pytest.approx((3.0, 4.0, 7.0))


# --- the blank -------------------------------------------------------------


@pytest.mark.parametrize("member", MEMBERS)
@pytest.mark.parametrize("beta", BETAS)
def test_blank_runs_from_the_bore_to_the_tip_and_back(beta, member):
    g = sets(beta)
    outline = blank_outline(g, member)
    m = g.member(member)
    radii = [r for r, _ in outline]
    heights = [z for _, z in outline]
    assert min(radii) == pytest.approx(g.params.bore / 2.0)
    assert max(radii) == pytest.approx(m.tip_r)
    assert min(heights) == 0.0
    assert max(heights) == pytest.approx(g.params.face_width + g.params.hub_thickness)


@pytest.mark.parametrize("member", MEMBERS)
def test_blank_front_face_sits_at_the_origin(geo, member):
    """Front face at z = 0 is what the whole coordinate system is anchored on."""
    outline = blank_outline(geo, member)
    assert outline[0][1] == 0.0
    assert outline[1][1] == 0.0


@pytest.mark.parametrize("member", MEMBERS)
def test_blank_covers_the_full_face_width_at_the_tip_radius(geo, member):
    outline = blank_outline(geo, member)
    m = geo.member(member)
    at_tip = [z for r, z in outline if r == pytest.approx(m.tip_r)]
    assert min(at_tip) == 0.0
    assert max(at_tip) == pytest.approx(geo.params.face_width)


@pytest.mark.parametrize("member", MEMBERS)
def test_the_hub_never_eats_into_the_teeth(geo, member):
    """A boss standing proud of the root circle would bridge the tooth spaces."""
    outline = blank_outline(geo, member)
    behind = [r for r, z in outline if z > geo.params.face_width + 1e-9]
    assert behind
    assert max(behind) <= geo.member(member).root_r + 1e-9


@pytest.mark.parametrize("member", MEMBERS)
def test_no_hub_leaves_a_plain_cylinder(member):
    g = compute_set(SpurSetParams.with_defaults(2.0, 17, 43, hub_thickness=0.0))
    outline = blank_outline(g, member)
    assert len(outline) == 4
    assert max(z for _, z in outline) == pytest.approx(g.params.face_width)


# --- what the blank dimension plan assumes ---------------------------------
#
# gears/sw/spur_part.py indexes into this outline by position and counts on the
# horizontal/vertical alternation for its degree-of-freedom arithmetic. Both are
# properties of the geometry, so both can be pinned here rather than discovered
# when require_fully_defined refuses a sketch in SOLIDWORKS.


@pytest.mark.parametrize("member", MEMBERS)
@pytest.mark.parametrize("hub", [0.0, 5.0])
def test_blank_outline_has_the_length_the_builder_switches_on(member, hub):
    g = compute_set(SpurSetParams.with_defaults(2.0, 17, 43, hub_thickness=hub))
    outline = blank_outline(g, member)
    assert len(outline) == (6 if hub else 4)
    assert (len(outline) > 4) is bool(hub)


@pytest.mark.parametrize("member", MEMBERS)
def test_blank_outline_vertices_are_where_the_dimensions_look_for_them(member):
    g = compute_set(SpurSetParams.with_defaults(2.0, 17, 43, hub_thickness=5.0))
    outline = blank_outline(g, member)
    p, m = g.params, g.member(member)

    assert outline[0] == (pytest.approx(p.bore / 2.0), 0.0)          # bore, front
    assert outline[1] == (pytest.approx(m.tip_r), 0.0)               # tip, front
    assert outline[2][1] == pytest.approx(p.face_width)              # back face
    assert outline[3][1] == pytest.approx(p.face_width)              # hub radius
    assert outline[4][1] == pytest.approx(p.face_width + p.hub_thickness)


@pytest.mark.parametrize("member", MEMBERS)
@pytest.mark.parametrize("hub", [0.0, 5.0])
def test_blank_segments_alternate_horizontal_and_vertical(member, hub):
    """Every segment gets exactly one relation, which is what the DOF count assumes.

    A segment that was neither flat nor cylindrical would take no relation at
    all, and the sketch would come out one degree of freedom short of defined
    with nothing in the dimension plan to absorb it.
    """
    g = compute_set(SpurSetParams.with_defaults(2.0, 17, 43, hub_thickness=hub))
    outline = blank_outline(g, member)
    n = len(outline)

    kinds = []
    for i in range(n):
        (r1, z1), (r2, z2) = outline[i], outline[(i + 1) % n]
        horizontal = abs(z1 - z2) < 1e-9
        vertical = abs(r1 - r2) < 1e-9
        assert horizontal != vertical, f"segment {i} is neither flat nor cylindrical"
        kinds.append("H" if horizontal else "V")

    assert kinds == ["H", "V"] * (n // 2)


# --- how many loft sections the twist needs --------------------------------
#
# A loft chords each profile point between consecutive sections, and a helix is
# an arc, so too few sections cut inside the true helicoid. Measured in
# SOLIDWORKS on the helical anchor pinion: two sections plus a guide curve left
# the mid-face profile 0.198 mm proud of the end profile and the cut 0.9% short
# of the helicoid's volume; six sections brought both to 0.011 mm and -0.068%.


@pytest.mark.parametrize("member", MEMBERS)
def test_straight_teeth_need_only_two_sections(member):
    assert section_count(compute_set(ANCHOR), member) == 2
    assert len(section_heights(compute_set(ANCHOR), member)) == 2


@pytest.mark.parametrize("member", MEMBERS)
@pytest.mark.parametrize("beta", [10.0, 15.0, 20.0, 30.0])
def test_helical_teeth_need_more_than_two(beta, member):
    assert section_count(sets(beta), member) > 2


@pytest.mark.parametrize("member", MEMBERS)
@pytest.mark.parametrize("beta", [10.0, 15.0, 20.0, 30.0, 40.0])
def test_the_chord_sagitta_stays_inside_the_tolerance(beta, member):
    """The property the count is chosen for, checked at the worst radius."""
    g = sets(beta)
    m = g.member(member)
    n = section_count(g, member)
    step = abs(m.twist) / (n - 1)
    sagitta = m.tip_r * (1.0 - math.cos(step / 2.0))
    assert sagitta <= MAX_SECTION_SAGITTA_MM


@pytest.mark.parametrize("member", MEMBERS)
@pytest.mark.parametrize("beta", [10.0, 15.0, 30.0])
def test_a_tighter_tolerance_asks_for_more_sections(beta, member):
    g = sets(beta)
    loose = section_count(g, member, max_sagitta=0.05)
    tight = section_count(g, member, max_sagitta=0.005)
    assert tight > loose


@pytest.mark.parametrize("member", MEMBERS)
@pytest.mark.parametrize("beta", BETAS)
def test_section_heights_span_the_overshot_face_evenly(beta, member):
    g = sets(beta)
    heights = section_heights(g, member)
    overshoot = end_overshoot(g)

    assert heights[0] == pytest.approx(-overshoot)
    assert heights[-1] == pytest.approx(g.params.face_width + overshoot)
    gaps = [b - a for a, b in zip(heights, heights[1:])]
    assert all(gap == pytest.approx(gaps[0]) for gap in gaps)


@pytest.mark.parametrize("member", MEMBERS)
def test_every_section_still_lies_on_the_same_helix(helical, member):
    """Adding sections must not disturb what the guide curve is drawn through."""
    heights = section_heights(helical, member)
    guide = guide_helix(helical, member, heights[0], heights[-1])

    for z in heights:
        segments = tooth_space_section(
            helical, member, z=z, split_cap=True
        ).segments_3d()
        vertex = segments["cap_neg"][-1]
        assert min(math.dist(vertex, point) for point in guide) < 1e-6
