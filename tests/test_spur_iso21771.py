"""Independent equation checks for the ISO 21771 spur implementation.

The expected values in this module are deliberately formed from equations
written here.  The production geometry is used only for the values being
checked, never for expected-value calculation.
"""

from __future__ import annotations

import math

import pytest

from gears.spur import mesh
from gears.spur.geometry import compute_set, tooth_space_section
from gears.spur.params import SpurSetParams
from gears.spur.validate import validate


def _inv(alpha: float) -> float:
    """The involute function, written locally for independent expectations."""
    return math.tan(alpha) - alpha


def _inverse_inv(value: float) -> float:
    """Invert ``tan(alpha) - alpha`` by an independent bounded bisection."""
    if value < -1e-14:
        raise ValueError("expected involute is outside the physical domain")
    if abs(value) <= 1e-14:
        return 0.0
    lo = 0.0
    hi = math.nextafter(math.pi / 2.0, 0.0)
    for _ in range(120):
        mid = (lo + hi) / 2.0
        if _inv(mid) < value:
            lo = mid
        else:
            hi = mid
    return (lo + hi) / 2.0


def _expected_pair(
    module: float,
    z1: int,
    z2: int,
    alpha_deg: float,
    beta_deg: float,
    x1: float,
    x2: float,
    backlash: float = 0.0,
    internal: bool = False,
    face_width: float = 20.0,
):
    """Return independent reference/working values for one gear pair."""
    alpha_n = math.radians(alpha_deg)
    beta = math.radians(beta_deg)
    cos_beta = math.cos(beta)
    m_t = module / cos_beta
    alpha_t = math.atan2(math.tan(alpha_n), cos_beta)
    q = z2 - z1 if internal else z1 + z2
    combination = x2 - x1 if internal else x1 + x2
    a_ref = m_t * q / 2.0

    target = _inv(alpha_t) + 2.0 * combination * math.tan(alpha_n) / q
    alpha_w = alpha_t if abs(combination) <= 1e-15 else _inverse_inv(target)
    a_work = a_ref * math.cos(alpha_t) / math.cos(alpha_w)
    tip_alteration = (
        (a_ref - a_work) / module + combination
        if internal
        else (a_work - a_ref) / module - combination
    )

    members = []
    for z, x, ring in ((z1, x1, False), (z2, x2, internal)):
        r_ref = m_t * z / 2.0
        r_base = r_ref * math.cos(alpha_t)
        s_n_geometric = module * (
            math.pi / 2.0 + 2.0 * x * math.tan(alpha_n)
        )
        s_t_geometric = s_n_geometric / cos_beta
        s_t_actual = s_t_geometric - backlash / 2.0
        s_n_actual = s_t_actual * cos_beta
        if ring:
            addendum = module * (1.0 - x + tip_alteration)
            dedendum = module * (1.25 + x)
            r_tip = r_ref - addendum
            r_root = r_ref + dedendum
            space_width = math.pi * m_t - s_t_actual
            psi0 = space_width / (2.0 * r_ref) + _inv(alpha_t)
        else:
            addendum = module * (1.0 + x + tip_alteration)
            dedendum = module * (1.25 - x)
            r_tip = r_ref + addendum
            r_root = r_ref - dedendum
            psi0 = s_t_actual / (2.0 * r_ref) + _inv(alpha_t)

        r_work = r_base / math.cos(alpha_w)
        alpha_at_work = _inv(alpha_w)
        if ring:
            working_tooth = 2.0 * r_work * (
                math.pi / z - (psi0 - alpha_at_work)
            )
        else:
            working_tooth = 2.0 * r_work * (psi0 - alpha_at_work)
        members.append(
            {
                "reference_r": r_ref,
                "base_r": r_base,
                "working_r": r_work,
                "tip_r": r_tip,
                "root_r": r_root,
                "s_n_geometric": s_n_geometric,
                "s_t_geometric": s_t_geometric,
                "s_n_actual": s_n_actual,
                "s_t_actual": s_t_actual,
                "psi0": psi0,
                "working_tooth": working_tooth,
            }
        )

    branches = [
        math.sqrt(max(0.0, item["tip_r"] ** 2 - item["base_r"] ** 2))
        for item in members
    ]
    action = (
        branches[0] - branches[1] + a_work * math.sin(alpha_w)
        if internal
        else branches[0] + branches[1] - a_work * math.sin(alpha_w)
    )
    epsilon_alpha = max(
        0.0,
        action / (math.pi * m_t * math.cos(alpha_t)),
    )
    epsilon_beta = (
        face_width * abs(math.sin(beta)) / (math.pi * module)
        if abs(beta) > 1e-15
        else 0.0
    )
    if internal:
        working_depth = a_work + members[0]["tip_r"] - members[1]["tip_r"]
        tip_clearance_1 = members[1]["root_r"] - a_work - members[0]["tip_r"]
        tip_clearance_2 = members[1]["tip_r"] - a_work - members[0]["root_r"]
    else:
        working_depth = members[0]["tip_r"] + members[1]["tip_r"] - a_work
        tip_clearance_1 = a_work - members[0]["tip_r"] - members[1]["root_r"]
        tip_clearance_2 = a_work - members[1]["tip_r"] - members[0]["root_r"]
    return {
        "m_t": m_t,
        "alpha_t": alpha_t,
        "a_ref": a_ref,
        "a_work": a_work,
        "tip_alteration": tip_alteration,
        "alpha_w": alpha_w,
        "epsilon_alpha": epsilon_alpha,
        "epsilon_beta": epsilon_beta,
        "epsilon_gamma": epsilon_alpha + epsilon_beta,
        "working_depth": working_depth,
        "tip_clearance_1": tip_clearance_1,
        "tip_clearance_2": tip_clearance_2,
        "members": members,
    }


def _assert_pair_matches_expected(geo, expected, internal: bool) -> None:
    """Compare all pair/member quantities represented by the model."""
    assert geo.transverse_module == pytest.approx(expected["m_t"], abs=1e-11)
    assert geo.reference_pressure_angle == pytest.approx(
        expected["alpha_t"], abs=1e-11
    )
    assert geo.reference_centre_distance == pytest.approx(
        expected["a_ref"], abs=1e-11
    )
    assert geo.working_centre_distance == pytest.approx(
        expected["a_work"], abs=1e-11
    )
    assert geo.working_pressure_angle == pytest.approx(
        expected["alpha_w"], abs=1e-11
    )
    assert geo.transverse_contact_ratio == pytest.approx(
        expected["epsilon_alpha"], abs=1e-11
    )
    assert geo.overlap_ratio == pytest.approx(expected["epsilon_beta"], abs=1e-11)
    assert geo.total_contact_ratio == pytest.approx(
        expected["epsilon_gamma"], abs=1e-11
    )
    assert geo.working_depth == pytest.approx(expected["working_depth"], abs=1e-11)
    assert geo.tip_clearance_1 == pytest.approx(
        expected["tip_clearance_1"], abs=1e-11
    )
    assert geo.tip_clearance_2 == pytest.approx(
        expected["tip_clearance_2"], abs=1e-11
    )
    assert geo.minimum_tip_clearance == pytest.approx(
        min(expected["tip_clearance_1"], expected["tip_clearance_2"]),
        abs=1e-11,
    )
    assert geo.tip_alteration_coefficient == pytest.approx(
        expected["tip_alteration"], abs=1e-11
    )

    for actual, reference in zip(
        (geo.pinion, geo.gear), expected["members"]
    ):
        assert actual.reference_d == pytest.approx(
            2.0 * reference["reference_r"], abs=1e-11
        )
        assert actual.base_d == pytest.approx(
            2.0 * reference["base_r"], abs=1e-11
        )
        assert actual.working_d == pytest.approx(
            2.0 * reference["working_r"], abs=1e-11
        )
        assert actual.tip_d == pytest.approx(
            2.0 * reference["tip_r"], abs=1e-11
        )
        assert actual.root_d == pytest.approx(
            2.0 * reference["root_r"], abs=1e-11
        )
        assert actual.geometric_tooth_thickness == pytest.approx(
            reference["s_t_geometric"], abs=1e-11
        )
        assert actual.reference_tooth_thickness == pytest.approx(
            reference["s_t_actual"], abs=1e-11
        )
        assert actual.normal_geometric_tooth_thickness == pytest.approx(
            reference["s_n_geometric"], abs=1e-11
        )
        assert actual.normal_tooth_thickness == pytest.approx(
            reference["s_n_actual"], abs=1e-11
        )
        assert actual.psi0 == pytest.approx(reference["psi0"], abs=1e-11)

        # ``working tooth thickness`` is an implied quantity, not a stored
        # production field. Derive it from the reported involute phase and
        # working radius to verify that the actual profile has that thickness.
        inv_at_work = _inv(expected["alpha_w"])
        if actual.internal:
            actual_working_tooth = 2.0 * actual.working_r * (
                actual.half_pitch - (actual.psi0 - inv_at_work)
            )
        else:
            actual_working_tooth = 2.0 * actual.working_r * (
                actual.psi0 - inv_at_work
            )
        assert actual_working_tooth == pytest.approx(
            reference["working_tooth"], abs=1e-10
        )


def _independent_rack_root_point(
    reference_r, alpha, depth, rho, psi0, half_pitch, phi
):
    """Independent rolling-envelope point used by active-contact fixtures."""
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


def _independent_rack_undercut_root_form(member, module, rho):
    """Calculate the undercut SOI radius without production SOI helpers."""
    reference_r = member.reference_r
    alpha = math.acos(member.base_r / reference_r)
    depth = member.dedendum
    centre_v = -depth + rho
    centre_u = math.tan(alpha) * centre_v - rho / math.cos(alpha)
    phi_root = centre_u / reference_r
    tangent_v = centre_v - rho * math.sin(alpha)
    phi_transition = tangent_v * (1.0 + math.tan(alpha) ** 2) / (
        math.tan(alpha) * reference_r
    )

    corner_depth = depth - rho * (1.0 - math.sin(alpha))
    undercut = reference_r * math.sin(alpha) ** 2 - corner_depth < 0.0
    if not undercut:
        point = _independent_rack_root_point(
            reference_r, alpha, depth, rho, member.psi0, member.half_pitch,
            phi_transition,
        )
        return math.hypot(*point)

    base_space_angle = member.half_pitch - member.psi0

    def residual(phi):
        point = _independent_rack_root_point(
            reference_r, alpha, depth, rho, member.psi0, member.half_pitch, phi
        )
        radius = math.hypot(*point)
        if radius < member.base_r:
            return None
        roll = math.sqrt((radius / member.base_r) ** 2 - 1.0)
        return math.atan2(point[1], point[0]) - (
            base_space_angle + roll - math.atan(roll)
        )

    lo = min(phi_transition, phi_root)
    hi = max(phi_transition, phi_root)
    previous = None
    bracket = None
    for index in range(4097):
        phi = lo + (hi - lo) * index / 4096.0
        value = residual(phi)
        if value is None:
            previous = None
            continue
        if abs(value) < 1e-13:
            bracket = (phi, phi)
            break
        if previous is not None and previous[1] * value < 0.0:
            bracket = (previous[0], phi)
            break
        previous = (phi, value)
    assert bracket is not None
    lo, hi = bracket
    if lo != hi:
        f_lo = residual(lo)
        for _ in range(120):
            mid = (lo + hi) / 2.0
            f_mid = residual(mid)
            assert f_mid is not None
            if abs(f_mid) < 1e-14:
                lo = hi = mid
                break
            if f_lo * f_mid <= 0.0:
                hi = mid
            else:
                lo, f_lo = mid, f_mid
    point = _independent_rack_root_point(
        reference_r, alpha, depth, rho, member.psi0, member.half_pitch,
        (lo + hi) / 2.0,
    )
    return math.hypot(*point)


def _independent_q(member, radius):
    return math.sqrt(max(0.0, radius * radius - member.base_r ** 2))


def _independent_nominal_path(geo):
    q1w = _independent_q(geo.pinion, geo.pinion.working_r)
    q2w = _independent_q(geo.gear, geo.gear.working_r)
    q1a = _independent_q(geo.pinion, geo.pinion.tip_r)
    q2a = _independent_q(geo.gear, geo.gear.tip_r)
    if geo.params.internal:
        return (q1a - q1w) + (q2w - q2a)
    return (q1a - q1w) + (q2a - q2w)


EXTERNAL_STRAIGHT_CASES = (
    ("standard", 0.0, 0.0),
    ("positive_pinion", 0.3, 0.0),
    ("negative_pinion", -0.3, 0.0),
    ("zero_total_unequal", 0.5, -0.5),
    ("positive_total", 0.3, 0.2),
)


@pytest.mark.parametrize("case,x1,x2", EXTERNAL_STRAIGHT_CASES)
def test_external_straight_matrix_matches_independent_iso_equations(
    case, x1, x2
):
    p = SpurSetParams.with_defaults(
        2.0,
        17,
        43,
        face_width=20.0,
        profile_shift_1=x1,
        profile_shift_2=x2,
    )
    geo = compute_set(p)
    expected = _expected_pair(2.0, 17, 43, 20.0, 0.0, x1, x2)
    _assert_pair_matches_expected(geo, expected, internal=False)


@pytest.mark.parametrize("beta", [15.0, 30.0])
@pytest.mark.parametrize("x1,x2", [(0.3, -0.1), (0.4, 0.2)])
def test_external_helical_matrix_matches_independent_iso_equations(beta, x1, x2):
    p = SpurSetParams.with_defaults(
        2.0,
        17,
        43,
        face_width=20.0,
        helix_angle=beta,
        profile_shift_1=x1,
        profile_shift_2=x2,
    )
    geo = compute_set(p)
    expected = _expected_pair(2.0, 17, 43, 20.0, beta, x1, x2)
    _assert_pair_matches_expected(geo, expected, internal=False)
    assert geo.pinion.beta == pytest.approx(math.radians(beta), abs=1e-12)
    assert geo.gear.beta == pytest.approx(-math.radians(beta), abs=1e-12)


INTERNAL_STRAIGHT_CASES = (
    ("standard", 0.0, 0.0),
    ("pinion_shift", 0.3, 0.0),
    ("zero_total_unequal", 0.3, -0.3),
    ("positive_total", 0.0, 0.3),
)


@pytest.mark.parametrize("case,x1,x2", INTERNAL_STRAIGHT_CASES)
def test_internal_straight_matrix_matches_independent_iso_equations(
    case, x1, x2
):
    p = SpurSetParams.with_defaults(
        2.0,
        18,
        60,
        internal=True,
        face_width=20.0,
        profile_shift_1=x1,
        profile_shift_2=x2,
    )
    geo = compute_set(p)
    expected = _expected_pair(
        2.0, 18, 60, 20.0, 0.0, x1, x2, internal=True
    )
    _assert_pair_matches_expected(geo, expected, internal=True)
    assert geo.working_centre_distance == pytest.approx(
        geo.gear.working_r - geo.pinion.working_r, abs=1e-11
    )


def test_internal_helical_profile_shift_matches_independent_equations():
    p = SpurSetParams.with_defaults(
        2.0,
        18,
        60,
        internal=True,
        face_width=20.0,
        helix_angle=15.0,
        profile_shift_1=0.3,
        profile_shift_2=0.1,
    )
    geo = compute_set(p)
    expected = _expected_pair(
        2.0, 18, 60, 20.0, 15.0, 0.3, 0.1, internal=True
    )
    _assert_pair_matches_expected(geo, expected, internal=True)
    assert geo.pinion.beta == pytest.approx(geo.gear.beta, abs=1e-12)


@pytest.mark.parametrize(
    "internal,beta,z1,z2,x1,x2",
    [
        (False, 0.0, 18, 18, 0.0, 0.0),
        (False, 0.0, 18, 18, 0.8, 0.8),
        (False, 0.0, 18, 18, -0.3, -0.2),
        (False, 15.0, 17, 43, 0.4, 0.2),
        (True, 0.0, 18, 60, 0.0, 0.3),
    ],
)
def test_tip_alteration_matrix_is_independent_and_pair_level(
    internal, beta, z1, z2, x1, x2
):
    """Check k, h_w, c1/c2, and both actual tip diameters independently."""
    p = SpurSetParams.with_defaults(
        2.0,
        z1,
        z2,
        internal=internal,
        helix_angle=beta,
        profile_shift_1=x1,
        profile_shift_2=x2,
    )
    geo = compute_set(p)
    expected = _expected_pair(
        2.0,
        z1,
        z2,
        20.0,
        beta,
        x1,
        x2,
        internal=internal,
        face_width=p.face_width,
    )
    _assert_pair_matches_expected(geo, expected, internal=internal)
    assert geo.pinion.tip_d == pytest.approx(
        2.0 * expected["members"][0]["tip_r"], abs=1e-11
    )
    assert geo.gear.tip_d == pytest.approx(
        2.0 * expected["members"][1]["tip_r"], abs=1e-11
    )


def test_automatic_clearance_prevents_the_high_positive_shift_interference_case():
    automatic = SpurSetParams.with_defaults(
        2.0, 18, 18, profile_shift_1=0.8, profile_shift_2=0.8
    )
    geo = compute_set(automatic)
    expected = _expected_pair(2.0, 18, 18, 20.0, 0.0, 0.8, 0.8)

    assert geo.tip_alteration_coefficient < 0.0
    assert geo.minimum_tip_clearance == pytest.approx(0.5, abs=1e-11)
    assert validate(automatic).ok
    _assert_pair_matches_expected(geo, expected, internal=False)

    legacy = SpurSetParams.with_defaults(
        2.0,
        18,
        18,
        profile_shift_1=0.8,
        profile_shift_2=0.8,
        tip_alteration_mode="legacy",
    )
    legacy_geo = compute_set(legacy)
    assert legacy_geo.minimum_tip_clearance < 0.0
    assert not validate(legacy).ok
    assert {issue.field for issue in validate(legacy).errors} >= {
        "tip_clearance_1", "tip_clearance_2"
    }


def test_explicit_tip_alteration_override_is_separate_from_profile_shift():
    automatic = compute_set(
        SpurSetParams.with_defaults(
            2.0, 18, 18, profile_shift_1=0.8, profile_shift_2=0.8
        )
    )
    explicit = compute_set(
        SpurSetParams.with_defaults(
            2.0,
            18,
            18,
            profile_shift_1=0.8,
            profile_shift_2=0.8,
            tip_alteration_mode="explicit",
            tip_alteration_coefficient=automatic.tip_alteration_coefficient,
        )
    )
    for field in (
        "reference_r", "base_r", "working_r", "reference_tooth_thickness"
    ):
        assert getattr(explicit.pinion, field) == pytest.approx(
            getattr(automatic.pinion, field), abs=1e-12
        )
    assert explicit.working_pressure_angle == pytest.approx(
        automatic.working_pressure_angle, abs=1e-12
    )
    assert explicit.pinion.tip_d == pytest.approx(automatic.pinion.tip_d, abs=1e-12)


def test_profile_shift_sign_for_internal_pair_is_not_external_sum():
    p = SpurSetParams.with_defaults(
        2.0,
        18,
        60,
        internal=True,
        face_width=20.0,
        profile_shift_1=0.3,
        profile_shift_2=0.0,
    )
    geo = compute_set(p)
    correct = _expected_pair(2.0, 18, 60, 20.0, 0.0, 0.3, 0.0, internal=True)
    wrong = _expected_pair(2.0, 18, 60, 20.0, 0.0, -0.3, 0.0, internal=True)

    assert geo.working_centre_distance == pytest.approx(correct["a_work"], abs=1e-11)
    assert geo.working_centre_distance != pytest.approx(wrong["a_work"], abs=1e-8)
    assert geo.working_centre_distance < geo.reference_centre_distance


def test_scaling_all_linear_inputs_preserves_angles_ratios_and_dimensionless_x():
    small = compute_set(
        SpurSetParams.with_defaults(
            2.0,
            17,
            43,
            face_width=20.0,
            bore=8.0,
            hub_thickness=5.0,
            backlash=0.12,
            helix_angle=15.0,
            profile_shift_1=0.4,
            profile_shift_2=0.2,
        )
    )
    large = compute_set(
        SpurSetParams.with_defaults(
            4.0,
            17,
            43,
            face_width=40.0,
            bore=16.0,
            hub_thickness=10.0,
            backlash=0.24,
            helix_angle=15.0,
            profile_shift_1=0.4,
            profile_shift_2=0.2,
        )
    )

    for field in (
        "transverse_module",
        "circular_pitch",
        "reference_centre_distance",
        "working_centre_distance",
        "axial_pitch",
        "whole_depth",
    ):
        assert getattr(large, field) == pytest.approx(2.0 * getattr(small, field))

    for small_member, large_member in zip(
        (small.pinion, small.gear), (large.pinion, large.gear)
    ):
        for field in (
            "reference_r",
            "base_r",
            "working_r",
            "tip_r",
            "root_r",
            "geometric_tooth_thickness",
            "reference_tooth_thickness",
            "normal_geometric_tooth_thickness",
            "normal_tooth_thickness",
        ):
            assert getattr(large_member, field) == pytest.approx(
                2.0 * getattr(small_member, field)
            )
        assert large_member.profile_shift == small_member.profile_shift
        assert large_member.psi0 == pytest.approx(small_member.psi0, abs=1e-12)
        assert large_member.half_pitch == pytest.approx(
            small_member.half_pitch, abs=1e-12
        )
        assert large_member.twist == pytest.approx(
            small_member.twist, abs=1e-12
        )

    assert large.reference_pressure_angle == pytest.approx(
        small.reference_pressure_angle, abs=1e-12
    )
    assert large.working_pressure_angle == pytest.approx(
        small.working_pressure_angle, abs=1e-12
    )
    assert large.transverse_contact_ratio == pytest.approx(
        small.transverse_contact_ratio, abs=1e-12
    )
    assert large.overlap_ratio == pytest.approx(small.overlap_ratio, abs=1e-12)
    assert large.total_contact_ratio == pytest.approx(
        small.total_contact_ratio, abs=1e-12
    )


def test_swapping_external_members_preserves_pair_geometry():
    first = compute_set(
        SpurSetParams.with_defaults(
            2.0,
            17,
            43,
            face_width=20.0,
            profile_shift_1=0.4,
            profile_shift_2=0.2,
        )
    )
    swapped = compute_set(
        SpurSetParams.with_defaults(
            2.0,
            43,
            17,
            face_width=20.0,
            profile_shift_1=0.2,
            profile_shift_2=0.4,
        )
    )

    assert swapped.reference_centre_distance == pytest.approx(
        first.reference_centre_distance, abs=1e-11
    )
    assert swapped.working_centre_distance == pytest.approx(
        first.working_centre_distance, abs=1e-11
    )
    assert swapped.working_pressure_angle == pytest.approx(
        first.working_pressure_angle, abs=1e-11
    )
    assert swapped.transverse_contact_ratio == pytest.approx(
        first.transverse_contact_ratio, abs=1e-11
    )
    assert swapped.pinion.reference_d == pytest.approx(first.gear.reference_d)
    assert swapped.gear.reference_d == pytest.approx(first.pinion.reference_d)


def test_constant_external_shift_sum_preserves_pair_condition_but_redistributes_teeth():
    first = compute_set(
        SpurSetParams.with_defaults(
            2.0, 17, 43, face_width=20.0, profile_shift_1=0.4, profile_shift_2=0.2
        )
    )
    redistributed = compute_set(
        SpurSetParams.with_defaults(
            2.0, 17, 43, face_width=20.0, profile_shift_1=0.1, profile_shift_2=0.5
        )
    )

    assert first.params.profile_shift_combination == pytest.approx(0.6)
    assert redistributed.params.profile_shift_combination == pytest.approx(0.6)
    assert redistributed.working_centre_distance == pytest.approx(
        first.working_centre_distance, abs=1e-11
    )
    assert redistributed.working_pressure_angle == pytest.approx(
        first.working_pressure_angle, abs=1e-11
    )
    assert sum(
        member.geometric_tooth_thickness for member in (first.pinion, first.gear)
    ) == pytest.approx(
        sum(
            member.geometric_tooth_thickness
            for member in (redistributed.pinion, redistributed.gear)
        ),
        abs=1e-11,
    )
    assert redistributed.pinion.reference_tooth_thickness != pytest.approx(
        first.pinion.reference_tooth_thickness, abs=1e-11
    )


def test_zero_shift_is_exactly_the_standard_reference_case():
    default = compute_set(SpurSetParams.with_defaults(2.0, 17, 43))
    explicit = compute_set(
        SpurSetParams.with_defaults(
            2.0,
            17,
            43,
            profile_shift_1=0.0,
            profile_shift_2=0.0,
            working_centre_distance=None,
            root_geometry="legacy",
        )
    )
    assert explicit == default
    assert explicit.pinion.reference_r == pytest.approx(17.0, abs=1e-12)
    assert explicit.gear.reference_r == pytest.approx(43.0, abs=1e-12)
    assert explicit.working_centre_distance == pytest.approx(60.0, abs=1e-12)


def test_beta_tending_to_zero_converges_to_the_straight_equations():
    straight = compute_set(
        SpurSetParams.with_defaults(
            2.0,
            17,
            43,
            face_width=20.0,
            profile_shift_1=0.4,
            profile_shift_2=0.2,
        )
    )
    nearly_straight = compute_set(
        SpurSetParams.with_defaults(
            2.0,
            17,
            43,
            face_width=20.0,
            helix_angle=1e-6,
            profile_shift_1=0.4,
            profile_shift_2=0.2,
        )
    )

    for field in (
        "transverse_module",
        "reference_centre_distance",
        "working_centre_distance",
        "circular_pitch",
        "whole_depth",
        "transverse_contact_ratio",
    ):
        assert getattr(nearly_straight, field) == pytest.approx(
            getattr(straight, field), abs=1e-10
        )
    assert nearly_straight.total_contact_ratio == pytest.approx(
        straight.total_contact_ratio + nearly_straight.overlap_ratio,
        abs=1e-10,
    )
    assert nearly_straight.reference_pressure_angle == pytest.approx(
        straight.reference_pressure_angle, abs=1e-12
    )
    assert nearly_straight.working_pressure_angle == pytest.approx(
        straight.working_pressure_angle, abs=1e-12
    )
    assert nearly_straight.overlap_ratio < 1e-7
    for straight_member, nearly_member in zip(
        (straight.pinion, straight.gear),
        (nearly_straight.pinion, nearly_straight.gear),
    ):
        for field in (
            "reference_r",
            "base_r",
            "working_r",
            "tip_r",
            "root_r",
            "reference_tooth_thickness",
            "psi0",
        ):
            assert getattr(nearly_member, field) == pytest.approx(
                getattr(straight_member, field), abs=1e-10
            )


HOSTILE_CASES = (
    ("near_undercut", 2.0, 17, 43, 20.0, 0.0, 0.0, 0.0),
    ("large_positive_shift", 2.0, 17, 43, 20.0, 0.8, 0.0, 0.0),
    ("large_negative_shift", 2.0, 17, 43, 20.0, -0.5, 0.0, 0.0),
    ("low_pressure_angle", 2.0, 24, 48, 14.5, 0.2, -0.1, 0.0),
    ("high_pressure_angle", 2.0, 24, 48, 25.0, 0.2, -0.1, 0.0),
    ("small_helix", 2.0, 24, 48, 20.0, 0.2, -0.1, 0.1),
    ("large_helix", 2.0, 24, 48, 20.0, 0.2, -0.1, 40.0),
)


@pytest.mark.parametrize(
    "case,module,z1,z2,alpha,x1,x2,beta", HOSTILE_CASES
)
def test_numerically_hostile_but_valid_cases_remain_finite(
    case, module, z1, z2, alpha, x1, x2, beta
):
    p = SpurSetParams.with_defaults(
        module,
        z1,
        z2,
        face_width=20.0,
        pressure_angle=alpha,
        helix_angle=beta,
        profile_shift_1=x1,
        profile_shift_2=x2,
    )
    geo = compute_set(p)
    expected = _expected_pair(module, z1, z2, alpha, beta, x1, x2)
    _assert_pair_matches_expected(geo, expected, internal=False)
    assert 0.0 < geo.working_pressure_angle < math.pi / 2.0
    assert geo.working_centre_distance > 0.0


def _angle_difference(first: float, second: float) -> float:
    return abs((first - second + math.pi) % (2.0 * math.pi) - math.pi)


@pytest.mark.parametrize(
    "internal,x1,x2,backlash",
    [
        (False, 0.3, -0.1, 0.12),
        (False, 0.4, 0.2, 0.12),
        (True, 0.3, 0.0, 0.12),
        (True, 0.3, -0.3, 0.12),
    ],
)
def test_mesh_phase_and_backlash_are_consistent_at_working_distance(
    internal, x1, x2, backlash
):
    p = SpurSetParams.with_defaults(
        2.0,
        18,
        60,
        internal=internal,
        face_width=20.0,
        profile_shift_1=x1,
        profile_shift_2=x2,
        backlash=backlash,
    )
    geo = compute_set(p)
    expected = _expected_pair(
        2.0,
        18,
        60,
        20.0,
        0.0,
        x1,
        x2,
        backlash,
        internal=internal,
    )

    signed_distance = -1.0 if internal else 1.0
    assert mesh.gear_translation(geo) == pytest.approx(
        (signed_distance * expected["a_work"], 0.0, 0.0), abs=1e-11
    )

    clock = mesh.clocking_for(geo)
    tau = 2.0 * math.pi / geo.gear.z
    target = 0.0 if internal else math.pi
    centres = [clock + (k + 0.5) * tau for k in range(geo.gear.z)]
    assert min(_angle_difference(value, target) for value in centres) < 1e-12

    actual_tooth_thickness = []
    for member in (geo.pinion, geo.gear):
        angle = _inv(geo.reference_pressure_angle)
        width = 2.0 * member.reference_r * (member.psi0 - angle)
        if member.internal:
            width = geo.circular_pitch - width
        actual_tooth_thickness.append(width)

    geometric_total = sum(
        member.geometric_tooth_thickness for member in (geo.pinion, geo.gear)
    )
    actual_total = sum(actual_tooth_thickness)
    assert geometric_total - actual_total == pytest.approx(backlash, abs=1e-11)
    for member, width in zip((geo.pinion, geo.gear), actual_tooth_thickness):
        assert member.geometric_tooth_thickness - width == pytest.approx(
            backlash / 2.0, abs=1e-11
        )


@pytest.mark.parametrize(
    "internal,beta,z1,z2,x1,x2,root_geometry",
    [
        (False, 0.0, 17, 43, 0.0, 0.0, "legacy"),
        (False, 0.0, 12, 43, 0.0, 0.0, "rack_generated"),
        (False, 15.0, 17, 43, 0.3, -0.1, "legacy"),
        (True, 0.0, 18, 60, 0.3, 0.0, "legacy"),
        (True, 15.0, 18, 60, 0.3, 0.1, "legacy"),
    ],
)
def test_profile_segments_respect_the_independent_circles_and_involute_transition(
    internal, beta, z1, z2, x1, x2, root_geometry
):
    p = SpurSetParams.with_defaults(
        2.0,
        z1,
        z2,
        internal=internal,
        face_width=20.0,
        helix_angle=beta,
        profile_shift_1=x1,
        profile_shift_2=x2,
        root_geometry=root_geometry,
    )
    geo = compute_set(p)

    for name in ("pinion", "gear"):
        member = geo.member(name)
        section = tooth_space_section(geo, name)

        member_shift = x2 if name == "gear" else x1
        reference_r = (
            p.module / math.cos(math.radians(beta)) * member.z / 2.0
        )
        alpha_t = math.atan2(
            math.tan(math.radians(p.pressure_angle)),
            math.cos(math.radians(beta)),
        )
        expected_tip = (
            reference_r - p.module * (
                1.0 - member_shift + geo.tip_alteration_coefficient
            )
            if member.internal
            else reference_r + p.module * (
                1.0 + member_shift + geo.tip_alteration_coefficient
            )
        )
        expected_root = (
            reference_r + p.module * (1.25 + member_shift)
            if member.internal
            else reference_r - p.module * (1.25 - member_shift)
        )
        expected_base = reference_r * math.cos(alpha_t)
        assert section.r_root == pytest.approx(member.root_r, abs=1e-11)
        assert member.base_r == pytest.approx(expected_base, abs=1e-11)
        assert member.tip_r == pytest.approx(expected_tip, abs=1e-11)
        assert member.root_r == pytest.approx(expected_root, abs=1e-11)
        assert section.r_tip == pytest.approx(expected_tip, abs=1e-11)
        assert all(
            math.hypot(x, y) == pytest.approx(section.r_root, abs=1e-8)
            for x, y in section.segments["root"]
        )
        assert all(
            math.hypot(x, y) == pytest.approx(section.r_cap, abs=1e-8)
            for x, y in section.segments["cap"]
        )

        flank = section.segments["flank_pos"]
        assert math.hypot(*flank[0]) == pytest.approx(section.r_tip, abs=1e-8)
        # A circular legacy fillet can trim its first/last flank point to an
        # interpolated tangent point. The remaining points are the sampled
        # analytical involute and are the ones checked against the equation.
        involute_points = (
            section.segments["flank_pos"][:-1]
            + section.segments["flank_neg"][1:]
        )
        for point in involute_points:
            radius = math.hypot(*point)
            if radius < member.base_r - 1e-8:
                continue
            roll_angle = math.acos(min(1.0, member.base_r / radius))
            observed = abs(math.atan2(point[1], point[0]))
            if member.internal:
                expected = member.psi0 - _inv(roll_angle)
            else:
                expected = member.half_pitch - member.psi0 + _inv(roll_angle)
            assert observed == pytest.approx(expected, abs=2e-8)

        if member.internal:
            assert section.r_cap < section.r_tip < section.r_root
        else:
            assert section.r_root < section.r_tip < section.r_cap

        if root_geometry == "rack_generated" and not internal and beta == 0.0:
            assert section.rack_generated
            generated = (
                section.segments["generated_root_pos"]
                + section.segments["generated_root_neg"]
            )
            assert generated
            assert all(
                math.hypot(x, y) >= member.root_r - 1e-8
                for x, y in generated
            )


def test_standard_non_undercut_active_path_matches_nominal_reference_path():
    """A standard rack-generated pair above the limit transitions smoothly."""
    p = SpurSetParams.with_defaults(2.0, 30, 43, root_geometry="rack_generated")
    geo = compute_set(p)
    independent_root_1 = _independent_rack_undercut_root_form(
        geo.pinion, p.module, p.basic_rack_root_radius_factor * p.module
    )
    independent_root_2 = _independent_rack_undercut_root_form(
        geo.gear, p.module, p.basic_rack_root_radius_factor * p.module
    )
    q1w = _independent_q(geo.pinion, geo.pinion.working_r)
    q2w = _independent_q(geo.gear, geo.gear.working_r)
    q1a = _independent_q(geo.pinion, geo.pinion.tip_form_r)
    q2a = _independent_q(geo.gear, geo.gear.tip_form_r)
    q1f = _independent_q(geo.pinion, independent_root_1)
    q2f = _independent_q(geo.gear, independent_root_2)
    expected_path = min(q1a - q1w, q2w - q2f) + min(q2a - q2w, q1w - q1f)

    assert geo.pinion.undercut is False
    assert geo.gear.undercut is False
    assert geo.contact_ratio_basis == "active_profile"
    assert geo.path_of_contact == pytest.approx(
        expected_path, abs=2e-9
    )
    assert geo.transverse_contact_ratio == pytest.approx(
        geo.path_of_contact /
        (math.pi * geo.transverse_module * math.cos(geo.reference_pressure_angle)),
        abs=1e-11,
    )
    for member in (geo.pinion, geo.gear):
        assert member.active_tip_d == pytest.approx(member.tip_form_d, abs=1e-11)


def test_shifted_non_undercut_pair_uses_actual_tip_form_limits():
    geo = compute_set(
        SpurSetParams.with_defaults(
            2.0, 20, 40, profile_shift_1=0.25, profile_shift_2=0.10
        )
    )

    expected_path = _independent_nominal_path(geo)
    assert geo.path_of_contact == pytest.approx(expected_path, abs=1e-11)
    assert geo.pinion.active_tip_d == pytest.approx(geo.pinion.tip_form_d, abs=1e-11)
    assert geo.gear.active_tip_d == pytest.approx(geo.gear.tip_form_d, abs=1e-11)
    assert all(
        member.start_active_profile_d is not None
        and member.active_tip_d is not None
        for member in (geo.pinion, geo.gear)
    )


def test_undercut_active_path_uses_independent_root_form_and_iso_94_geometry():
    """The old nominal-tip path overstates the 12-tooth rack-generated pair."""
    p = SpurSetParams.with_defaults(2.0, 12, 43, root_geometry="rack_generated")
    geo = compute_set(p)
    pinion, gear = geo.pinion, geo.gear
    rho = p.basic_rack_root_radius_factor * p.module
    independent_root_form = _independent_rack_undercut_root_form(
        pinion, p.module, rho
    )

    q1w = _independent_q(pinion, pinion.working_r)
    q2w = _independent_q(gear, gear.working_r)
    q1a = _independent_q(pinion, pinion.tip_form_r)
    q2a = _independent_q(gear, gear.tip_form_r)
    q1f = _independent_q(pinion, independent_root_form)
    # ISO 21771-1:2024 5.5.2.2 Eq. (82) / 5.5.6.2 Eq. (97): the gear active
    # tip is limited by the pinion start-of-involute when that is the smaller
    # approach/recess interval.
    active_tip_1_q = q1a
    active_tip_2_q = q1w + q2w - q1f
    expected_path = (active_tip_1_q - q1w) + (active_tip_2_q - q2w)
    old_nominal_path = (q1a - q1w) + (q2a - q2w)

    assert pinion.undercut is True
    assert geo.contact_ratio_basis == "active_profile"
    assert old_nominal_path > expected_path + 1e-6
    assert geo.path_of_contact == pytest.approx(expected_path, abs=2e-9)
    assert geo.path_of_contact < old_nominal_path
    assert geo.transverse_contact_ratio == pytest.approx(
        expected_path /
        (math.pi * geo.transverse_module * math.cos(geo.reference_pressure_angle)),
        abs=2e-11,
    )

    expected_n_f1 = independent_root_form
    expected_n_a2 = math.sqrt(gear.base_r ** 2 + active_tip_2_q ** 2)
    expected_n_f2_q = q1w + q2w - q1a
    expected_n_f2 = math.sqrt(gear.base_r ** 2 + expected_n_f2_q ** 2)
    assert pinion.start_active_profile_d == pytest.approx(2.0 * expected_n_f1, abs=2e-8)
    assert gear.active_tip_d == pytest.approx(2.0 * expected_n_a2, abs=2e-8)
    assert gear.start_active_profile_d == pytest.approx(2.0 * expected_n_f2, abs=2e-8)

    for member in (pinion, gear):
        assert member.root_form_d <= member.start_active_profile_d <= member.active_tip_d
        assert member.active_tip_d <= member.tip_form_d + 1e-10


def test_positive_profile_shift_reduces_undercut_and_increases_active_path():
    negative = compute_set(
        SpurSetParams.with_defaults(
            2.0, 12, 43, profile_shift_1=-0.3, root_geometry="rack_generated"
        )
    )
    positive = compute_set(
        SpurSetParams.with_defaults(
            2.0, 12, 43, profile_shift_1=0.3, root_geometry="rack_generated"
        )
    )

    assert negative.pinion.undercut is True
    assert positive.pinion.undercut is False
    assert positive.path_of_contact > negative.path_of_contact
    assert positive.transverse_contact_ratio > negative.transverse_contact_ratio


def test_tip_shortening_reduces_active_path_without_a_correction_factor():
    standard = compute_set(SpurSetParams.with_defaults(2.0, 20, 40))
    shortened = compute_set(
        SpurSetParams.with_defaults(
            2.0,
            20,
            40,
            tip_alteration_mode="explicit",
            tip_alteration_coefficient=-0.2,
        )
    )

    assert shortened.pinion.tip_d < standard.pinion.tip_d
    assert shortened.gear.tip_d < standard.gear.tip_d
    assert shortened.path_of_contact < standard.path_of_contact
    assert shortened.transverse_contact_ratio < standard.transverse_contact_ratio


def test_helical_contact_ratio_keeps_active_transverse_path_and_overlap_separate():
    p = SpurSetParams.with_defaults(2.0, 20, 40, helix_angle=15.0)
    geo = compute_set(p)
    expected_path = _independent_nominal_path(geo)
    expected_epsilon_beta = p.face_width * abs(math.sin(p.beta)) / (math.pi * p.module)

    assert geo.contact_ratio_basis == "approximate"
    assert geo.path_of_contact == pytest.approx(expected_path, abs=1e-11)
    assert geo.transverse_contact_ratio == pytest.approx(
        expected_path /
        (math.pi * geo.transverse_module * math.cos(geo.reference_pressure_angle)),
        abs=1e-11,
    )
    assert geo.overlap_ratio == pytest.approx(expected_epsilon_beta, abs=1e-11)
    assert geo.total_contact_ratio == pytest.approx(
        geo.transverse_contact_ratio + expected_epsilon_beta, abs=1e-11
    )


def test_internal_active_limits_use_the_internal_sign_branch():
    geo = compute_set(
        SpurSetParams.with_defaults(2.0, 18, 60, internal=True)
    )
    pinion, ring = geo.pinion, geo.gear
    expected_path = _independent_nominal_path(geo)

    assert geo.contact_ratio_basis == "approximate"
    assert geo.path_of_contact == pytest.approx(expected_path, abs=1e-11)
    assert geo.path_of_contact == pytest.approx(
        _independent_q(pinion, pinion.tip_r)
        - _independent_q(pinion, pinion.working_r)
        + _independent_q(ring, ring.working_r)
        - _independent_q(ring, ring.tip_r),
        abs=1e-11,
    )
    assert ring.active_tip_d == pytest.approx(ring.tip_form_d, abs=1e-11)
    assert pinion.start_active_profile_d < pinion.active_tip_d
    assert ring.active_tip_d < ring.start_active_profile_d < ring.root_d
