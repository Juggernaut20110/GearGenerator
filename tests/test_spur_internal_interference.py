"""Independent checks for ISO internal running-pair interference.

The expected margins in this module are deliberately written here instead of
calling the validator's private helpers.  The tests therefore detect a
regression back to the old fixed tooth-count rule or to a circle-overlap test.
"""

from __future__ import annotations

import dataclasses
import math

from gears.spur.geometry import compute_set
from gears.spur.params import SpurSetParams
from gears.spur.validate import validate


def _inv(angle: float) -> float:
    return math.tan(angle) - angle


def _active_tip_radius(member) -> float:
    return member.active_tip_r if member.active_tip_r is not None else member.tip_r


def _independent_ca_ct1_margin(geo) -> float:
    """Positive means CA < CT1 for the positive-radius internal convention."""
    ring_tip = _active_tip_radius(geo.gear)
    if ring_tip < geo.gear.base_r:
        return -math.inf
    alpha_a2 = math.acos(geo.gear.base_r / ring_tip)
    return geo.pinion.z / geo.gear.z - (
        1.0 - math.tan(alpha_a2) / math.tan(geo.working_pressure_angle)
    )


def _independent_tip_to_tip_margin(geo) -> float:
    """Recompute ISO 21771-1:2024 5.5.8.3's rotation inequality."""
    r_a1 = _active_tip_radius(geo.pinion)
    r_a2 = _active_tip_radius(geo.gear)
    a_w = geo.working_centre_distance
    cosine_aa1 = (r_a2**2 - r_a1**2 - a_w**2) / (2.0 * a_w * r_a1)
    assert -1.0 <= cosine_aa1 <= 1.0
    theta_aa1 = math.acos(cosine_aa1)
    theta_aa2 = math.asin(r_a1 / r_a2 * math.sin(theta_aa1))
    alpha_at1 = math.acos(geo.pinion.base_r / r_a1)
    alpha_at2 = math.acos(geo.gear.base_r / r_a2)
    theta_a1 = _inv(alpha_at1) - _inv(geo.working_pressure_angle)
    theta_a2 = _inv(geo.working_pressure_angle) - _inv(alpha_at2)
    omega_1 = theta_aa1 + theta_a1
    omega_2 = theta_aa2 - theta_a2
    return geo.pinion.z * omega_1 - geo.gear.z * omega_2


def _internal(z1: int, z2: int, **overrides) -> SpurSetParams:
    return SpurSetParams.with_defaults(2.0, z1, z2, internal=True, **overrides)


def _error_fields(params: SpurSetParams) -> set[str]:
    return {issue.field for issue in validate(params).errors}


def test_standard_internal_pair_above_ca_ct1_boundary_is_valid():
    params = _internal(21, 60)
    geo = compute_set(params)

    assert _independent_ca_ct1_margin(geo) > 0.0
    assert _independent_tip_to_tip_margin(geo) > 0.0
    result = validate(params)
    assert result.ok
    assert not {issue.field for issue in result.errors}
    assert any("d_Ff" in issue.message for issue in result.warnings)


def test_standard_twenty_degree_pair_below_running_boundary_is_rejected():
    params = _internal(20, 60)
    geo = compute_set(params)

    assert _independent_ca_ct1_margin(geo) < 0.0
    assert "internal_tip_to_dedendum" in _error_fields(params)


def test_standard_twelve_tooth_pinion_records_undercut_and_interference():
    params = _internal(12, 60)
    geo = compute_set(params)
    result = validate(params)

    assert _independent_ca_ct1_margin(geo) < 0.0
    assert any(issue.field == "z1" and "undercut" in issue.message for issue in result.warnings)
    assert "internal_tip_to_dedendum" in _error_fields(params)


def test_theoretical_twenty_degree_undercut_boundary_is_independent_of_pair_rule():
    alpha_t = math.radians(20.0)
    theoretical_limit = 2.0 / math.sin(alpha_t) ** 2
    assert 17.0 < theoretical_limit < 18.0

    below = validate(_internal(17, 60))
    above = validate(_internal(18, 60))
    assert any(issue.field == "z1" and "undercut" in issue.message for issue in below.warnings)
    assert not any(issue.field == "z1" and "undercut" in issue.message for issue in above.warnings)


def test_positive_pinion_profile_shift_reduces_running_interference():
    negative = _internal(20, 60, profile_shift_1=-0.25)
    zero = _internal(20, 60)
    positive = _internal(20, 60, profile_shift_1=0.25)
    margins = [
        _independent_ca_ct1_margin(compute_set(params))
        for params in (negative, zero, positive)
    ]

    assert margins[0] < margins[1] < margins[2]
    assert "internal_tip_to_dedendum" in _error_fields(negative)
    assert "internal_tip_to_dedendum" in _error_fields(zero)
    assert validate(positive).ok


def test_negative_pinion_profile_shift_increases_undercut_warning():
    negative = _internal(18, 60, profile_shift_1=-0.25)
    positive = _internal(18, 60, profile_shift_1=0.25)
    negative_result = validate(negative)
    positive_result = validate(positive)

    assert any(issue.field == "z1" and "undercut" in issue.message for issue in negative_result.warnings)
    assert not any(issue.field == "z1" and "undercut" in issue.message for issue in positive_result.warnings)


def test_tip_to_tip_uses_rotation_not_circle_overlap():
    safe = _internal(30, 39)
    interfering = _internal(30, 38)
    safe_geo = compute_set(safe)
    interfering_geo = compute_set(interfering)

    assert _independent_ca_ct1_margin(safe_geo) > 0.0
    assert _independent_ca_ct1_margin(interfering_geo) > 0.0
    assert _independent_tip_to_tip_margin(safe_geo) > 0.0
    assert _independent_tip_to_tip_margin(interfering_geo) < 0.0
    assert validate(safe).ok
    assert _error_fields(interfering) == {"internal_tip_to_tip"}


def test_tip_alteration_changes_both_working_interference_limits():
    safe = _internal(30, 40, tip_alteration_mode="explicit", tip_alteration_coefficient=0.1)
    interfering = _internal(30, 40, tip_alteration_mode="explicit", tip_alteration_coefficient=0.2)

    safe_geo = compute_set(safe)
    interfering_geo = compute_set(interfering)
    assert _independent_ca_ct1_margin(safe_geo) > 0.0
    assert _independent_tip_to_tip_margin(safe_geo) > 0.0
    assert _independent_ca_ct1_margin(interfering_geo) < 0.0
    assert _independent_tip_to_tip_margin(interfering_geo) < 0.0
    assert validate(safe).ok
    assert {"internal_tip_to_dedendum", "internal_tip_to_tip"} <= _error_fields(interfering)


def test_delta_z_below_ten_is_accepted_when_geometry_is_safe():
    params = _internal(26, 34, profile_shift_2=0.25)
    geo = compute_set(params)
    result = validate(params)

    assert params.z2 - params.z1 < 10
    assert _independent_ca_ct1_margin(geo) > 0.0
    assert _independent_tip_to_tip_margin(geo) > 0.0
    assert result.ok
    assert any(issue.field == "z2" and "non-normative" in issue.message for issue in result.warnings)


def test_internal_helical_pair_uses_transverse_running_geometry():
    params = _internal(24, 60, helix_angle=15.0)
    geo = compute_set(params)
    result = validate(params)

    assert _independent_ca_ct1_margin(geo) > 0.0
    assert _independent_tip_to_tip_margin(geo) > 0.0
    assert result.ok
    assert not any(issue.field == "internal_tip_to_tip" for issue in result.errors)
    assert any("transverse section" in issue.message for issue in result.warnings)


def test_nominal_root_diameter_is_not_used_as_internal_root_form_diameter():
    params = _internal(21, 60)
    geo = compute_set(params)
    result = validate(params)

    assert geo.gear.root_form_d is None
    assert any("nominal d_f" in issue.message and "d_Ff" in issue.message for issue in result.warnings)


def test_old_fixed_difference_rule_is_not_the_rejection_reason():
    params = dataclasses.replace(_internal(30, 40), profile_shift_1=-0.1)
    result = validate(params)

    assert params.z2 - params.z1 >= 10
    assert result.ok
