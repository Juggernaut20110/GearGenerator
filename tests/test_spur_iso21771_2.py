"""Independent ISO 21771-2 backlash checks for parallel-axis spur pairs.

The expected working tooth widths in this file are deliberately calculated
from the member's working radius, base radius, psi0, and the involute
function.  They do not call the production backlash properties, so a change
that merely makes a reported property equal its request cannot pass these
tests.
"""

from __future__ import annotations

import math

import pytest

from gears.involute import inv
from gears.spur.geometry import compute_set, tooth_space_section
from gears.spur.params import SpurSetParams
from gears.spur.preview import build_scene, dxf_lines
from gears.spur.validate import validate


def _independent_working_tooth_width(member) -> float:
    alpha_r = math.acos(member.base_r / member.working_r)
    involute_width = member.psi0 - inv(alpha_r)
    if member.internal:
        return 2.0 * member.working_r * (
            member.half_pitch - involute_width
        )
    return 2.0 * member.working_r * involute_width


def _independent_working_backlash(geo) -> float:
    working_pitch = 2.0 * math.pi * geo.pinion.working_r / geo.pinion.z
    return working_pitch - sum(
        _independent_working_tooth_width(member)
        for member in (geo.pinion, geo.gear)
    )


@pytest.mark.parametrize(
    "kwargs",
    [
        {},
        {"profile_shift_1": 0.35, "profile_shift_2": -0.10},
        {"helix_angle": 15.0, "profile_shift_1": 0.25, "profile_shift_2": 0.15},
        {"internal": True},
        {"internal": True, "helix_angle": 15.0, "profile_shift_1": 0.25},
    ],
)
def test_working_circumferential_backlash_closes_independent_working_spaces(kwargs):
    p = SpurSetParams.with_defaults(
        2.0,
        21 if kwargs.get("internal") else 18,
        60 if kwargs.get("internal") else 43,
        backlash=0.10,
        backlash_mode="working_circumferential",
        **kwargs,
    )
    geo = compute_set(p)

    expected = _independent_working_backlash(geo)
    assert expected == pytest.approx(0.10, abs=1e-11)
    assert geo.j_wt == pytest.approx(expected, abs=1e-11)
    assert validate(p).ok


def test_shifted_pair_distinguishes_reference_and_working_backlash():
    common = dict(
        module=2.0,
        z1=17,
        z2=43,
        profile_shift_1=0.4,
        profile_shift_2=0.2,
        backlash=0.10,
    )
    legacy = compute_set(SpurSetParams.with_defaults(**common))
    working = compute_set(
        SpurSetParams.with_defaults(
            **common, backlash_mode="working_circumferential"
        )
    )

    # The old behavior removes the requested amount from reference tooth
    # thickness, but profile shift also changes the natural tooth-width sum.
    # It must not silently be presented as a working-circle request once a_w
    # differs from a.
    legacy_reduction = sum(
        member.geometric_tooth_thickness - member.reference_tooth_thickness
        for member in (legacy.pinion, legacy.gear)
    )
    assert legacy_reduction == pytest.approx(0.10, abs=1e-11)
    assert legacy.j_wt != pytest.approx(0.10, abs=1e-7)
    assert working.j_wt == pytest.approx(0.10, abs=1e-11)
    assert working.j_t != pytest.approx(0.10, abs=1e-7)


def test_working_backlash_allocation_is_explicit_and_not_an_iso_requirement():
    p = SpurSetParams.with_defaults(
        2.0,
        17,
        43,
        profile_shift_1=0.35,
        profile_shift_2=0.15,
        backlash=0.12,
        backlash_mode="working_circumferential",
        backlash_allocation=0.25,
    )
    geo = compute_set(p)

    assert geo.j_wt == pytest.approx(0.12, abs=1e-11)
    assert geo.pinion.working_tooth_thickness_allowance == pytest.approx(
        0.03, abs=1e-11
    )
    assert geo.gear.working_tooth_thickness_allowance == pytest.approx(
        0.09, abs=1e-11
    )
    assert geo.pinion.reference_tooth_thickness_allowance == pytest.approx(
        0.03 * geo.pinion.reference_r / geo.pinion.working_r,
        abs=1e-11,
    )


def test_normal_base_backlash_mode_converts_through_working_geometry():
    p = SpurSetParams.with_defaults(
        2.0,
        17,
        43,
        helix_angle=15.0,
        profile_shift_1=0.3,
        profile_shift_2=0.2,
        backlash=0.08,
        backlash_mode="normal",
    )
    geo = compute_set(p)

    assert _independent_working_backlash(geo) == pytest.approx(
        geo.j_wt, abs=1e-11
    )
    assert geo.j_bn == pytest.approx(0.08, abs=1e-11)
    assert geo.j_bt == pytest.approx(
        geo.j_wt * math.cos(geo.working_pressure_angle), abs=1e-11
    )
    assert geo.j_wn == pytest.approx(
        geo.j_wt * math.cos(geo.working_helix_angle), abs=1e-11
    )
    assert validate(p).ok


@pytest.mark.parametrize(
    "kwargs,field",
    [
        ({"backlash_mode": "not-a-mode"}, "backlash_mode"),
        ({"backlash_allocation": -0.1}, "backlash_allocation"),
        ({"backlash_allocation": 1.1}, "backlash_allocation"),
        ({"backlash": -0.01}, "backlash"),
    ],
)
def test_backlash_input_validation_rejects_ambiguous_or_impossible_values(
    kwargs, field
):
    p = SpurSetParams.with_defaults(2.0, 17, 43, **kwargs)
    result = validate(p)
    assert field in {issue.field for issue in result.errors}


def test_zero_backlash_preserves_working_space_closure_for_internal_pair():
    p = SpurSetParams.with_defaults(
        2.0,
        18,
        60,
        internal=True,
        profile_shift_1=0.25,
        profile_shift_2=0.25,
        backlash_mode="working_circumferential",
        backlash=0.0,
    )
    geo = compute_set(p)
    assert _independent_working_backlash(geo) == pytest.approx(0.0, abs=1e-11)
    assert geo.gear.working_tooth_thickness > 0.0
    assert validate(p).ok


@pytest.mark.parametrize("internal", [False, True])
def test_working_mode_keeps_named_cad_segments_and_dxf_topology(internal):
    p = SpurSetParams.with_defaults(
        2.0,
        18,
        60 if internal else 43,
        internal=internal,
        profile_shift_1=0.2,
        profile_shift_2=0.2,
        backlash=0.1,
        backlash_mode="working_circumferential",
    )
    geo = compute_set(p)
    section = tooth_space_section(geo, "pinion")

    assert len(section.loop_2d) > 10
    assert section.loop_2d[0] != section.loop_2d[-1]
    assert {
        "fillet_neg", "flank_neg", "riser_neg", "riser_pos",
        "flank_pos", "fillet_pos", "root",
    } <= set(section.segments)
    scene = build_scene(geo, "pinion", "transverse")
    assert dxf_lines(scene)
