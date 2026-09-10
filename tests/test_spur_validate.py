"""Each spur validation rule should fire when it should and stay quiet otherwise."""

from __future__ import annotations

import dataclasses
import math

import pytest

from gears.spur.geometry import compute_set, undercut_limit
from gears.spur.params import SpurSetParams
from gears.validate import MIN_PRESSURE_ANGLE
from gears.spur.validate import MAX_HELIX_ANGLE, validate

ANCHOR = SpurSetParams.with_defaults(2.0, 17, 43)
ANCHOR_HELICAL = SpurSetParams.with_defaults(2.0, 17, 43, helix_angle=15.0)
ANCHOR_INTERNAL = SpurSetParams.with_defaults(2.0, 18, 60, internal=True)


def tweak(base=ANCHOR, **kw) -> SpurSetParams:
    return dataclasses.replace(base, **kw)


def fields_with_errors(p) -> set[str]:
    return {i.field for i in validate(p).errors}


def fields_with_warnings(p) -> set[str]:
    return {i.field for i in validate(p).warnings}


# --- the happy path --------------------------------------------------------


def test_anchor_case_builds_with_only_the_undercut_warning():
    """17 teeth at 20 degrees sits just under the 17.1 limit, so it warns.

    That is a real property of a 17-tooth standard pinion, not a quirk of this
    code, so the test records the warning rather than picking a tooth count that
    hides it.
    """
    result = validate(ANCHOR)
    assert result.ok
    assert fields_with_warnings(ANCHOR) == {"z1"}


def test_the_helical_anchor_is_clean():
    """A 15 degree helix lifts the transverse pressure angle past 17 teeth."""
    result = validate(ANCHOR_HELICAL)
    assert result.ok
    assert result.warnings == []


@pytest.mark.parametrize("z1,z2", [(20, 20), (25, 60), (18, 30)])
def test_ordinary_sets_pass_without_complaint(z1, z2):
    p = SpurSetParams.with_defaults(2.0, z1, z2)
    assert validate(p).ok
    assert validate(p).warnings == []


# --- basics ----------------------------------------------------------------


def test_basic_boundaries_are_inclusive_where_the_geometry_is_valid():
    assert "module" not in fields_with_errors(tweak(module=1.0))
    assert "module" in fields_with_errors(tweak(module=0.0))

    assert "z1" not in fields_with_errors(tweak(z1=6))
    assert "z1" in fields_with_errors(tweak(z1=5))

    assert "pressure_angle" not in fields_with_errors(
        tweak(pressure_angle=MIN_PRESSURE_ANGLE)
    )
    assert "pressure_angle" in fields_with_errors(
        tweak(pressure_angle=MIN_PRESSURE_ANGLE - 1e-6)
    )

    near_limit = SpurSetParams.with_defaults(
        2.0, 17, 43, helix_angle=MAX_HELIX_ANGLE - 1e-6
    )
    assert "helix_angle" not in fields_with_errors(near_limit)
    assert "helix_angle" in fields_with_errors(
        SpurSetParams.with_defaults(
            2.0, 17, 43, helix_angle=MAX_HELIX_ANGLE
        )
    )


def test_fractional_tooth_counts_are_not_rounded_into_a_different_gear():
    assert "z1" in fields_with_errors(tweak(z1=17.5))


@pytest.mark.parametrize(
    "kw,field",
    [
        ({"module": 0.0}, "module"),
        ({"module": -1.0}, "module"),
        ({"z1": 3}, "z1"),
        ({"z2": 5}, "z2"),
        ({"pressure_angle": 10.0}, "pressure_angle"),
        ({"pressure_angle": 30.0}, "pressure_angle"),
        ({"face_width": 0.0}, "face_width"),
        ({"bore": -1.0}, "bore"),
        ({"hub_thickness": -1.0}, "hub_thickness"),
        ({"backlash": -0.1}, "backlash"),
        ({"fillet_factor": -0.1}, "fillet_factor"),
        ({"basic_rack_addendum_factor": 0.0}, "basic_rack_addendum_factor"),
        ({"basic_rack_clearance_factor": -0.1}, "basic_rack_clearance_factor"),
        ({"basic_rack_root_radius_factor": -0.1}, "basic_rack_root_radius_factor"),
        ({"root_geometry": "trochoid"}, "root_geometry"),
        ({"tip_alteration_mode": "unknown"}, "tip_alteration_mode"),
        ({"hand": "sideways"}, "hand"),
        ({"helix_angle": MAX_HELIX_ANGLE}, "helix_angle"),
        ({"helix_angle": 60.0}, "helix_angle"),
    ],
)
def test_basic_rules_fire_on_their_own_field(kw, field):
    assert field in fields_with_errors(tweak(**kw))


def test_a_basic_failure_stops_before_the_geometry_is_computed():
    """compute_set would divide by zero on module = 0, so validate must not call it."""
    result = validate(tweak(module=0.0))
    assert not result.ok
    assert {i.field for i in result.errors} == {"module"}


@pytest.mark.parametrize("hand", ["right", "left"])
def test_either_hand_is_accepted(hand):
    assert "hand" not in fields_with_errors(tweak(ANCHOR_HELICAL, hand=hand))


# --- undercut --------------------------------------------------------------


def test_undercut_warning_fires_exactly_at_the_limit():
    """The rule is z < 2 cos(beta) / sin(alpha_t)^2, and nothing softer."""
    limit = undercut_limit(math.radians(20.0), 0.0)
    assert 17 < limit < 18
    assert "z1" in fields_with_warnings(tweak(z1=17))
    assert "z1" not in fields_with_warnings(tweak(z1=18, z2=43))


def test_profile_shift_moves_the_external_undercut_boundary():
    """The rounded rack criterion uses x, dedendum, and tip radius."""
    alpha = math.radians(20.0)
    z = 17
    # Rearranged from Clause 10.2 for the default rack:
    # z >= 2 * (h_fP* - x - rho_fP*(1 - sin(alpha))) / sin(alpha)^2.
    h_f = 1.25
    rho = 0.38
    effective = h_f - rho * (1.0 - math.sin(alpha))
    x_boundary = effective - z * math.sin(alpha) ** 2 / 2.0

    just_inside = tweak(profile_shift_1=x_boundary + 1e-6)
    just_outside = tweak(profile_shift_1=x_boundary - 1e-6)
    assert "z1" not in fields_with_warnings(just_inside)
    assert "z1" in fields_with_warnings(just_outside)


def test_generated_root_undercut_warning_describes_the_selected_geometry():
    p = tweak(root_geometry="rack_generated")
    messages = [issue.message for issue in validate(p).warnings]
    assert any("selected rack-generated root shows" in message for message in messages)
    assert not any("does not show the generated undercut" in message for message in messages)


def test_profile_shift_is_not_rejected_by_a_generic_minus_one_to_one_rail():
    result = validate(tweak(profile_shift_1=0.8))
    assert result.ok


def test_explicit_tip_alteration_requires_a_finite_coefficient():
    missing = tweak(tip_alteration_mode="explicit")
    assert "tip_alteration_coefficient" in fields_with_errors(missing)

    ignored = tweak(tip_alteration_coefficient=0.1)
    assert "tip_alteration_coefficient" in fields_with_errors(ignored)

    invalid = tweak(
        tip_alteration_mode="explicit", tip_alteration_coefficient=float("nan")
    )
    assert "tip_alteration_coefficient" in fields_with_errors(invalid)


def test_small_positive_tip_clearance_is_a_warning_not_an_interference_error():
    p = tweak(
        tip_alteration_mode="explicit",
        tip_alteration_coefficient=0.2499998,
    )
    result = validate(p)
    assert result.ok
    assert {issue.field for issue in result.warnings} >= {
        "tip_clearance_1", "tip_clearance_2"
    }


def test_tip_alteration_cannot_remove_the_addendum():
    p = tweak(
        tip_alteration_mode="explicit",
        tip_alteration_coefficient=-1.0,
    )
    assert "profile_shift_1" in fields_with_errors(p)


def test_internal_tip_base_boundary_is_checked_from_the_actual_radii():
    normal_module = 2.0
    z2 = 60
    alpha_t = math.radians(20.0)
    reference_r = normal_module * z2 / 2.0
    base_r = reference_r * math.cos(alpha_t)
    x_boundary = 1.0 - (reference_r - base_r) / normal_module

    just_inside = dataclasses.replace(
        ANCHOR_INTERNAL,
        profile_shift_2=x_boundary + 1e-6,
        tip_alteration_mode="legacy",
    )
    just_outside = dataclasses.replace(
        ANCHOR_INTERNAL,
        profile_shift_2=x_boundary - 1e-6,
        tip_alteration_mode="legacy",
    )
    assert "z2" not in fields_with_errors(just_inside)
    assert "z2" in fields_with_errors(just_outside)


def test_undercut_warning_names_the_member_that_undercuts():
    assert fields_with_warnings(tweak(z1=12, z2=60)) >= {"z1"}
    assert "z2" not in fields_with_warnings(tweak(z1=12, z2=60))
    assert fields_with_warnings(tweak(z1=8, z2=9)) >= {"z1", "z2"}


@pytest.mark.parametrize("beta", [15.0, 25.0, 30.0])
def test_a_helix_pulls_a_17_tooth_pinion_out_of_undercut(beta):
    assert "z1" not in fields_with_warnings(tweak(helix_angle=beta))


# --- contact ratio ---------------------------------------------------------


def test_contact_ratio_below_one_is_an_error():
    """Below 1.0 the pair stops transmitting continuous motion - not a warning.

    It takes the corner of the allowed envelope to get there: 6 teeth, the
    highest pressure angle the rails permit, and a steep helix, which between
    them push the transverse contact ratio to 0.894. Everything short of that
    keeps a tooth pair engaged, which is the point of the next test.
    """
    p = SpurSetParams.with_defaults(2.0, 6, 6, pressure_angle=25.0, helix_angle=40.0)
    geo = compute_set(p)
    assert geo.transverse_contact_ratio == pytest.approx(0.8943, abs=1e-4)
    assert "z1" in fields_with_errors(p)


def test_the_anchor_pair_keeps_more_than_one_tooth_pair_engaged():
    assert compute_set(ANCHOR).transverse_contact_ratio > 1.1


@pytest.mark.parametrize("shift_1,shift_2", [(0.2, 0.0), (-0.2, 0.0), (0.5, -0.5), (0.2, 0.3)])
def test_scoped_external_straight_profile_shift_sets_remain_valid(shift_1, shift_2):
    p = tweak(profile_shift_1=shift_1, profile_shift_2=shift_2)
    result = validate(p)
    assert result.ok
    assert compute_set(p).transverse_contact_ratio > 1.0


@pytest.mark.parametrize("beta", [15.0, 30.0])
@pytest.mark.parametrize("shift_1,shift_2", [(0.3, -0.3), (0.4, 0.2)])
def test_external_helical_profile_shift_sets_remain_valid(beta, shift_1, shift_2):
    p = SpurSetParams.with_defaults(
        2.0,
        17,
        43,
        helix_angle=beta,
        profile_shift_1=shift_1,
        profile_shift_2=shift_2,
    )
    result = validate(p)
    assert result.ok
    assert compute_set(p).transverse_contact_ratio > 1.0


def test_explicit_helical_working_distance_is_checked_against_profile_shift():
    shifted = SpurSetParams.with_defaults(
        2.0,
        17,
        43,
        helix_angle=15.0,
        profile_shift_1=0.4,
        profile_shift_2=0.2,
    )
    expected_distance = compute_set(shifted).working_centre_distance
    pinned = tweak(shifted, working_centre_distance=expected_distance)
    assert validate(pinned).ok

    conflicting = tweak(
        shifted, working_centre_distance=shifted.reference_centre_distance
    )
    assert "working_centre_distance" in fields_with_errors(conflicting)


@pytest.mark.parametrize("shift_1,shift_2", [(0.3, 0.0), (0.3, -0.3), (0.0, 0.3)])
def test_internal_profile_shift_sets_remain_valid(shift_1, shift_2):
    p = SpurSetParams.with_defaults(
        2.0,
        18,
        60,
        internal=True,
        profile_shift_1=shift_1,
        profile_shift_2=shift_2,
    )
    result = validate(p)
    assert result.ok
    assert compute_set(p).transverse_contact_ratio > 1.0


def test_explicit_internal_working_distance_is_checked_against_profile_shift():
    shifted = dataclasses.replace(
        ANCHOR_INTERNAL, profile_shift_1=0.3, profile_shift_2=0.0
    )
    expected_distance = compute_set(shifted).working_centre_distance
    pinned = dataclasses.replace(shifted, working_centre_distance=expected_distance)
    assert validate(pinned).ok

    conflicting = dataclasses.replace(
        shifted, working_centre_distance=shifted.reference_centre_distance
    )
    assert "working_centre_distance" in fields_with_errors(conflicting)


def test_shifted_ring_tip_inside_base_circle_is_rejected_exactly():
    p = SpurSetParams.with_defaults(
        2.0, 18, 60, internal=True, profile_shift_2=-0.82
    )
    geo = compute_set(p)
    assert geo.gear.tip_r < geo.gear.base_r
    errors = validate(p).errors
    assert any(issue.field == "z2" and "no involute" in issue.message for issue in errors)


# --- helix overlap ---------------------------------------------------------


def test_a_helix_too_narrow_to_overlap_warns_on_the_face_width():
    p = tweak(ANCHOR_HELICAL, face_width=8.0)
    assert 0.0 < compute_set(p).axial_contact_ratio < 1.0
    assert "face_width" in fields_with_warnings(p)


def test_straight_teeth_never_draw_the_overlap_warning():
    """Axial contact ratio is zero, not "less than one", when there is no helix."""
    for width in (5.0, 20.0, 60.0):
        p = tweak(face_width=width)
        assert compute_set(p).axial_contact_ratio == 0.0
        assert not any(
            "axial contact" in i.message for i in validate(p).warnings
        )


def test_a_full_axial_pitch_of_face_satisfies_the_overlap_rule():
    assert compute_set(ANCHOR_HELICAL).axial_contact_ratio >= 1.0
    assert "face_width" not in fields_with_warnings(ANCHOR_HELICAL)


# --- the blank -------------------------------------------------------------


def test_a_bore_past_the_root_circle_is_an_error():
    assert "bore" in fields_with_errors(tweak(bore=40.0))


def test_a_thin_wall_under_the_teeth_only_warns():
    p = tweak(bore=27.0)      # pinion root circle is 29 mm
    assert validate(p).ok
    assert "bore" in fields_with_warnings(p)


def test_bore_wall_warning_starts_just_below_the_minimum():
    root_r = compute_set(ANCHOR).pinion.root_r
    exact = tweak(bore=2.0 * (root_r - ANCHOR.module))
    just_thin = tweak(bore=2.0 * (root_r - ANCHOR.module + 1e-6))
    assert "bore" not in fields_with_warnings(exact)
    assert "bore" in fields_with_warnings(just_thin)


def test_ring_rim_warning_starts_just_below_the_minimum():
    exact = dataclasses.replace(ANCHOR_INTERNAL, rim_thickness=ANCHOR_INTERNAL.module)
    just_thin = dataclasses.replace(
        ANCHOR_INTERNAL, rim_thickness=ANCHOR_INTERNAL.module - 1e-6
    )
    assert "rim_thickness" not in fields_with_warnings(exact)
    assert "rim_thickness" in fields_with_warnings(just_thin)


def test_a_hub_thinner_than_the_teeth_warns():
    assert "hub_thickness" in fields_with_warnings(tweak(hub_thickness=1.0))


def test_no_hub_at_all_is_not_a_complaint():
    assert "hub_thickness" not in fields_with_warnings(tweak(hub_thickness=0.0))


# --- face width ------------------------------------------------------------


@pytest.mark.parametrize("width,expected", [(4.0, True), (20.0, False), (40.0, True)])
def test_face_width_is_warned_about_outside_the_usual_band(width, expected):
    p = tweak(face_width=width)
    assert ("face_width" in fields_with_warnings(p)) is expected


# --- tooth form ------------------------------------------------------------


def test_a_pointed_tooth_is_an_error():
    """Standard proportions do not point a spur tooth. Backlash does.

    Worth recording, because it is the opposite of the bevel case: there the
    Gleason long addendum nearly points a 17-tooth pinion all on its own, and the
    tip radius has to be clamped. A standard spur tooth keeps 0.58 mm of top land
    even at 6 teeth and 25 degrees. What takes it negative is asking for a
    millimetre of backlash on a 2 mm module - the tooth is simply thinned away.
    """
    p = SpurSetParams.with_defaults(2.0, 6, 6, pressure_angle=25.0, backlash=1.0)
    assert {i.field for i in validate(p).errors} >= {"z1", "z2"}


def test_a_nearly_pointed_tooth_only_warns():
    p = SpurSetParams.with_defaults(2.0, 6, 6, pressure_angle=25.0, backlash=0.5)
    result = validate(p)
    assert result.ok
    assert {"z1", "z2"} <= fields_with_warnings(p)


def test_a_large_backlash_warns_on_its_own_field_before_it_points_anything():
    """The sanity rail, fired on a set the tooth-form checks are happy with.

    17 x 43 keeps plenty of top land at 0.4 mm, so this is the warning arriving
    on the strength of the number alone - which is what it is for: 0.4 mm is
    6.4 % of the 6.28 mm circular pitch, and a real backlash is under 1 %.
    """
    result = validate(tweak(backlash=0.4))
    assert result.ok
    assert "backlash" in {w.field for w in result.warnings}
    assert "backlash" not in fields_with_warnings(tweak(backlash=0.05))


def test_actual_tooth_space_is_checked_for_a_valid_legacy_and_generated_case():
    for root_geometry in ("legacy", "rack_generated"):
        p = SpurSetParams.with_defaults(
            2.0, 12, 43, root_geometry=root_geometry
        )
        assert not any(
            "self-intersects" in issue.message for issue in validate(p).errors
        )


def test_validation_never_raises_whatever_it_is_given():
    """The contract: report everything, refuse nothing by exception."""
    for kw in (
        {"module": -5.0, "z1": 0, "z2": -3},
        {"bore": 1e6},
        {"face_width": 1e-6},
        {"helix_angle": 44.9},
        {"fillet_factor": 5.0},
        {"backlash": 100.0},
    ):
        validate(tweak(**kw))
