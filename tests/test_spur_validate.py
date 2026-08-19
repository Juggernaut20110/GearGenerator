"""Each spur validation rule should fire when it should and stay quiet otherwise."""

from __future__ import annotations

import dataclasses
import math

import pytest

from gears.spur.geometry import compute_set, undercut_limit
from gears.spur.params import SpurSetParams
from gears.spur.validate import MAX_HELIX_ANGLE, validate

ANCHOR = SpurSetParams.with_defaults(2.0, 17, 43)
ANCHOR_HELICAL = SpurSetParams.with_defaults(2.0, 17, 43, helix_angle=15.0)


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
