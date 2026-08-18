"""Each validation rule should fire when it should and stay quiet otherwise."""

from __future__ import annotations

import dataclasses
import math

import pytest

from gears.bevel.params import BevelSetParams
from gears.bevel.validate import validate

ANCHOR = BevelSetParams.with_defaults(2.0, 17, 43)


def tweak(**kw) -> BevelSetParams:
    return dataclasses.replace(ANCHOR, **kw)


def fields_with_errors(p) -> set[str]:
    return {i.field for i in validate(p).errors}


def fields_with_warnings(p) -> set[str]:
    return {i.field for i in validate(p).warnings}


# --- the happy path --------------------------------------------------------


def test_anchor_case_builds_with_only_the_thin_top_land_warning():
    """17:43 at m=2 is buildable, but its pinion is genuinely near-pointed.

    The Gleason long addendum gives the pinion a 2.78 mm addendum against an
    18.28-tooth virtual gear, leaving only 0.36 mm of top land (0.18 x module).
    That is a real property of this gear set, so the warning is correct and
    the test records it rather than hiding it.
    """
    result = validate(ANCHOR)
    assert result.ok
    assert [w.field for w in result.warnings] == ["pinion"]
    assert "top land" in result.warnings[0].message


def test_a_generous_set_is_completely_clean():
    result = validate(BevelSetParams.with_defaults(2.0, 25, 40))
    assert result.ok
    assert not result.warnings, [str(w) for w in result.warnings]


@pytest.mark.parametrize(
    "sigma,z1,z2",
    [
        (45.0, 17, 43),
        (60.0, 17, 43),
        (90.0, 17, 43),
        (120.0, 17, 30),   # ratio 1.76 < 2.00
        (150.0, 17, 19),   # ratio 1.12 < 1.15
    ],
)
def test_valid_shaft_angles_are_accepted(sigma, z1, z2):
    assert validate(BevelSetParams.with_defaults(2.0, z1, z2, shaft_angle=sigma)).ok


@pytest.mark.parametrize("sigma", [100.0, 120.0, 135.0, 150.0])
def test_obtuse_shaft_angle_limits_the_usable_ratio(sigma):
    """Above 90 degrees the *gear* cone angle is what runs out of room.

    delta2 = sigma - delta1 stays below 90 only while z2/z1 < 1/|cos(sigma)|.
    That is a real geometric limit, not an implementation shortcoming, so the
    validator must accept just inside it and reject just outside.
    """
    limit = 1.0 / abs(math.cos(math.radians(sigma)))
    z1 = 20

    inside = BevelSetParams.with_defaults(2.0, z1, int(z1 * limit * 0.9), shaft_angle=sigma)
    outside = BevelSetParams.with_defaults(2.0, z1, int(z1 * limit * 1.2), shaft_angle=sigma)

    assert validate(inside).ok, [str(e) for e in validate(inside).errors]
    result = validate(outside)
    assert not result.ok
    assert any("internal or crown" in i.message for i in result.errors)


def test_miter_pair_is_accepted():
    assert validate(BevelSetParams.with_defaults(3.0, 20, 20)).ok


# --- basic range errors ----------------------------------------------------


@pytest.mark.parametrize("value", [0.0, -1.0])
def test_non_positive_module_is_an_error(value):
    assert "module" in fields_with_errors(tweak(module=value))


@pytest.mark.parametrize("value", [0, 3, 5])
def test_too_few_teeth_is_an_error(value):
    assert "z1" in fields_with_errors(tweak(z1=value))
    assert "z2" in fields_with_errors(tweak(z2=value))


@pytest.mark.parametrize("value", [10.0, 14.0, 25.5, 30.0])
def test_pressure_angle_outside_range_is_an_error(value):
    assert "pressure_angle" in fields_with_errors(tweak(pressure_angle=value))


@pytest.mark.parametrize("value", [14.5, 20.0, 25.0])
def test_pressure_angle_inside_range_is_accepted(value):
    assert "pressure_angle" not in fields_with_errors(tweak(pressure_angle=value))


@pytest.mark.parametrize("value", [0.0, -10.0, 180.0, 200.0])
def test_shaft_angle_outside_range_is_an_error(value):
    assert "shaft_angle" in fields_with_errors(tweak(shaft_angle=value))


@pytest.mark.parametrize("value", [0.0, -5.0])
def test_non_positive_face_width_is_an_error(value):
    assert "face_width" in fields_with_errors(tweak(face_width=value))


def test_negative_bore_and_hub_are_errors():
    assert "bore" in fields_with_errors(tweak(bore=-1.0))
    assert "hub_thickness" in fields_with_errors(tweak(hub_thickness=-1.0))
    assert "min_root_thickness" in fields_with_errors(tweak(min_root_thickness=-1.0))


def test_no_root_rim_warns_when_the_heel_would_feather():
    """Only when the wedge is actually shallow, and only with no rim to fix it.

    17/43 leaves the gear at 25 degrees and the pinion at 70, so exactly one
    member should be named.
    """
    fields = {w.field for w in validate(tweak(min_root_thickness=0.0)).warnings}
    assert "min_root_thickness" in fields

    messages = [
        w.message for w in validate(tweak(min_root_thickness=0.0)).warnings
        if w.field == "min_root_thickness"
    ]
    assert len(messages) == 1 and "gear" in messages[0]

    assert "min_root_thickness" not in {
        w.field for w in validate(tweak(min_root_thickness=0.5)).warnings
    }


def test_validate_never_raises_on_nonsense():
    """The GUI calls this on every keystroke; it must not blow up."""
    for p in (
        tweak(module=0.0),
        tweak(shaft_angle=0.0),
        tweak(z1=0, z2=0),
        tweak(face_width=-3.0, bore=-2.0),
        tweak(module=-1.0, shaft_angle=400.0, pressure_angle=-5.0),
    ):
        assert not validate(p).ok


# --- geometry-dependent errors ---------------------------------------------


def test_face_width_beyond_the_cone_distance_is_an_error():
    # Ao for the anchor case is about 46.24 mm.
    assert "face_width" in fields_with_errors(tweak(face_width=60.0))


def test_bore_that_eats_the_teeth_is_an_error():
    assert "bore" in fields_with_errors(tweak(bore=40.0))


def test_internal_bevel_is_rejected():
    """An obtuse shaft angle with a reducing ratio drives delta1 past 90 deg.

    This needs z2/z1 <= -cos(sigma), so it only happens for sigma > 90.
    """
    p = BevelSetParams.with_defaults(2.0, 40, 20, shaft_angle=150.0)
    result = validate(p)
    assert not result.ok
    assert "shaft_angle" in {i.field for i in result.errors}
    assert any("internal or crown" in i.message for i in result.errors)


# --- warnings --------------------------------------------------------------


def test_excessive_face_width_warns_but_still_builds():
    p = tweak(face_width=25.0)  # over min(Ao/3, 10m) = 15.41
    result = validate(p)
    assert result.ok
    assert "face_width" in {w.field for w in result.warnings}


def test_undercut_risk_warns():
    """A low-tooth-count pinion on a shallow cone falls under z_v = 17.1."""
    p = BevelSetParams.with_defaults(2.0, 8, 9)
    assert "pinion" in fields_with_warnings(p)
    assert any("undercut" in w.message for w in validate(p).warnings)


def test_thin_hub_warns():
    p = tweak(hub_thickness=1.0)  # whole depth is 4.376
    result = validate(p)
    assert result.ok
    assert "hub_thickness" in {w.field for w in result.warnings}


def test_extreme_ratio_warns():
    p = BevelSetParams.with_defaults(2.0, 6, 90)
    assert "z2" in fields_with_warnings(p)


def test_warnings_do_not_block():
    p = tweak(hub_thickness=1.0, face_width=25.0)
    result = validate(p)
    assert result.ok
    assert len(result.warnings) >= 2
