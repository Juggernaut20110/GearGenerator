"""The spiral bevel inputs, from the CLI flag and the GUI field down to geometry.

Separate from `test_spiral_geometry.py` on purpose: that file asks whether the
trace is right, this one asks whether a person can reach it. The two fail for
completely different reasons and it is worth being told which.
"""

from __future__ import annotations

import math

import pytest

from gears.__main__ import main
from gears.bevel.geometry import compute_set, phase_at_cone_distance
from gears.bevel.params import BevelSetParams
from gears.bevel.report import params_from_args
from gears.bevel import preview

ANCHOR = ["--module", "2", "--z1", "17", "--z2", "43"]


# --- the CLI ---------------------------------------------------------------


def test_the_spiral_flags_reach_the_parameters():
    assert main(ANCHOR + ["--spiral", "35"]) == 0
    assert main(ANCHOR + ["--spiral", "35", "--cutter-radius", "45"]) == 0
    assert main(ANCHOR + ["--spiral", "35", "--hand", "left"]) == 0


def test_the_report_prints_a_spiral_block_only_when_there_is_a_spiral(capsys):
    main(ANCHOR)
    assert "SPIRAL" not in capsys.readouterr().out

    main(ANCHOR + ["--spiral", "35"])
    out = capsys.readouterr().out
    assert "SPIRAL" in out
    assert "spiral angle at toe" in out
    assert "contact ratio, face" in out


def _value(out: str, label: str) -> float:
    """Pull one number out of a report row, without depending on its column.

    Asserting the whole formatted line instead ties these tests to
    `report_format.row`'s widths, which are a presentation choice that has
    nothing to do with whether the spiral reached the geometry.
    """
    for line in out.splitlines():
        if line.strip().startswith(label):
            return float(line[len(label) + 2:].split()[0])
    raise AssertionError(f"no row labelled {label!r} in the report")


def test_a_zerol_set_is_reachable_from_the_command_line(capsys):
    """Zero spiral angle with a cutter radius. The trace curves; the mean is radial."""
    main(ANCHOR + ["--spiral", "0", "--cutter-radius", "39.3"])
    out = capsys.readouterr().out
    assert "SPIRAL" in out
    assert _value(out, "spiral angle at mean") == 0.0
    # Curved all the same, which is the whole distinction Zerol turns on.
    assert _value(out, "spiral angle at toe") > 0.0
    assert _value(out, "spiral angle at heel") > 0.0


def test_the_defaults_leave_a_straight_bevel_set_straight(capsys):
    main(ANCHOR)
    out = capsys.readouterr().out
    assert _value(out, "spiral angle (mean)") == 0.0
    assert "cutter radius" not in out


# --- the parameter plumbing ------------------------------------------------


class _Args:
    """The attribute bag `params_from_args` reads. Only the flags it touches."""

    module, z1, z2 = 2.0, 17, 43
    alpha, sigma = 20.0, 90.0
    spiral, hand = 35.0, "right"
    cutter_radius = None
    face_width = bore = hub = min_root = None


def test_params_from_args_carries_the_spiral_through():
    p = params_from_args(_Args())
    assert p.spiral_angle == 35.0
    assert p.hand == "right"
    assert p.is_curved
    # Defaulted to Am, which is what makes the arc's curvature match the gear.
    assert p.cutter_radius == pytest.approx(39.3035, abs=1e-3)


def test_an_explicit_cutter_radius_beats_the_default():
    args = _Args()
    args.cutter_radius = 60.0
    assert params_from_args(args).cutter_radius == 60.0


def test_a_spiral_set_is_sized_to_the_tighter_face_limit():
    """0.30*Ao rather than Ao/3, which is 13.87 mm against 15.41 on the anchor."""
    straight = BevelSetParams.with_defaults(2.0, 17, 43)
    spiral = BevelSetParams.with_defaults(2.0, 17, 43, spiral_angle=35.0)
    assert straight.face_width == pytest.approx(15.41, abs=0.01)
    assert spiral.face_width == pytest.approx(13.87, abs=0.01)
    assert spiral.face_width < straight.face_width


def test_a_zerol_set_is_sized_as_a_curved_one():
    """The bug this guards: `with_defaults` and `is_curved` must agree.

    They asked the question differently once - one looked at the spiral angle
    alone and the other also at the cutter radius - so a Zerol set was sized as
    straight and then warned about by the validator that sized it as curved.
    """
    zerol = BevelSetParams.with_defaults(2.0, 17, 43, cutter_radius=39.3)
    assert zerol.is_curved
    assert zerol.face_width == pytest.approx(13.87, abs=0.01)

    from gears.bevel.validate import validate
    assert not any(
        issue.field == "face_width" for issue in validate(zerol).warnings
    )


def test_a_preset_written_before_the_spiral_existed_still_loads(tmp_path):
    """`JsonParams.from_json` drops unknown keys and defaults missing ones.

    A file saved by the straight-only version has no spiral_angle, hand or
    cutter_radius in it, and has to come back as the straight set it was.
    """
    import json

    path = tmp_path / "old_preset.json"
    path.write_text(
        json.dumps(
            {
                "module": 2.0, "z1": 17, "z2": 43, "face_width": 15.41,
                "bore": 8.5, "hub_thickness": 5.0, "min_root_thickness": 0.5,
                "pressure_angle": 20.0, "shaft_angle": 90.0,
            }
        ),
        encoding="utf-8",
    )
    p = BevelSetParams.from_json(path)
    assert p.spiral_angle == 0.0
    assert p.cutter_radius is None
    assert not p.is_curved


# --- the preview -----------------------------------------------------------


def test_the_trace_scene_exists_for_both_straight_and_spiral_sets():
    """It has something honest to say either way, so it is never unavailable."""
    for kwargs in ({}, {"spiral_angle": 35.0}):
        geo = compute_set(BevelSetParams.with_defaults(2.0, 17, 43, **kwargs))
        scene = preview.build_scene(geo, "pinion", "trace")
        assert scene.key == "trace"
        assert scene.polylines


def test_the_straight_trace_scene_says_the_trace_is_the_generator():
    geo = compute_set(BevelSetParams.with_defaults(2.0, 17, 43))
    assert "straight teeth" in preview.build_scene(geo, "pinion", "trace").title


def test_the_readout_grows_a_spiral_block_only_when_there_is_one():
    labels = lambda geo: [row.label for row in preview.derived_rows(geo)]  # noqa: E731

    straight = compute_set(BevelSetParams.with_defaults(2.0, 17, 43))
    assert "SPIRAL" not in labels(straight)
    assert "sweep over the face" not in labels(straight)

    spiral = compute_set(
        BevelSetParams.with_defaults(2.0, 17, 43, spiral_angle=35.0)
    )
    assert "SPIRAL" in labels(spiral)
    assert "cutter radius" in labels(spiral)
    assert "sweep over the face" in labels(spiral)


def test_the_axial_scene_reports_the_sweep_for_a_spiral_set():
    """The angle between the two end sections is what that view is for."""
    spiral = compute_set(
        BevelSetParams.with_defaults(2.0, 17, 43, spiral_angle=35.0)
    )
    title = preview.build_scene(spiral, "pinion", "axial").title
    assert "spiral sweep" in title

    straight = compute_set(BevelSetParams.with_defaults(2.0, 17, 43))
    assert "spiral sweep" not in preview.build_scene(
        straight, "pinion", "axial"
    ).title


def test_the_two_end_sections_are_drawn_apart_by_the_sweep():
    """The property behind the axial scene, checked on the geometry it draws.

    For a straight set the two ends coincide exactly; for a spiral one they sit
    the sweep apart. Drawing them on top of each other for a spiral gear would
    be the silent failure - the view would look like a straight gear's.
    """
    for kwargs, expected in (({}, 0.0), ({"spiral_angle": 35.0}, None)):
        geo = compute_set(BevelSetParams.with_defaults(2.0, 17, 43, **kwargs))
        sweep = phase_at_cone_distance(
            geo, "pinion", geo.outer_cone_dist
        ) - phase_at_cone_distance(geo, "pinion", geo.inner_cone_dist)
        if expected == 0.0:
            assert sweep == 0.0
        else:
            assert abs(math.degrees(sweep)) == pytest.approx(38.790, abs=1e-3)
