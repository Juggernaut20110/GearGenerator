"""Wiring checks on the window itself.

These build a real (never mapped) Tk window, so they need a display; on a
machine without one the whole module skips. They deliberately do not test
appearance - only that editing an input flows through parse, validate, compute
and scene build, and that the Build button follows the validation state.
"""

from __future__ import annotations

import math

import pytest

tk = pytest.importorskip("tkinter")

from bevelgear import preview                      # noqa: E402
from bevelgear.gui import App, FIELDS              # noqa: E402
from bevelgear.params import BevelSetParams        # noqa: E402


@pytest.fixture(scope="module")
def root():
    """One interpreter for the whole module.

    A Tk root per test also works, but each one re-reads the Tcl library from
    disk, and doing that sixteen times in two seconds is enough to occasionally
    lose a race with whatever else on Windows has those files open.
    """
    try:
        instance = tk.Tk()
    except tk.TclError as exc:                     # no display available
        pytest.skip(f"tkinter cannot open a display: {exc}")
    instance.withdraw()
    try:
        yield instance
    finally:
        instance.destroy()


@pytest.fixture
def app(root):
    widget = App(root)
    try:
        yield widget
    finally:
        widget.destroy()


def set_input(app: App, attr: str, text: str) -> None:
    """Type into a field and force the debounced refresh to happen now."""
    app.vars[attr].set(text)
    app.refresh()


# --- the default state -----------------------------------------------------


def test_opens_on_the_anchor_set(app):
    assert app.vars["module"].get() == "2"
    assert (app.vars["z1"].get(), app.vars["z2"].get()) == ("17", "43")

    p = app._params
    assert (p.module, p.z1, p.z2) == (2.0, 17, 43)
    assert app._geo is not None
    assert app._scene is not None
    assert app.build_button.instate(["!disabled"])


def test_readout_is_populated_from_the_geometry(app):
    rows = app.readout.get_children()
    assert len(rows) == len(preview.derived_rows(app._geo))

    labels = [app.readout.item(row, "values")[0] for row in rows]
    assert "outer cone distance Ao" in labels
    assert "mounting distance" in labels


def test_the_anchor_warning_reaches_the_messages_pane(app):
    """17:43 has a genuinely thin top land; the GUI must say so, not hide it."""
    text = app.messages.get("1.0", "end")
    assert "Warnings" in text
    assert "top land" in text


# --- editing ---------------------------------------------------------------


def test_editing_an_input_recomputes_the_geometry(app):
    set_input(app, "z2", "30")

    assert app._params.z2 == 30
    assert app._geo.gear.z == 30
    assert app._geo.pinion.pitch_angle == pytest.approx(math.atan2(1.0, 30.0 / 17.0))


def test_a_validation_error_blocks_the_build_button(app):
    set_input(app, "z1", "4")                      # below MIN_TEETH

    assert not app._validation.ok
    assert app.build_button.instate(["disabled"])
    assert "at least 6 teeth" in app.messages.get("1.0", "end")


def test_a_validation_error_still_leaves_a_preview_to_look_at(app):
    """Errors are for blocking the build, not for hiding why the set is wrong."""
    set_input(app, "z1", "4")

    assert app._geo is not None
    assert app._scene is not None


def test_unreadable_input_is_flagged_on_the_field_and_clears_the_geometry(app):
    set_input(app, "module", "two")

    assert app._geo is None
    assert app._scene is None
    assert app.entries["module"].cget("foreground") == "#b0202a"
    assert "is not a number" in app.messages.get("1.0", "end")

    set_input(app, "module", "2")
    assert app._geo is not None
    assert app.entries["module"].cget("foreground") == "black"


def test_tooth_counts_must_be_whole_numbers(app):
    set_input(app, "z1", "17.5")

    assert app._geo is None
    assert "whole number" in app.messages.get("1.0", "end")


def test_an_impossible_shaft_angle_and_ratio_is_reported_not_crashed(app):
    """Sigma = 120 with a 2.53:1 ratio is an internal bevel - rejected, not fatal."""
    set_input(app, "shaft_angle", "120")

    assert not app._validation.ok
    assert "internal or crown bevel" in app.messages.get("1.0", "end")


# --- buttons and derived actions -------------------------------------------


def test_auto_size_rewrites_only_the_blank_fields(app):
    set_input(app, "face_width", "3")
    set_input(app, "bore", "5")

    app.auto_size()

    sized = BevelSetParams.with_defaults(2.0, 17, 43)
    assert app._params.face_width == pytest.approx(sized.face_width)
    assert app._params.bore == pytest.approx(sized.bore)
    assert app._params.hub_thickness == pytest.approx(sized.hub_thickness)
    assert (app._params.module, app._params.z1) == (2.0, 17)


def test_switching_member_and_view_rebuilds_the_scene(app):
    for key, _label in preview.SCENE_LABELS:
        for member in ("pinion", "gear"):
            app.member.set(member)
            app.scene_key.set(key)
            app._on_view_change()
            assert app._scene is not None
            assert app._scene.key == key
            assert app._scene.title.startswith(member)


def test_switching_view_resets_zoom_and_pan(app):
    app._zoom, app._pan = 3.0, [40.0, -20.0]
    app._on_view_change()
    assert (app._zoom, app._pan) == (1.0, [0.0, 0.0])


def test_presets_round_trip_through_json(app, tmp_path):
    set_input(app, "bore", "9.5")
    path = tmp_path / "preset.json"
    app._params.to_json(path)

    set_input(app, "bore", "4")
    app.load_preset(path)

    assert app._params.bore == pytest.approx(9.5)
    for field in FIELDS:
        assert app.vars[field.attr].get() != ""


def test_a_bad_preset_file_does_not_take_the_window_down(app, tmp_path, monkeypatch):
    monkeypatch.setattr("bevelgear.gui.messagebox.showerror", lambda *a, **k: None)
    bad = tmp_path / "bad.json"
    bad.write_text("{not json", encoding="utf-8")

    app.load_preset(bad)

    assert app._params.z1 == 17                    # unchanged


# --- the build path, without SOLIDWORKS ------------------------------------


def test_build_report_formatting_survives_a_stub_result(app):
    """`_format_result` runs on the worker thread; it must touch only plain data."""

    class Part:
        member, teeth, body_count, face_count = "pinion", 17, 1, 214
        path = r"C:\out\pinion.sldprt"

    class Result:
        pinion = gear = Part()
        measured_shaft_angle_deg = 90.0
        shaft_angle_error_deg = 0.0
        clocking_deg = 4.1860
        interference_count = 0
        interference_volume_mm3 = 0.0
        assembly_path = r"C:\out\set.sldasm"
        mates = ("pinion apex - assembly origin", "gear mate 17:43")
        gear_ratio = (17.0, 43.0)
        articulates = True

    lines = App._format_result(Result())

    assert lines[0] == "Build finished."
    assert any("17 teeth" in line for line in lines)
    assert any("interference: none" in line for line in lines)
    assert any("gear mate 17:43" in line for line in lines)
    assert any("the set turns" in line for line in lines)


def test_a_failed_build_is_reported_in_the_messages_pane(app, monkeypatch):
    monkeypatch.setattr("bevelgear.gui.messagebox.showerror", lambda *a, **k: None)
    app._build_queue.put(("error", ["SOLIDWORKS rejected: the loft cut"]))

    app._poll_build()

    assert "loft cut" in app.messages.get("1.0", "end")
    assert app.build_button.instate(["!disabled"])  # ready to try again
