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

from gears.bevel import preview                      # noqa: E402
from gears.spur import preview as spur_preview     # noqa: E402
from gears.spur.params import SpurSetParams        # noqa: E402
from gears.gui import (                            # noqa: E402
    BEVEL_FIELDS as FIELDS,
    App,
    KINDS,
    _bevel_result_lines,
    _spur_result_lines,
)
from gears.bevel.params import BevelSetParams        # noqa: E402


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


# The one scene that is not a view of a single member. A bevel pair's tooth
# trace is one arc in the shared crown plane - both members map that same curve
# onto their own cones, which is the meshing condition - so titling it after
# whichever member happens to be selected would state something untrue.
SHARED_SCENES = {"trace"}


@pytest.mark.parametrize("kind_key", ["bevel", "spur"])
def test_every_field_round_trips_through_format_and_parse(kind_key):
    """`format` writes the widget, `parse` reads it back - they must agree.

    They came apart once: a bool field formatted as "True", which is not one of
    its combobox choices, so parsing it back raised and every input in the
    window came out as None. Cheap to assert, and it catches the whole class.
    """
    kind = KINDS[kind_key]
    params = kind.params_cls.with_defaults(2.0, 17, 43)
    for field in kind.fields:
        value = getattr(params, field.attr)
        text = field.format(value)
        if field.choices:
            assert text in field.choices
        assert field.parse(text) == pytest.approx(value) if isinstance(
            value, float
        ) else field.parse(text) == value


def test_the_arrangement_field_reads_back_as_a_bool(app):
    """The spur type's one non-numeric, non-string input."""
    app.kind_key.set("spur")
    app.on_kind_change()

    set_input(app, "internal", "internal")
    assert app._params.internal is True
    assert app._geo.centre_distance == pytest.approx(
        app._params.transverse_module * (app._params.z2 - app._params.z1) / 2.0
    )

    set_input(app, "internal", "external")
    assert app._params.internal is False


def test_switching_member_and_view_rebuilds_the_scene(app):
    for key, _label in preview.SCENE_LABELS:
        for member in ("pinion", "gear"):
            app.member.set(member)
            app.scene_key.set(key)
            app._on_view_change()
            assert app._scene is not None
            assert app._scene.key == key
            if key in SHARED_SCENES:
                assert app._scene.title.startswith("both members")
            else:
                assert app._scene.title.startswith(member)


def test_a_shared_scene_draws_the_same_thing_for_either_member(app):
    """The claim `SHARED_SCENES` makes, checked rather than asserted in a comment."""
    drawn = {}
    for member in ("pinion", "gear"):
        app.member.set(member)
        app.scene_key.set("trace")
        app._on_view_change()
        drawn[member] = [list(line.points) for line in app._scene.polylines]
    assert drawn["pinion"] == drawn["gear"]


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
    monkeypatch.setattr("gears.gui.messagebox.showerror", lambda *a, **k: None)
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

    lines = App._format_result(Result(), _bevel_result_lines)

    assert lines[0] == "Build finished."
    assert any("17 teeth" in line for line in lines)
    assert any("interference: none" in line for line in lines)
    assert any("gear mate 17:43" in line for line in lines)
    assert any("the set turns" in line for line in lines)


def test_a_failed_build_is_reported_in_the_messages_pane(app, monkeypatch):
    monkeypatch.setattr("gears.gui.messagebox.showerror", lambda *a, **k: None)
    app._build_queue.put(("error", ["SOLIDWORKS rejected: the loft cut"]))

    app._poll_build()

    assert "loft cut" in app.messages.get("1.0", "end")
    assert app.build_button.instate(["!disabled"])  # ready to try again


# --- switching gear type ---------------------------------------------------


def test_bevel_is_what_the_window_opens_on(app):
    assert app.kind_key.get() == "bevel"
    assert app.kind.key == "bevel"
    assert isinstance(app._params, BevelSetParams)


def visible(app):
    """The field rows currently gridded, in order."""
    return [
        attr for attr, row in app.field_rows.items() if row[1].winfo_manager()
    ]


@pytest.mark.parametrize("key", ["bevel", "spur"])
def test_only_the_active_type_s_rows_are_shown(app, key):
    app.kind_key.set(key)
    app.on_kind_change()
    assert visible(app) == [f.attr for f in KINDS[key].fields]


@pytest.mark.parametrize("key", ["bevel", "spur"])
def test_only_the_active_type_s_scenes_are_offered(app, key):
    app.kind_key.set(key)
    app.on_kind_change()
    shown = [k for k, b in app.scene_buttons.items() if b.winfo_manager()]
    assert set(shown) == {k for k, _ in KINDS[key].preview.SCENE_LABELS}
    assert app.scene_key.get() in shown


def test_switching_type_keeps_the_module_and_tooth_counts(app):
    """The three inputs both types share should survive the switch.

    They cannot be carried by re-parsing the widgets: the fields only the new
    type has are still empty at that point, so the parse fails and a naive
    implementation falls back to a stub.
    """
    set_input(app, "z2", "31")
    app.kind_key.set("spur")
    app.on_kind_change()

    assert isinstance(app._params, SpurSetParams)
    assert (app._params.module, app._params.z1, app._params.z2) == (2.0, 17, 31)


def test_switching_type_re_sizes_the_blank(app):
    """A face width that suited a bevel set is not the one a spur set wants."""
    bevel_width = app._params.face_width
    app.kind_key.set("spur")
    app.on_kind_change()
    assert app._params.face_width == pytest.approx(
        SpurSetParams.with_defaults(2.0, 17, 43).face_width
    )
    assert app._params.face_width != pytest.approx(bevel_width)


def test_switching_back_and_forth_stays_valid(app):
    for key in ("spur", "bevel", "spur", "bevel"):
        app.kind_key.set(key)
        app.on_kind_change()
        assert app._geo is not None
        assert app._scene is not None
        assert app._validation.ok
        assert isinstance(app._params, KINDS[key].params_cls)


def test_a_spur_set_computes_and_draws(app):
    app.kind_key.set("spur")
    app.on_kind_change()
    assert app._geo.centre_distance == pytest.approx(60.0)
    assert app._scene.key == "transverse"
    assert "centre distance" in [r.label for r in spur_preview.derived_rows(app._geo)]
    assert app.build_button.instate(["!disabled"])


def test_the_helix_angle_and_hand_reach_the_geometry(app):
    app.kind_key.set("spur")
    app.on_kind_change()
    set_input(app, "helix_angle", "15")
    app.vars["hand"].set("left")
    app.refresh()

    assert app._params.helix_angle == pytest.approx(15.0)
    assert app._geo.pinion.hand == "left"
    assert app._geo.gear.hand == "right"
    assert app._geo.axial_contact_ratio > 0.0


def test_the_hand_combobox_refuses_anything_else(app):
    app.kind_key.set("spur")
    app.on_kind_change()
    app.vars["hand"].set("sideways")
    app.refresh()
    assert app._geo is None
    assert "must be one of right, left" in app.messages.get("1.0", "end")


def test_auto_size_uses_the_active_type_s_rules(app):
    app.kind_key.set("spur")
    app.on_kind_change()
    set_input(app, "helix_angle", "15")
    set_input(app, "face_width", "8")
    app.auto_size()

    # A helical set is sized to at least one axial pitch, which 8 mm is not.
    sized = SpurSetParams.with_defaults(2.0, 17, 43, helix_angle=15.0)
    assert app._params.face_width == pytest.approx(sized.face_width)
    assert app._params.helix_angle == pytest.approx(15.0)


def test_a_spur_status_line_names_spur_quantities(app):
    app.kind_key.set("spur")
    app.on_kind_change()
    status = app._status_text()
    assert "a 60.000 mm" in status
    assert "cones" not in status


def test_the_spur_build_report_survives_a_stub_result():
    class Part:
        member, teeth, body_count, face_count = "pinion", 17, 1, 90
        path = r"C:\out\pinion.sldprt"

    class Result:
        pinion = gear = Part()
        measured_centre_distance_mm = 60.0
        centre_distance_error_mm = 0.0
        measured_axis_angle_deg = 0.0
        clocking_deg = 0.0
        interference_count = 0
        interference_volume_mm3 = 0.0
        assembly_path = r"C:\out\spur.sldasm"
        mates = ("pinion origin - assembly origin", "gear mate 17:43")
        gear_ratio = (17.0, 43.0)
        articulates = True

    lines = App._format_result(Result(), _spur_result_lines)
    text = "\n".join(lines)
    assert "centre distance 60.0000 mm measured" in text
    assert "parallel is 0" in text
    assert "the set turns" in text
