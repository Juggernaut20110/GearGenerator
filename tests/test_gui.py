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
    _hypoid_result_lines,
    _planetary_result_lines,
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


def select_trace(app: App, value: str) -> None:
    """Pick a value in the Tooth trace row, as a click on it would.

    The handler hangs off `<<ComboboxSelected>>` rather than off the variable -
    so that `refresh` re-rendering the row cannot rewrite the boxes it was
    rendered from - and a test setting the variable has to call it in the same
    way the binding does.
    """
    app.vars["trace_kind"].set(value)
    app.on_trace_change()
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


@pytest.mark.parametrize("key", ["bevel", "spur", "hypoid", "planetary"])
def test_auto_size_works_on_every_tab(app, key):
    """It read `p.z1` off whatever was in hand, and a planetary set has no z1.

    `GearKind.counts` exists to answer this - a pair calls them z1 and z2, a
    planetary set z_sun and z_planet - and going through it is the difference
    between the button working and raising AttributeError on one tab of three.
    """
    app.kind_key.set(key)
    app.on_kind_change()
    set_input(app, "face_width", "3")

    app.auto_size()

    first, second = KINDS[key].counts(app._params)
    sized = KINDS[key].params_cls.with_defaults(app._params.module, first, second)
    assert app._params.face_width == pytest.approx(sized.face_width)


def test_auto_size_keeps_a_backlash_that_was_typed(app):
    """It is not a blank dimension, so "Auto-size blank" has no business on it.

    Staying out of the `*_AUTO` tuples is what does it, and that is easy to
    undo by accident when the next field is added beside it.
    """
    set_input(app, "backlash", "0.08")
    app.auto_size()
    assert app._params.backlash == pytest.approx(0.08)


# The one scene that is not a view of a single member. A bevel pair's tooth
# trace is one arc in the shared crown plane - both members map that same curve
# onto their own cones, which is the meshing condition - so titling it after
# whichever member happens to be selected would state something untrue.
SHARED_SCENES = {"trace"}


@pytest.mark.parametrize("kind_key", ["bevel", "spur", "hypoid", "planetary"])
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
    # Every row filled in, except the ones whose blank is itself the answer -
    # `cutter_radius` blank means Am, and writing a number in would change the
    # set rather than describe it.
    for field in FIELDS:
        if field.optional:
            continue
        assert app.vars[field.attr].get() != ""


def test_a_blank_optional_field_reads_back_as_none(app):
    """Blank is a value on an optional row, not a parse error.

    `cutter_radius` is `float | None` and None is what `with_defaults` reads as
    "the Gleason nominal, Am". Every other row treats an empty box as something
    the user has not finished typing.
    """
    field = next(f for f in FIELDS if f.attr == "cutter_radius")
    assert field.optional
    assert field.parse("  ") is None
    assert field.format(None) == ""
    # The round trip both ways, which is the pairing every other field is held to.
    assert field.parse(field.format(39.3035)) == pytest.approx(39.3035)

    set_input(app, "cutter_radius", "")
    params, problems = app.read_params()
    assert problems == []
    assert params.cutter_radius is None


def test_a_zerol_set_is_reachable_from_the_window(app):
    """The window could not express a Zerol set at all before the cutter row.

    With no cutter radius to type, a zero spiral angle could only ever mean a
    straight gear - so the one bevel variant the geometry had supported all
    along was the one the GUI could not ask for.
    """
    set_input(app, "spiral_angle", "0")
    set_input(app, "cutter_radius", "39.3035")

    params, problems = app.read_params()
    assert problems == []
    assert params.is_curved
    assert params.trace_kind == "zerol"
    assert "zerol" in app._status_text()


def test_a_zerol_preset_survives_a_load(app, tmp_path):
    """It did not: the cutter radius was dropped on the way back in.

    `set_params` only writes the widgets its type has rows for, and `read_params`
    then rebuilds by replacing those values onto the *previous* params object. A
    field with no row was therefore never written and never read, so a saved
    Zerol set came back as the straight set the widgets could describe.
    """
    set_input(app, "spiral_angle", "0")
    set_input(app, "cutter_radius", "39.3035")
    path = tmp_path / "zerol.json"
    app._params.to_json(path)

    set_input(app, "cutter_radius", "")
    assert app._params.cutter_radius is None

    app.load_preset(path)
    assert app._params.cutter_radius == pytest.approx(39.3035)
    assert app._params.is_curved


def test_the_trace_row_names_the_state_the_two_inputs_encode(app):
    """straight / zerol / spiral, off a spiral angle and a cutter radius.

    The row is derived, so this is really a test that it renders rather than
    stores: nothing writes "zerol" anywhere: it is what those two inputs *mean*.
    """
    assert app.vars["trace_kind"].get() == "straight"

    set_input(app, "cutter_radius", "39.3035")
    assert app.vars["trace_kind"].get() == "zerol"

    set_input(app, "spiral_angle", "35")
    assert app.vars["trace_kind"].get() == "spiral"

    set_input(app, "cutter_radius", "")
    set_input(app, "spiral_angle", "0")
    assert app.vars["trace_kind"].get() == "straight"


def test_selecting_a_trace_writes_the_rows_it_stands_for(app):
    """The one row that acts rather than being parsed."""
    select_trace(app, "zerol")
    assert app._params.spiral_angle == 0.0
    assert app._params.cutter_radius == pytest.approx(39.3035, abs=1e-3)
    assert app._params.is_curved

    select_trace(app, "spiral")
    assert app._params.spiral_angle == pytest.approx(35.0)
    assert app._params.cutter_radius == pytest.approx(39.3035, abs=1e-3)

    select_trace(app, "straight")
    assert app._params.spiral_angle == 0.0
    assert app._params.cutter_radius is None
    assert not app._params.is_curved


def test_selecting_zerol_gives_the_same_set_the_zerol_flag_does(app):
    """Window and command line have to agree, or one of them is quoting a
    nominal cutter for a gear the user does not have.

    Am is measured against the face width, and a curved set's face is the
    tighter Gleason 0.30*Ao - so this only lands if the offered cutter is sized
    the way `--zerol` sizes it rather than against the straight width still in
    the box at the moment of selection.
    """
    select_trace(app, "zerol")
    app.auto_size()
    assert app._params == BevelSetParams.with_defaults(2.0, 17, 43, zerol=True)


def test_selecting_a_trace_keeps_a_cutter_radius_already_chosen(app):
    """A cutter picked to flatten the swing is not re-picked behind your back."""
    select_trace(app, "zerol")
    set_input(app, "cutter_radius", "60")
    select_trace(app, "spiral")
    assert app._params.cutter_radius == pytest.approx(60.0)
    select_trace(app, "zerol")
    assert app._params.cutter_radius == pytest.approx(60.0)


def test_the_trace_row_is_never_read_back_as_an_input(app):
    """`trace_kind` is a property; `dataclasses.replace` would raise on it.

    Asserted through `read_params` rather than by inspecting the flag, because
    the flag existing is not the point - the point is that a derived row cannot
    reach the constructor.
    """
    field = next(f for f in FIELDS if f.attr == "trace_kind")
    assert field.derived
    app.vars["trace_kind"].set("nonsense")
    params, problems = app.read_params()
    assert problems == []
    assert params is not None


def test_auto_size_keeps_a_zerol_set_curved(app):
    """The face width a Zerol set gets is the curved one, 0.30*Ao not Ao/3.

    `auto_kwargs` had to learn about the cutter radius for this: it is the only
    input saying a Zerol tooth is curved, so without it "Auto-size blank" handed
    back the straight set's 15.41 mm and the validator then warned about it.
    """
    set_input(app, "spiral_angle", "0")
    set_input(app, "cutter_radius", "39.3035")
    app.auto_size()
    assert app._params.face_width == pytest.approx(13.87, abs=0.01)
    assert app._params.cutter_radius == pytest.approx(39.3035)


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

        # Mirrors the real result classes. The formatter reads these rather
        # than a fixed pinion and gear, because a planetary build has three
        # parts and two meshes.
        @property
        def parts(self):
            return (self.pinion, self.gear)

        @property
        def gear_ratios(self):
            return (self.gear_ratio,)

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


@pytest.mark.parametrize("key", ["bevel", "spur", "planetary"])
def test_only_the_active_type_s_rows_are_shown(app, key):
    app.kind_key.set(key)
    app.on_kind_change()
    assert visible(app) == [f.attr for f in KINDS[key].fields]


@pytest.mark.parametrize("key", ["bevel", "spur", "planetary"])
def test_every_type_offers_a_backlash_row(app, key):
    """It is a tooth-form input, so every type has one and they share a variable.

    Sharing is the point: a backlash typed on a spur set is still there when the
    set becomes a bevel one, the same way the module and the tooth counts are.
    """
    app.kind_key.set(key)
    app.on_kind_change()
    assert "backlash" in visible(app)
    # Between `hand` and the blank dimensions, which is where it belongs: it
    # changes the tooth, not the solid the tooth is cut out of.
    rows = visible(app)
    assert rows.index("hand") < rows.index("backlash") < rows.index("face_width")


@pytest.mark.parametrize("key", ["bevel", "spur", "planetary"])
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

        # Mirrors the real result classes. The formatter reads these rather
        # than a fixed pinion and gear, because a planetary build has three
        # parts and two meshes.
        @property
        def parts(self):
            return (self.pinion, self.gear)

        @property
        def gear_ratios(self):
            return (self.gear_ratio,)

    lines = App._format_result(Result(), _spur_result_lines)
    text = "\n".join(lines)
    assert "centre distance 60.0000 mm measured" in text
    assert "parallel is 0" in text
    assert "the set turns" in text


# --- three members rather than two -----------------------------------------


def test_each_type_shows_only_its_own_member_buttons(app):
    """A pair has a pinion and a gear; a planetary train has three members."""
    for key, expected in (
        ("bevel", ("pinion", "gear")),
        ("spur", ("pinion", "gear")),
        ("planetary", ("sun", "planet", "ring")),
    ):
        app.kind_key.set(key)
        app.on_kind_change()
        assert KINDS[key].members == expected
        assert app.member.get() in expected
        shown = {
            name for name, button in app.member_buttons.items()
            if button.winfo_manager()
        }
        assert shown == set(expected)


def test_the_third_readout_column_appears_only_for_a_planetary_set(app):
    """A gear pair's readout must look exactly as it did before trains existed."""
    app.kind_key.set("spur")
    app.on_kind_change()
    assert "third" not in app.readout.cget("displaycolumns")
    assert app.readout.heading("first")["text"] == "Pinion"

    app.kind_key.set("planetary")
    app.on_kind_change()
    assert "third" in app.readout.cget("displaycolumns")
    assert app.readout.heading("first")["text"] == "Sun"
    assert app.readout.heading("third")["text"] == "Ring"


def test_switching_to_planetary_carries_the_tooth_counts_across(app):
    """The field names differ - z1/z2 against z_sun/z_planet - so this is wiring.

    `GearKind.count_attrs` is what makes it work, and reading the counts through
    the kind that *wrote* the params object is what makes it work in both
    directions.
    """
    app.kind_key.set("spur")
    app.on_kind_change()
    set_input(app, "z1", "24")
    set_input(app, "z2", "18")

    app.kind_key.set("planetary")
    app.on_kind_change()
    assert (app._params.z_sun, app._params.z_planet) == (24, 18)
    assert app._params.z_ring == 60

    app.kind_key.set("spur")
    app.on_kind_change()
    assert (app._params.z1, app._params.z2) == (24, 18)


def test_a_planetary_set_computes_and_draws(app):
    app.kind_key.set("planetary")
    app.on_kind_change()
    set_input(app, "z_sun", "24")
    set_input(app, "z_planet", "18")
    set_input(app, "n_planets", "3")

    assert app._validation.ok
    assert app._geo is not None
    assert app._geo.ring.z == 60

    app.scene_key.set("train")
    app._on_view_change()
    assert app._scene is not None
    assert app._scene.key == "train"
    # Sun + ring + every planet, so a good deal more than a pair's two profiles.
    assert len(app._scene.polylines) > 100


def test_a_hypoid_set_computes_and_draws(app):
    app.kind_key.set("hypoid")
    app.on_kind_change()
    for attr, value in (
        ("module", str(170.0 / 42.0)),
        ("z1", "13"),
        ("z2", "42"),
        ("offset", "15"),
        ("face_width", "30"),
        ("spiral_angle", "50"),
        ("cutter_radius", "63.5"),
    ):
        set_input(app, attr, value)

    assert app._validation.ok
    assert app._geo is not None
    assert app._geo.pitch_plane_offset == pytest.approx(15.075, abs=0.002)
    assert app._scene is not None
    assert app._scene.key == "contact"
    assert "offset 15 mm" in app.status.cget("text")
    assert app.build_button.instate(["!disabled"])


def test_a_negative_hypoid_spiral_magnitude_is_rejected_in_the_gui(app):
    app.kind_key.set("hypoid")
    app.on_kind_change()
    set_input(app, "spiral_angle", "-35")

    assert app._geo is None
    assert "Pinion spiral-angle magnitude" in app.messages.get("1.0", "end")
    assert "non-negative magnitude" in app.messages.get("1.0", "end")
    assert app.build_button.instate(["disabled"])


def test_a_failed_assembly_condition_blocks_the_build_button(app):
    app.kind_key.set("planetary")
    app.on_kind_change()
    set_input(app, "z_sun", "27")
    set_input(app, "z_planet", "17")
    set_input(app, "n_planets", "3")

    assert not app._validation.ok
    assert app.build_button.instate(["disabled"])
    assert "not divisible" in app.messages.get("1.0", "end")


def test_the_planetary_status_line_names_train_quantities(app):
    app.kind_key.set("planetary")
    app.on_kind_change()
    set_input(app, "z_sun", "24")
    set_input(app, "z_planet", "18")

    status = app.status.cget("text")
    assert "ring 60" in status
    assert "3.500:1 with the ring held" in status
    assert "assembles" in status


def test_every_type_can_name_a_file_to_export(app):
    """`_default_stem` runs before every Export and Save, on every tab.

    It read `p.z1` directly until a planetary set was exported - and a
    planetary set has z_sun and z_planet instead, so all three of that tab's
    file buttons raised AttributeError. The round-trip test above did not catch
    it because it calls `to_json` rather than the dialog that names the file.
    """
    for key, kind in KINDS.items():
        app.kind_key.set(key)
        app.on_kind_change()
        for member in kind.members:
            app.member.set(member)
            stem = app._default_stem()
            first, second = kind.counts(app._params)
            assert stem == f"{member}_m2_z{first}x{second}", stem


def test_the_report_formatter_reads_fields_the_real_result_classes_have():
    """The stub tests above cannot catch a name the real classes never had.

    `_format_result` read `result.clocking_deg` for every type, and a
    `PlanetarySetResult` carries `clockings_deg` and `ring_clocking_deg`
    instead - so a planetary build raised AttributeError in the report, after
    SOLIDWORKS had built and saved the whole assembly. Both stubs above spelled
    the attribute the formatter's way, which is exactly the blind spot.

    So this asserts against the dataclasses themselves rather than against a
    stub: every field the formatter and the four `result_lines` touch has to
    exist on the class that will really be passed in. It imports from `sw`,
    which needs pywin32 only at call time, not at import time.
    """
    sw = pytest.importorskip("gears.sw")

    wanted = {
        sw.SetResult: (
            _bevel_result_lines,
            ("measured_shaft_angle_deg", "shaft_angle_error_deg", "clocking_deg"),
        ),
        sw.SpurSetResult: (
            _spur_result_lines,
            ("measured_centre_distance_mm", "centre_distance_error_mm",
             "measured_axis_angle_deg", "clocking_deg"),
        ),
        sw.HypoidSetResult: (
            _hypoid_result_lines,
            ("measured_shaft_angle_deg", "shaft_angle_error_deg",
             "measured_offset_mm", "offset_error_mm", "clocking_deg"),
        ),
        sw.PlanetarySetResult: (
            _planetary_result_lines,
            ("centre_distance_mm", "planets", "worst_position_error_mm",
             "clockings_deg", "ring_clocking_deg"),
        ),
    }
    # What `_format_result` itself reads, on every type alike.
    shared = (
        "parts", "mates", "gear_ratios", "articulates", "interference_count",
        "interference_volume_mm3", "assembly_path",
    )

    for cls, (_lines, names) in wanted.items():
        have = set(cls.__dataclass_fields__) | {
            name for name in dir(cls) if not name.startswith("_")
        }
        for name in names + shared:
            assert name in have, f"{cls.__name__} has no {name}"
