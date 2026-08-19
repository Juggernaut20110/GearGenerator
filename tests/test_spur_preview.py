"""Spur scenes and exports. No tkinter, no SOLIDWORKS."""

from __future__ import annotations

import math

import pytest

from gears.spur import preview
from gears.spur.geometry import compute_set
from gears.spur.params import SpurSetParams

ANCHOR = SpurSetParams.with_defaults(2.0, 17, 43)
ANCHOR_HELICAL = SpurSetParams.with_defaults(2.0, 17, 43, helix_angle=15.0)

MEMBERS = ["pinion", "gear"]
SCENES = ["transverse", "twist", "blank"]


@pytest.fixture
def geo():
    return compute_set(ANCHOR)


@pytest.fixture
def helical():
    return compute_set(ANCHOR_HELICAL)


# --- the space loop --------------------------------------------------------


@pytest.mark.parametrize("member", MEMBERS)
def test_space_loop_stops_at_the_tip_not_past_it(geo, member):
    """The cut runs past the tip on purpose; the drawing must not.

    `tooth_space_section` carries the loop out to `r_cap` so the loft cut clears
    the blank. Drawn as-is that is a notch outside the tip circle where the real
    tooth has a top land, so the preview closes the space across the tip
    instead.
    """
    from gears.spur.geometry import tooth_space_section

    section = tooth_space_section(geo, member)
    drawn = preview.space_loop(section)
    assert max(math.hypot(x, y) for x, y in drawn) == pytest.approx(
        section.r_tip, abs=1e-9
    )
    assert max(math.hypot(x, y) for x, y in section.loop_2d) == pytest.approx(
        section.r_cap, abs=1e-9
    )


@pytest.mark.parametrize("member", MEMBERS)
def test_space_loop_is_still_closed_and_symmetric(geo, member):
    from gears.spur.geometry import tooth_space_section

    drawn = preview.space_loop(tooth_space_section(geo, member))
    assert len(drawn) > 20
    ys = sorted(round(y, 9) for _, y in drawn)
    assert all(a == pytest.approx(-b, abs=1e-9) for a, b in zip(ys, reversed(ys)))


# --- scenes ----------------------------------------------------------------


@pytest.mark.parametrize("key", SCENES)
@pytest.mark.parametrize("member", MEMBERS)
def test_every_scene_builds_and_is_non_empty(geo, key, member):
    scene = preview.build_scene(geo, member, key)
    assert scene.key == key
    assert scene.title.startswith(member)
    assert scene.polylines
    assert all(len(pl.points) >= 2 for pl in scene.polylines)


@pytest.mark.parametrize("key", SCENES)
@pytest.mark.parametrize("member", MEMBERS)
def test_every_scene_builds_for_helical_teeth_too(helical, key, member):
    scene = preview.build_scene(helical, member, key)
    assert scene.polylines
    x0, y0, x1, y1 = scene.bounds()
    assert x1 > x0 and y1 > y0


def test_an_unknown_scene_is_refused(geo):
    with pytest.raises(ValueError, match="unknown scene"):
        preview.build_scene(geo, "pinion", "nope")


def test_scene_labels_cover_every_builder():
    assert {k for k, _ in preview.SCENE_LABELS} == set(preview.SCENE_BUILDERS)


@pytest.mark.parametrize("member", MEMBERS)
def test_the_twist_scene_draws_one_space_per_tooth(geo, member):
    """Straight teeth: one loop per tooth, and the two ends coincide."""
    scene = preview.build_scene(geo, member, "twist")
    spaces = [pl for pl in scene.polylines if pl.style in ("outer", "inner")]
    assert len(spaces) == geo.member(member).z
    assert all(pl.style == "outer" for pl in spaces)


@pytest.mark.parametrize("member", MEMBERS)
def test_the_twist_scene_draws_both_ends_when_they_differ(helical, member):
    scene = preview.build_scene(helical, member, "twist")
    z = helical.member(member).z
    styles = [pl.style for pl in scene.polylines if pl.style in ("outer", "inner")]
    assert styles.count("outer") == z
    assert styles.count("inner") == z


@pytest.mark.parametrize("member", MEMBERS)
def test_the_twist_scene_spaces_close_around_the_circle(helical, member):
    """z spaces at one angular pitch each must fill exactly one turn.

    The cheapest check that the tooth count and the angular pitch agree. Every
    space is the same loop rotated, so the same vertex of each one is a
    consistent reference; the last gap is the one that wraps through pi, and it
    has to be a pitch like all the others or the spaces do not close.
    """
    scene = preview.build_scene(helical, member, "twist")
    m = helical.member(member)
    fronts = [pl for pl in scene.polylines if pl.style == "outer"]
    assert len(fronts) == m.z

    angles = sorted(math.atan2(pl.points[0][1], pl.points[0][0]) for pl in fronts)
    gaps = [b - a for a, b in zip(angles, angles[1:])]
    gaps.append(angles[0] + 2.0 * math.pi - angles[-1])

    assert sum(gaps) == pytest.approx(2.0 * math.pi, abs=1e-12)
    assert all(g == pytest.approx(m.angular_pitch, abs=1e-9) for g in gaps)


@pytest.mark.parametrize("member", MEMBERS)
def test_the_blank_scene_reaches_the_hub(geo, member):
    scene = preview.build_scene(geo, member, "blank")
    _, y0, x1, y1 = scene.bounds()
    p = geo.params
    assert y1 >= p.face_width + p.hub_thickness - 1e-9
    assert x1 >= geo.member(member).tip_r - 1e-9
    # The cut reaches past both faces, which is what the overshoot is for.
    assert y0 < 0.0


def test_the_transverse_title_names_the_transverse_quantities(helical):
    title = preview.build_scene(helical, "pinion", "transverse").title
    assert "m_t = 2.0706" in title
    assert "alpha_t = 20.647" in title


def test_the_twist_title_says_straight_when_it_is(geo):
    assert "straight teeth" in preview.build_scene(geo, "pinion", "twist").title


@pytest.mark.parametrize("member,expected", [("pinion", "right"), ("gear", "left")])
def test_the_twist_title_names_the_hand(helical, member, expected):
    title = preview.build_scene(helical, member, "twist").title
    assert f"{expected} hand" in title


# --- the readout -----------------------------------------------------------


def test_derived_rows_cover_the_set_and_both_members(geo):
    rows = preview.derived_rows(geo)
    labels = [r.label for r in rows]
    assert "SET" in labels and "MEMBERS" in labels
    for wanted in (
        "centre distance",
        "transverse module",
        "contact ratio, transverse",
        "contact ratio, axial",
        "twist over the face",
    ):
        assert wanted in labels


def test_derived_rows_report_the_numbers_the_geometry_holds(geo):
    rows = {r.label: r for r in preview.derived_rows(geo)}
    assert rows["centre distance"].pinion == "60.0000"
    assert rows["teeth"].pinion == "17"
    assert rows["teeth"].gear == "43"


def test_the_axial_pitch_of_straight_teeth_reads_as_infinite(geo):
    rows = {r.label: r for r in preview.derived_rows(geo)}
    assert rows["axial pitch"].pinion == "inf"


# --- export ----------------------------------------------------------------


@pytest.mark.parametrize("key", SCENES)
def test_dxf_is_well_formed(geo, key):
    lines = preview.dxf_lines(preview.build_scene(geo, "pinion", key))
    assert lines[0] == "0"
    assert lines[1] == "SECTION"
    assert lines[-1] == "EOF"
    assert len(lines) % 2 == 0
    assert "POLYLINE" in lines


def test_dxf_writes_to_a_file(geo, tmp_path):
    path = preview.write_dxf(
        tmp_path / "nested" / "spur.dxf", preview.build_scene(geo, "pinion", "blank")
    )
    assert path.exists()
    assert path.read_text(encoding="utf-8").rstrip().endswith("EOF")


@pytest.mark.parametrize("member", MEMBERS)
def test_csv_holds_both_end_sections(helical, member):
    lines = preview.csv_lines(helical, member)
    assert lines[0] == "member,section,index,x,y,z"

    rows = [line.split(",") for line in lines[1:]]
    assert {r[1] for r in rows} == {"front", "back"}
    assert all(len(r) == 6 for r in rows)
    assert all(r[0] == member for r in rows)


def test_csv_end_sections_differ_for_helical_teeth(helical):
    """Which is the whole reason both are written rather than one."""
    rows = [line.split(",") for line in preview.csv_lines(helical, "pinion")[1:]]
    front = [(float(r[3]), float(r[4])) for r in rows if r[1] == "front"]
    back = [(float(r[3]), float(r[4])) for r in rows if r[1] == "back"]
    assert len(front) == len(back)
    assert max(math.dist(a, b) for a, b in zip(front, back)) > 1.0


def test_csv_end_sections_coincide_for_straight_teeth(geo):
    rows = [line.split(",") for line in preview.csv_lines(geo, "pinion")[1:]]
    front = [(float(r[3]), float(r[4])) for r in rows if r[1] == "front"]
    back = [(float(r[3]), float(r[4])) for r in rows if r[1] == "back"]
    assert all(math.dist(a, b) < 1e-12 for a, b in zip(front, back))


def test_csv_writes_to_a_file(geo, tmp_path):
    path = preview.write_csv(tmp_path / "nested" / "spur.csv", geo, "pinion")
    assert path.exists()
    assert path.read_text(encoding="utf-8").startswith("member,section,index")
