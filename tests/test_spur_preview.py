"""Spur scenes and exports. No tkinter, no SOLIDWORKS."""

from __future__ import annotations

import math

import pytest

from gears.involute import inv
from gears.spur import preview
from gears.spur.geometry import compute_set, tooth_space_section
from gears.spur.params import SpurSetParams

ANCHOR = SpurSetParams.with_defaults(2.0, 17, 43)
ANCHOR_HELICAL = SpurSetParams.with_defaults(2.0, 17, 43, helix_angle=15.0)
ANCHOR_INTERNAL = SpurSetParams.with_defaults(2.0, 18, 60, internal=True)

MEMBERS = ["pinion", "gear"]
SCENES = ["transverse", "twist", "blank"]


@pytest.fixture
def geo():
    return compute_set(ANCHOR)


@pytest.fixture
def helical():
    return compute_set(ANCHOR_HELICAL)


@pytest.fixture
def internal():
    return compute_set(ANCHOR_INTERNAL)


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


# --- the tooth -------------------------------------------------------------


def in_metal(m, radius: float, theta: float) -> bool:
    """Whether a point in the member's own frame is inside its metal.

    Written out from the involute relation rather than read off the generated
    profile - the same helper `test_planetary.py` uses for the interlock tests,
    and here for the same reason: a check on the drawing has to inherit nothing
    from the code that produced the drawing.
    """
    if m.internal:
        if radius >= m.root_r:
            return True
        if radius <= m.tip_r:
            return False
    else:
        if radius <= m.root_r:
            return True
        if radius >= m.tip_r:
            return False

    alpha_r = math.acos(min(1.0, m.base_r / max(radius, m.base_r)))
    half_space = (
        m.psi0 - inv(alpha_r) if m.internal
        else m.half_pitch - m.psi0 + inv(alpha_r)
    )
    offset = theta % m.angular_pitch
    return min(offset, m.angular_pitch - offset) > half_space


def tooth_of(geo, member: str) -> list[tuple[float, float]]:
    m = geo.member(member)
    return preview.tooth_loop(tooth_space_section(geo, member), m.angular_pitch)


def just_inside(geo, m, loop, fraction: float = 1e-3) -> list[tuple[float, float]]:
    """The loop's vertices pulled a hair inward, with the fillet band dropped.

    Two adjustments, both so the question asked is the one meant.

    The vertices sit *on* the boundary - a flank point is exactly on the flank,
    a top-land point exactly on the tip circle - so asking whether one is inside
    the metal is asking a coin toss to come up heads. Pulling each toward the
    loop's centroid asks about the shape rather than about its edge.

    And the band within one fillet radius of the root circle comes out, because
    `in_metal` has no fillet in it: below the base circle the involute is
    undefined and the relation falls back to a radial line, which is the
    standard simplification the geometry module also makes. A real fillet bulges
    into the space, so its own points are metal that the closed form cannot see.
    That is a gap in the reference, not in the drawing.
    """
    cx = sum(x for x, _ in loop) / len(loop)
    cy = sum(y for _, y in loop) / len(loop)
    fillet = geo.params.fillet_factor * geo.params.module
    return [
        (px, py)
        for px, py in (
            (x + fraction * (cx - x), y + fraction * (cy - y)) for x, y in loop
        )
        if abs(math.hypot(px, py) - m.root_r) > fillet
    ]


@pytest.mark.parametrize("member", MEMBERS)
def test_the_tooth_and_its_space_fill_one_angular_pitch(geo, member):
    """A tooth is the complement of two adjacent spaces, so this is exact.

    Measured at the pitch circle, where a tooth thickness is quoted. Off by a
    fillet, a top land or a mirrored flank and the sum misses.
    """
    m = geo.member(member)
    section = tooth_space_section(geo, member)

    def width(loop):
        hits = []
        for (x0, y0), (x1, y1) in zip(loop, loop[1:] + loop[:1]):
            r0, r1 = math.hypot(x0, y0), math.hypot(x1, y1)
            if (r0 - m.pitch_r) * (r1 - m.pitch_r) < 0.0:
                t = (m.pitch_r - r0) / (r1 - r0)
                hits.append(math.atan2(y0 + t * (y1 - y0), x0 + t * (x1 - x0)))
        return max(hits) - min(hits)

    tooth = width(preview.tooth_loop(section, m.angular_pitch))
    space = width(preview.space_loop(section))
    assert tooth + space == pytest.approx(m.angular_pitch, abs=1e-12)


@pytest.mark.parametrize("member", MEMBERS)
def test_the_drawn_tooth_is_made_of_metal_and_mis_centring_it_is_not(geo, member):
    """Every point of the drawn tooth is inside the gear. The partner fails.

    This is the check that matters to the train scene, which is the one view
    where two members are drawn against each other: a tooth drawn where the
    metal is not would show a mesh interlocking when it does not. The paired
    half is the error the tooth boundary can make quietly - centring it on angle
    0, where the *space* is, which still draws a perfectly gear-shaped ring of
    teeth in exactly the wrong half of every pitch.

    Measured, sampled points landing outside the metal:

        pinion, 17 teeth      0 of 87 correct,  41 of 87 half a pitch out
        gear,   43 teeth      0 of 69 correct,  31 of 69
        ring,   60 teeth      0 of 81 correct,  49 of 81
    """
    m = geo.member(member)
    tooth = tooth_of(geo, member)

    def outside(loop) -> tuple[int, int]:
        sampled = just_inside(geo, m, loop)
        return sum(
            1 for x, y in sampled
            if not in_metal(m, math.hypot(x, y), math.atan2(y, x))
        ), len(sampled)

    assert outside(tooth)[0] == 0

    wrong, total = outside(preview.rotate(tooth, -m.angular_pitch / 2.0))
    assert wrong > total // 3


@pytest.mark.parametrize("member", MEMBERS)
def test_the_tooth_stops_at_the_tip_not_out_at_the_cut(geo, member):
    """The same guarantee `space_loop` carries, restated for the tooth."""
    section = tooth_space_section(geo, member)
    radii = [math.hypot(x, y) for x, y in preview.tooth_loop(
        section, geo.member(member).angular_pitch
    )]
    assert max(radii) == pytest.approx(section.r_tip, abs=1e-9)
    assert min(radii) == pytest.approx(section.r_root, abs=1e-9)


def test_a_ring_gears_tooth_points_inward(internal):
    """The tip is the *innermost* radius of a ring tooth, and the root outermost.

    The tooth boundary reads both off the section's own segments rather than
    being told which is which, so a ring gear needs no special case - and this
    is the test that says the reading is the right way round.
    """
    section = tooth_space_section(internal, "gear")
    radii = [math.hypot(x, y) for x, y in tooth_of(internal, "gear")]

    assert min(radii) == pytest.approx(section.r_tip, abs=1e-9)
    assert max(radii) == pytest.approx(section.r_root, abs=1e-9)
    assert section.r_tip < internal.gear.pitch_r < section.r_root


def test_a_ring_gears_tooth_is_also_made_of_metal(internal):
    """The same in-material check and its partner, on the member whose radii
    run backwards. A ring gear reaches the right answer from the other side -
    tip innermost, root outermost - and neither the boundary nor this check has
    a special case for it."""
    m = internal.gear
    tooth = tooth_of(internal, "gear")

    def outside(loop) -> int:
        return sum(
            1 for x, y in just_inside(internal, m, loop)
            if not in_metal(m, math.hypot(x, y), math.atan2(y, x))
        )

    assert outside(tooth) == 0
    assert outside(preview.rotate(tooth, -m.angular_pitch / 2.0)) == 49


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
        "reference centre distance a",
        "transverse module m_t",
        "epsilon_alpha (transverse)",
        "epsilon_beta (overlap)",
        "twist over the face",
    ):
        assert wanted in labels


def test_derived_rows_report_the_numbers_the_geometry_holds(geo):
    rows = {r.label: r for r in preview.derived_rows(geo)}
    assert rows["reference centre distance a"].pinion == "60.0000"
    assert rows["working centre distance a_w"].pinion == "60.0000"
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
