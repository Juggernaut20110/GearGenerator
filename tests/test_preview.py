"""Checks on the preview layer: scenes, view fitting, drawing, and export.

No tkinter here - `draw_scene` only ever calls `create_line` and `create_text`,
so a recording stub stands in for a real Canvas.
"""

from __future__ import annotations

import math

import pytest

from gears.bevel import preview
from gears.bevel.geometry import compute_set, to_cone_3d, tooth_space_section
from gears.bevel.params import BevelSetParams

ANCHOR = BevelSetParams.with_defaults(2.0, 17, 43)
GEO = compute_set(ANCHOR)

TAU = 2.0 * math.pi


class FakeCanvas:
    """Records what would have been drawn."""

    def __init__(self):
        self.lines: list[tuple[tuple, dict]] = []
        self.texts: list[tuple[float, float, dict]] = []

    def create_line(self, *coords, **kwargs):
        self.lines.append((coords, kwargs))

    def create_text(self, x, y, **kwargs):
        self.texts.append((x, y, kwargs))


def radii(points) -> list[float]:
    return [math.hypot(x, y) for x, y in points]


def angles(points) -> list[float]:
    return [math.atan2(y, x) for x, y in points]


# --- View ------------------------------------------------------------------


def test_fit_centres_the_bounds_and_preserves_aspect():
    """The tighter of the two axes sets the scale, and the content is centred."""
    view = preview.View.fit((0.0, 0.0, 10.0, 5.0), 200.0, 100.0, margin=20.0)

    # 160 px for 10 mm across, 60 px for 5 mm up: the vertical fit wins.
    assert view.scale == pytest.approx(12.0)
    assert view.to_canvas(0.0, 0.0) == pytest.approx((40.0, 80.0))
    assert view.to_canvas(10.0, 5.0) == pytest.approx((160.0, 20.0))


def test_fit_flips_y_so_positive_is_up():
    view = preview.View.fit((-1.0, -1.0, 1.0, 1.0), 100.0, 100.0)
    _, low = view.to_canvas(0.0, -1.0)
    _, high = view.to_canvas(0.0, 1.0)
    assert high < low


def test_from_canvas_inverts_to_canvas():
    view = preview.View.fit((3.0, -2.0, 9.0, 4.0), 300.0, 220.0, zoom=1.7, pan=(11.0, -5.0))
    for point in ((3.0, -2.0), (5.5, 1.25), (9.0, 4.0)):
        assert view.from_canvas(*view.to_canvas(*point)) == pytest.approx(point)


def test_zoom_scales_and_pan_translates():
    plain = preview.View.fit((0.0, 0.0, 10.0, 10.0), 200.0, 200.0)
    zoomed = preview.View.fit((0.0, 0.0, 10.0, 10.0), 200.0, 200.0, zoom=2.0)
    panned = preview.View.fit((0.0, 0.0, 10.0, 10.0), 200.0, 200.0, pan=(7.0, -3.0))

    assert zoomed.scale == pytest.approx(2.0 * plain.scale)
    assert panned.to_canvas(0.0, 0.0)[0] == pytest.approx(plain.to_canvas(0.0, 0.0)[0] + 7.0)
    assert panned.to_canvas(0.0, 0.0)[1] == pytest.approx(plain.to_canvas(0.0, 0.0)[1] - 3.0)


def test_empty_scene_has_usable_bounds():
    """A degenerate scene must not make `fit` divide by zero."""
    view = preview.View.fit(preview.Scene("x", "x").bounds(), 100.0, 100.0)
    assert view.scale > 0.0


# --- the two tooth space boundaries ----------------------------------------


@pytest.mark.parametrize("member", ["pinion", "gear"])
@pytest.mark.parametrize("end", ["outer", "inner"])
def test_space_loop_stops_at_the_tip_where_the_cut_loop_runs_past_it(member, end):
    """`space_loop` is the tooth; `loop_2d` is what the cut needs. See the docstring."""
    section = tooth_space_section(GEO, member, end)

    assert max(radii(preview.space_loop(section))) == pytest.approx(section.r_tip)
    assert max(radii(section.loop_2d)) == pytest.approx(section.r_cap)
    assert section.r_cap > section.r_tip


@pytest.mark.parametrize("member", ["pinion", "gear"])
def test_space_loop_is_closed_without_a_duplicated_point(member):
    loop = preview.space_loop(tooth_space_section(GEO, member, "outer"))
    assert len(loop) > 20
    assert math.dist(loop[0], loop[-1]) > 1e-6


@pytest.mark.parametrize("member", ["pinion", "gear"])
def test_space_loop_is_symmetric_about_the_centreline(member):
    """The space is centred on angle 0, so its angular extent must be too."""
    loop = preview.space_loop(tooth_space_section(GEO, member, "outer"))
    a = angles(loop)
    assert max(a) == pytest.approx(-min(a), rel=1e-9)


# --- the tooth -------------------------------------------------------------


def developed_pitch(member: str) -> float:
    """The angular pitch the spaces repeat at in the developed plane."""
    return TAU / GEO.member(member).virtual_teeth


def crossings_at(loop, radius: float) -> tuple[float, float]:
    """The two angles where a loop crosses the circle of radius `radius`.

    Found by intersecting the polyline with the circle rather than by taking the
    extreme vertices, so it measures the shape at one radius the way a tooth
    thickness is measured, and not at the widest point of the whole loop.
    """
    hits = []
    for (x0, y0), (x1, y1) in zip(loop, loop[1:] + loop[:1]):
        r0, r1 = math.hypot(x0, y0), math.hypot(x1, y1)
        if (r0 - radius) * (r1 - radius) < 0.0:
            t = (radius - r0) / (r1 - r0)
            hits.append(math.atan2(y0 + t * (y1 - y0), x0 + t * (x1 - x0)))
    return min(hits), max(hits)


def angular_width_at(loop, radius: float) -> float:
    lo, hi = crossings_at(loop, radius)
    return hi - lo


@pytest.mark.parametrize("member", ["pinion", "gear"])
@pytest.mark.parametrize("end", ["outer", "inner"])
def test_the_tooth_and_its_space_partition_the_angular_pitch(member, end):
    """The property the whole tooth boundary rests on.

    A tooth is the complement of two adjacent spaces, so at any radius the two
    widths have to add up to exactly one angular pitch. Off by a fillet, a top
    land or a mirrored flank and this misses; off by half a pitch - the
    dangerous error, because a ring of teeth still looks like a ring of teeth -
    and it misses by half.
    """
    m = GEO.member(member)
    section = tooth_space_section(GEO, member, end)
    step = developed_pitch(member)
    radius = GEO.section_scale * m.virtual_pitch_r if end == "inner" else m.virtual_pitch_r

    tooth = angular_width_at(preview.tooth_loop(section, step), radius)
    space = angular_width_at(preview.space_loop(section), radius)

    assert tooth + space == pytest.approx(step, abs=1e-12)


@pytest.mark.parametrize("member", ["pinion", "gear"])
def test_the_tooth_stops_at_the_tip_where_the_cut_loop_runs_past_it(member):
    """The same guarantee `space_loop` carries, restated for the tooth."""
    section = tooth_space_section(GEO, member, "outer")
    loop = preview.tooth_loop(section, developed_pitch(member))

    assert max(radii(loop)) == pytest.approx(section.r_tip)
    assert min(radii(loop)) == pytest.approx(section.r_root)


@pytest.mark.parametrize("member", ["pinion", "gear"])
def test_the_tooth_is_centred_on_half_an_angular_pitch(member):
    """Where the real gear's tooth is - see `gears.preview.tooth_boundary`.

    The space is the thing centred on zero, so the tooth is symmetric about
    `pitch_step / 2` and not about the axis the space is symmetric about.
    """
    step = developed_pitch(member)
    loop = preview.tooth_loop(tooth_space_section(GEO, member, "outer"), step)
    a = angles(loop)

    assert min(a) + max(a) == pytest.approx(step, rel=1e-9)
    assert min(a) > 0.0


@pytest.mark.parametrize("member", ["pinion", "gear"])
@pytest.mark.parametrize("where", [0.15, 0.4, 0.65, 0.9])
def test_the_tooth_begins_exactly_where_the_space_ends(member, where):
    """The flank between them is one curve, so the two loops must share it.

    Checked at four radii up the flank rather than at the pitch circle alone:
    an identity that holds the whole way says the tooth was assembled out of the
    space's own segments and not out of a re-derivation of them that happens to
    agree at one radius. The two ends are left out because the circle is tangent
    to the root arc and to the top land there rather than crossing them.
    """
    m = GEO.member(member)
    step = developed_pitch(member)
    section = tooth_space_section(GEO, member, "outer")
    radius = m.virtual_root_r + where * (section.r_tip - m.virtual_root_r)

    space_lo, space_hi = crossings_at(preview.space_loop(section), radius)
    tooth_lo, tooth_hi = crossings_at(preview.tooth_loop(section, step), radius)

    assert tooth_lo == pytest.approx(space_hi, abs=1e-12)
    assert tooth_hi == pytest.approx(space_lo + step, abs=1e-12)


@pytest.mark.parametrize("member", ["pinion", "gear"])
def test_mis_centring_the_tooth_drops_it_into_its_own_space(member):
    """The failing-on-purpose partner of the two tests above.

    A check that cannot fail proves nothing, and this is the one error the tooth
    boundary can make quietly: rotate it home to angle 0 for tidiness and every
    tooth lands exactly where a space is. The drawing still looks like a gear -
    it is just the wrong half of one - so nothing but an assertion catches it.
    Half a pitch out, the tooth starts half a pitch short of where the space
    ends, which is the whole of the error and not a fraction of it.
    """
    m = GEO.member(member)
    step = developed_pitch(member)
    section = tooth_space_section(GEO, member, "outer")

    _, space_hi = crossings_at(preview.space_loop(section), m.virtual_pitch_r)
    mis_centred = preview.rotate(preview.tooth_loop(section, step), -step / 2.0)
    tooth_lo, _ = crossings_at(mis_centred, m.virtual_pitch_r)

    assert tooth_lo == pytest.approx(space_hi - step / 2.0, abs=1e-12)
    assert tooth_lo < 0.0            # inside the space, which straddles zero


# --- scenes ----------------------------------------------------------------


def test_developed_scene_draws_both_ends_the_cut_and_the_neighbours():
    scene = preview.developed_scene(GEO, "pinion", neighbours=2)
    by_style: dict[str, int] = {}
    for pl in scene.polylines:
        by_style[pl.style] = by_style.get(pl.style, 0) + 1
        assert len(pl.points) >= 2

    assert by_style["outer"] == 1
    assert by_style["inner"] == 1
    # Both spaces flanking the tooth, per end: they are the two cuts that leave
    # it standing, and a tooth-centred view drawing only one looks lopsided.
    assert by_style["cut"] == 4
    assert by_style["neighbour"] == 4    # two teeth either side
    assert scene.legend


def test_developed_neighbours_sit_one_virtual_pitch_apart():
    """In the developed plane the spaces repeat every 2*pi/z_v, not 2*pi/z."""
    m = GEO.pinion
    scene = preview.developed_scene(GEO, "pinion", neighbours=1)
    seed = next(pl for pl in scene.polylines if pl.style == "outer")
    neighbours = [pl for pl in scene.polylines if pl.style == "neighbour"]

    step = TAU / m.virtual_teeth
    offsets = sorted(
        angles(pl.points)[0] - angles(seed.points)[0] for pl in neighbours
    )
    assert offsets == pytest.approx([-step, step])


@pytest.mark.parametrize("member", ["pinion", "gear"])
def test_axial_scene_has_one_space_per_tooth(member):
    m = GEO.member(member)
    scene = preview.axial_scene(GEO, member)
    seeds = [pl for pl in scene.polylines if pl.style in ("outer", "inner")]
    repeats = [pl for pl in scene.polylines if pl.style == "neighbour"]

    assert len(seeds) == 2                      # outer and inner end
    assert len(repeats) == 2 * (m.z - 1)        # both ends, every other tooth


@pytest.mark.parametrize("member", ["pinion", "gear"])
def test_axial_spaces_leave_a_tooth_between_them(member):
    """The mapped space must be narrower than half the real angular pitch.

    This is the factor-of-cos(delta) guard in visual form: the developed
    half-pitch pi/z_v maps to pi/z about the real axis, so if the 1/cos(delta)
    on the angle were missing the spaces would overlap and there would be no
    tooth left between them.
    """
    m = GEO.member(member)
    section = tooth_space_section(GEO, member, "outer")
    loop = preview.to_axial(preview.space_loop(section), section)

    half_pitch = math.pi / m.z
    assert 0.0 < max(angles(loop)) < half_pitch


@pytest.mark.parametrize("member", ["pinion", "gear"])
def test_axial_outer_tip_lands_on_the_outside_diameter(member):
    """R = r_a * cos(delta) == d/2 + a*cos(delta), the catalogue outside radius."""
    m = GEO.member(member)
    section = tooth_space_section(GEO, member, "outer")
    loop = preview.to_axial(preview.space_loop(section), section)

    assert max(radii(loop)) == pytest.approx(m.outside_dia / 2.0)


def test_blank_scene_is_mirrored_about_the_axis():
    scene = preview.blank_scene(GEO, "gear")
    x0, y0, x1, y1 = scene.bounds()

    assert y0 == pytest.approx(-y1)
    assert x0 == pytest.approx(0.0)      # the pitch apex is the origin

    outlines = [pl for pl in scene.polylines if pl.style == "blank"]
    assert len(outlines) == 2
    assert all(pl.closed for pl in outlines)


def test_blank_scene_cone_generators_start_at_the_pitch_apex():
    """Both the pitch and the root cone pass through the origin - Gleason's rule."""
    scene = preview.blank_scene(GEO, "pinion")
    for style in ("pitch", "cone"):
        for pl in (p for p in scene.polylines if p.style == style):
            assert pl.points[0] == pytest.approx((0.0, 0.0))


def test_build_scene_covers_every_advertised_view():
    for key, _label in preview.SCENE_LABELS:
        scene = preview.build_scene(GEO, "pinion", key)
        assert scene.key == key
        assert scene.polylines
    with pytest.raises(ValueError):
        preview.build_scene(GEO, "pinion", "nope")


# --- drawing ---------------------------------------------------------------


def test_draw_scene_draws_every_polyline_inside_the_canvas():
    scene = preview.developed_scene(GEO, "pinion")
    width, height = 400.0, 300.0
    view = preview.View.fit(scene.bounds(), width, height)
    canvas = FakeCanvas()

    preview.draw_scene(canvas, scene, view, legend=False)

    assert len(canvas.lines) == len(scene.polylines)
    for coords, _kwargs in canvas.lines:
        assert all(0.0 <= x <= width for x in coords[0::2])
        assert all(0.0 <= y <= height for y in coords[1::2])


def test_draw_scene_closes_closed_loops():
    """A closed polyline repeats its first point so the canvas line joins up."""
    loop = preview.Polyline([(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)], "outer", closed=True)
    scene = preview.Scene("t", "t", [loop])
    canvas = FakeCanvas()

    preview.draw_scene(canvas, scene, preview.View(1.0, 0.0, 0.0), legend=False)

    coords = canvas.lines[0][0]
    assert len(coords) == 8                     # four points, x and y each
    assert coords[0:2] == coords[6:8]


def test_draw_scene_passes_dash_only_for_dashed_styles():
    scene = preview.Scene(
        "t", "t",
        [
            preview.Polyline([(0.0, 0.0), (1.0, 1.0)], "outer"),
            preview.Polyline([(0.0, 0.0), (1.0, 1.0)], "reference"),
        ],
    )
    canvas = FakeCanvas()
    preview.draw_scene(canvas, scene, preview.View(1.0, 0.0, 0.0), legend=False)

    assert "dash" not in canvas.lines[0][1]
    assert canvas.lines[1][1]["dash"] == preview.STYLES["reference"].dash


def test_legend_draws_one_swatch_and_one_label_per_entry():
    scene = preview.axial_scene(GEO, "pinion")
    canvas = FakeCanvas()
    preview.draw_scene(canvas, scene, preview.View(1.0, 0.0, 0.0), legend=True)

    assert len(canvas.texts) == len(scene.legend)
    assert len(canvas.lines) == len(scene.polylines) + len(scene.legend)


def test_scale_bar_is_a_round_length_and_stays_in_the_corner():
    canvas = FakeCanvas()
    view = preview.View(scale=4.0, ox=0.0, oy=200.0)
    preview.draw_scale_bar(canvas, view, 400.0, 200.0)

    label = canvas.texts[0][2]["text"]
    assert label.endswith(" mm")
    length_mm = float(label.removesuffix(" mm"))
    bar = canvas.lines[0][0]
    assert (bar[2] - bar[0]) == pytest.approx(length_mm * view.scale)
    assert bar[2] < 400.0 and bar[1] < 200.0


@pytest.mark.parametrize("scale", [0.05, 0.3, 1.0, 7.3, 55.0, 900.0])
def test_nice_length_is_a_round_number_near_the_target(scale):
    target = 110.0
    length = preview.nice_length(scale, target)
    raw = target / scale

    assert raw <= length <= 10.0 * raw
    mantissa = length / 10.0 ** math.floor(math.log10(length))
    assert round(mantissa) in (1, 2, 5)
    assert mantissa == pytest.approx(round(mantissa))


# --- export ----------------------------------------------------------------


def dxf_pairs(scene) -> list[tuple[str, str]]:
    lines = preview.dxf_lines(scene)
    assert len(lines) % 2 == 0, "DXF is group code / value pairs, always even"
    return list(zip(lines[::2], lines[1::2]))


def test_dxf_is_a_well_formed_r12_entities_file():
    scene = preview.blank_scene(GEO, "pinion")
    pairs = dxf_pairs(scene)

    assert pairs[0] == ("0", "SECTION")
    assert pairs[-1] == ("0", "EOF")
    assert ("2", "ENTITIES") in pairs
    assert ("1", "AC1009") in pairs
    assert pairs.count(("0", "SECTION")) == pairs.count(("0", "ENDSEC"))
    assert pairs.count(("0", "POLYLINE")) == pairs.count(("0", "SEQEND"))


def test_dxf_writes_every_point_on_a_layer_named_after_its_style():
    scene = preview.developed_scene(GEO, "gear")
    pairs = dxf_pairs(scene)

    expected_points = sum(len(pl.points) for pl in scene.polylines if len(pl.points) >= 2)
    assert sum(1 for pair in pairs if pair == ("0", "VERTEX")) == expected_points

    layers = {value for code, value in pairs if code == "8"}
    assert layers == {pl.style for pl in scene.polylines}


def test_dxf_marks_closed_loops_closed():
    open_pl = preview.Polyline([(0.0, 0.0), (1.0, 0.0)], "outer")
    closed_pl = preview.Polyline([(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)], "outer", True)
    flags = [
        value
        for code, value in dxf_pairs(preview.Scene("t", "t", [open_pl, closed_pl]))
        if code == "70"
    ]
    assert flags == ["4", "0", "1"]      # $INSUNITS, then the two polylines


def test_write_dxf_round_trips_through_the_file(tmp_path):
    scene = preview.axial_scene(GEO, "pinion")
    path = preview.write_dxf(tmp_path / "sub" / "gear.dxf", scene)

    text = path.read_text(encoding="ascii")
    assert text.startswith("0\nSECTION\n")
    assert text.endswith("EOF\n")
    assert text.count("VERTEX") == sum(
        len(pl.points) for pl in scene.polylines if len(pl.points) >= 2
    )


def test_csv_holds_both_ends_and_both_boundaries():
    lines = preview.csv_lines(GEO, "pinion")
    assert lines[0] == preview.CSV_HEADER

    rows = [line.split(",") for line in lines[1:]]
    assert {row[1] for row in rows} == {"outer", "inner"}
    assert {row[2] for row in rows} == {"space", "cut"}
    assert {row[0] for row in rows} == {"pinion"}

    expected = 0
    for end in ("outer", "inner"):
        section = tooth_space_section(GEO, "pinion", end)
        expected += len(preview.space_loop(section)) + len(section.loop_2d)
    assert len(rows) == expected


def test_csv_3d_columns_are_the_developed_points_mapped_onto_the_cone():
    section = tooth_space_section(GEO, "gear", "outer")
    rows = [
        line.split(",")
        for line in preview.csv_lines(GEO, "gear")[1:]
        if line.split(",")[1] == "outer" and line.split(",")[2] == "space"
    ]

    for row, (xd, yd) in zip(rows, preview.space_loop(section)):
        assert (float(row[4]), float(row[5])) == pytest.approx((xd, yd), abs=1e-6)
        expected = to_cone_3d(xd, yd, section.pitch_angle, section.cone_apex_z)
        got = (float(row[6]), float(row[7]), float(row[8]))
        assert got == pytest.approx(expected, abs=1e-6)


def test_write_csv_creates_missing_directories(tmp_path):
    path = preview.write_csv(tmp_path / "a" / "b" / "points.csv", GEO, "pinion")
    assert path.read_text(encoding="utf-8").startswith(preview.CSV_HEADER)


# --- readout ---------------------------------------------------------------


def test_derived_rows_report_the_anchor_case_correctly():
    rows = {row.label: row for row in preview.derived_rows(GEO)}

    assert float(rows["outer cone distance Ao"].pinion) == pytest.approx(46.2385, abs=1e-4)
    assert float(rows["pitch cone angle"].pinion) == pytest.approx(21.5713, abs=1e-4)
    assert float(rows["pitch cone angle"].gear) == pytest.approx(68.4287, abs=1e-4)
    assert rows["teeth"].pinion == "17"
    assert rows["teeth"].gear == "43"


def test_derived_rows_are_all_populated_and_carry_headers():
    rows = preview.derived_rows(GEO)
    headers = [row for row in rows if row.header]

    assert [row.label for row in headers] == ["SET", "MEMBERS"]
    for row in rows:
        if row.header:
            assert row.unit == ""
        else:
            assert row.pinion, f"{row.label} has no pinion value"
