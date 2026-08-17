"""Everything the GUI shows, expressed without importing tkinter.

Three jobs live here:

* **Scenes** - a `Scene` is a bundle of named-style polylines in millimetres,
  ready to be drawn. Three of them: the developed (virtual spur) tooth space,
  the same teeth projected down the gear axis, and the blank's meridian
  section.
* **View** - the fit/pan/zoom arithmetic that turns scene millimetres into
  canvas pixels.
* **Export** - the same polylines to DXF, and the tooth-space points to CSV.

`draw_scene` does touch a canvas, but only through `create_line` and
`create_text`, so tkinter is never imported and the whole module stays unit
testable against a recording stub.

Two boundaries of a tooth space
-------------------------------
`geometry.tooth_space_section` returns the loop the SOLIDWORKS loft cut needs:
it runs *past* the tip radius so the cut clears the blank. That extension is
correct for cutting and misleading to look at - it draws a notch outside the
tip circle where the real tooth has a top land. So the preview shows both: the
nominal space, closed across the tip circle by `space_loop`, and the cut
boundary as a faint dashed outline behind it.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path

from .geometry import SetGeometry, ToothSpaceSection, blank_outline, to_cone_3d, tooth_space_section

TAU = 2.0 * math.pi

Point2 = tuple[float, float]


# ---------------------------------------------------------------------------
# Styles
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class Style:
    """How one class of geometry is drawn. Colours suit a light canvas."""

    colour: str
    width: float = 1.0
    dash: tuple[int, ...] | None = None


STYLES: dict[str, Style] = {
    "outer": Style("#1b6fd4", 2.0),
    "inner": Style("#d4611b", 1.6),
    "neighbour": Style("#9db4c8", 1.0),
    "cut": Style("#b9a0d0", 1.0, (3, 3)),
    "pitch": Style("#2f8f4f", 1.2, (7, 3, 2, 3)),
    "reference": Style("#9a9a9a", 1.0, (4, 3)),
    "axis": Style("#7a7a7a", 1.0, (8, 3, 2, 3)),
    "blank": Style("#1b6fd4", 2.0),
    "cone": Style("#c0392b", 1.0, (6, 3)),
    "scale": Style("#4a4a4a", 1.4),
}

DEFAULT_STYLE = Style("#333333")


def style_for(name: str) -> Style:
    return STYLES.get(name, DEFAULT_STYLE)


# ---------------------------------------------------------------------------
# Scene
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class Polyline:
    points: list[Point2]
    style: str = "outer"
    closed: bool = False

    def drawn_points(self) -> list[Point2]:
        """The points as drawn - a closed loop repeats its first point."""
        if self.closed and len(self.points) > 2:
            return [*self.points, self.points[0]]
        return list(self.points)


@dataclass(frozen=True)
class Scene:
    """A drawable 2D view, in millimetres, plus what its colours mean."""

    key: str
    title: str
    polylines: list[Polyline] = field(default_factory=list)
    legend: list[tuple[str, str]] = field(default_factory=list)

    def bounds(self) -> tuple[float, float, float, float]:
        """(x_min, y_min, x_max, y_max) over every point in the scene."""
        xs = [x for pl in self.polylines for x, _ in pl.points]
        ys = [y for pl in self.polylines for _, y in pl.points]
        if not xs:
            return (-1.0, -1.0, 1.0, 1.0)
        return min(xs), min(ys), max(xs), max(ys)


@dataclass(frozen=True)
class View:
    """Scene millimetres to canvas pixels. y is flipped; aspect is preserved."""

    scale: float        # pixels per millimetre
    ox: float           # canvas x of the scene origin
    oy: float           # canvas y of the scene origin

    def to_canvas(self, x: float, y: float) -> Point2:
        return self.ox + x * self.scale, self.oy - y * self.scale

    def from_canvas(self, cx: float, cy: float) -> Point2:
        """The inverse, which is what cursor-anchored zooming needs."""
        return (cx - self.ox) / self.scale, (self.oy - cy) / self.scale

    def flatten(self, points) -> list[float]:
        """Points as the flat x0, y0, x1, y1... list a canvas line wants."""
        flat: list[float] = []
        for x, y in points:
            cx, cy = self.to_canvas(x, y)
            flat.extend((cx, cy))
        return flat

    @classmethod
    def fit(
        cls,
        bounds: tuple[float, float, float, float],
        width: float,
        height: float,
        margin: float = 20.0,
        zoom: float = 1.0,
        pan: Point2 = (0.0, 0.0),
    ) -> "View":
        """Centre `bounds` in a width x height canvas with room to spare."""
        x0, y0, x1, y1 = bounds
        span_x = max(x1 - x0, 1e-6)
        span_y = max(y1 - y0, 1e-6)
        avail_x = max(width - 2.0 * margin, 1.0)
        avail_y = max(height - 2.0 * margin, 1.0)
        scale = min(avail_x / span_x, avail_y / span_y) * max(zoom, 1e-6)

        cx, cy = 0.5 * (x0 + x1), 0.5 * (y0 + y1)
        return cls(
            scale=scale,
            ox=0.5 * width - cx * scale + pan[0],
            oy=0.5 * height + cy * scale + pan[1],
        )


# ---------------------------------------------------------------------------
# Small geometric helpers
# ---------------------------------------------------------------------------


def _polar(r: float, phi: float) -> Point2:
    return r * math.cos(phi), r * math.sin(phi)


def arc(radius: float, a0: float, a1: float, n: int = 64) -> list[Point2]:
    """`n` points along a circular arc about the origin, inclusive of both ends."""
    n = max(2, n)
    return [_polar(radius, a0 + (a1 - a0) * i / (n - 1)) for i in range(n)]


def circle(radius: float, n: int = 181) -> list[Point2]:
    return arc(radius, 0.0, TAU, n)


def rotate(points, angle: float) -> list[Point2]:
    c, s = math.cos(angle), math.sin(angle)
    return [(x * c - y * s, x * s + y * c) for x, y in points]


def _join(pieces) -> list[Point2]:
    """Concatenate polyline pieces, dropping repeated and closing points."""
    loop: list[Point2] = []
    for piece in pieces:
        for pt in piece:
            if not loop or math.dist(loop[-1], pt) > 1e-9:
                loop.append(pt)
    if len(loop) > 1 and math.dist(loop[0], loop[-1]) < 1e-9:
        loop.pop()
    return loop


def space_loop(section: ToothSpaceSection, tip_points: int = 9) -> list[Point2]:
    """The tooth space closed across the tip circle instead of out past it.

    Takes the section's own segments and swaps the three cut-clearance pieces
    (`riser_neg`, `cap`, `riser_pos`) for an arc at the tip radius, which is
    where the real tooth's top land is. See the module docstring.
    """
    seg = section.segments
    flank_pos = seg.get("flank_pos") or []
    if not flank_pos:
        return list(section.loop_2d)

    tip_x, tip_y = flank_pos[0]
    phi_tip = math.atan2(tip_y, tip_x)
    top_land = arc(math.hypot(tip_x, tip_y), -phi_tip, phi_tip, tip_points)

    return _join(
        (
            seg.get("fillet_neg") or [],
            seg.get("flank_neg") or [],
            top_land,
            flank_pos,
            seg.get("fillet_pos") or [],
            seg.get("root") or [],
        )
    )


def to_axial(points, section: ToothSpaceSection) -> list[Point2]:
    """Developed points onto the real cone, then projected down the gear axis."""
    return [
        (x, y)
        for x, y, _ in (
            to_cone_3d(px, py, section.pitch_angle, section.cone_apex_z)
            for px, py in points
        )
    ]


# ---------------------------------------------------------------------------
# Scenes
# ---------------------------------------------------------------------------


def developed_scene(geo: SetGeometry, member: str, neighbours: int = 1) -> Scene:
    """The tooth space in the developed (virtual spur gear) plane.

    This is where the involute actually lives, so it is the view that shows a
    kinked flank, a fillet that failed to fit, or a tip clamped for top land.
    Both end sections are overlaid: they share the same angles and differ only
    by the scale factor k = Ai/Ao.
    """
    m = geo.member(member)
    outer = tooth_space_section(geo, member, "outer")
    inner = tooth_space_section(geo, member, "inner")

    pitch_step = TAU / m.virtual_teeth
    span = (neighbours + 0.75) * pitch_step
    k = geo.section_scale

    lines: list[Polyline] = []

    # Reference arcs, drawn first so the profiles sit on top of them.
    lines.append(Polyline(arc(m.virtual_pitch_r, -span, span), "pitch"))
    for radius in (
        m.virtual_base_r,
        m.virtual_root_r,
        m.virtual_tip_r,
        k * m.virtual_root_r,
        m.virtual_tip_r_inner,
    ):
        lines.append(Polyline(arc(radius, -span, span), "reference"))

    outer_loop = space_loop(outer)
    inner_loop = space_loop(inner)

    for i in range(1, neighbours + 1):
        for sign in (-1, 1):
            lines.append(
                Polyline(rotate(outer_loop, sign * i * pitch_step), "neighbour", True)
            )

    # What the loft cut actually sketches, behind the nominal space.
    lines.append(Polyline(list(outer.loop_2d), "cut", True))
    lines.append(Polyline(list(inner.loop_2d), "cut", True))

    lines.append(Polyline(outer_loop, "outer", True))
    lines.append(Polyline(inner_loop, "inner", True))

    return Scene(
        key="developed",
        title=(
            f"{member} - developed tooth space, z_v = {m.virtual_teeth:.2f} teeth "
            f"at back cone radius {m.virtual_pitch_r:.3f} mm"
        ),
        polylines=lines,
        legend=[
            ("outer", "outer end space"),
            ("inner", "inner end space"),
            ("neighbour", "adjacent spaces"),
            ("cut", "loft cut boundary"),
            ("pitch", "pitch circle"),
            ("reference", "base / root / tip"),
        ],
    )


def axial_scene(geo: SetGeometry, member: str) -> Scene:
    """Every tooth space of both ends, looking straight down the gear axis.

    The developed plane holds z_v teeth; the real gear holds z. The 1/cos(delta)
    on the mapped angle is what converts one into the other, so if that factor
    were wrong the spaces here would not close around 360 degrees - they would
    overlap or leave a gap. That is the whole point of this view.
    """
    m = geo.member(member)
    outer = tooth_space_section(geo, member, "outer")
    inner = tooth_space_section(geo, member, "inner")

    outer_loop = to_axial(space_loop(outer), outer)
    inner_loop = to_axial(space_loop(inner), inner)

    cos_d = math.cos(m.pitch_angle)
    lines: list[Polyline] = [
        Polyline(circle(m.pitch_dia / 2.0), "pitch"),
        Polyline(circle(m.outside_dia / 2.0), "reference"),
        Polyline(circle(geo.inner_cone_dist * math.sin(m.pitch_angle)), "reference"),
        Polyline(circle(m.virtual_tip_r_inner * cos_d), "reference"),
    ]

    for i in range(1, m.z):
        angle = i * m.angular_pitch
        lines.append(Polyline(rotate(outer_loop, angle), "neighbour", True))
        lines.append(Polyline(rotate(inner_loop, angle), "neighbour", True))

    lines.append(Polyline(outer_loop, "outer", True))
    lines.append(Polyline(inner_loop, "inner", True))

    return Scene(
        key="axial",
        title=(
            f"{member} - {m.z} tooth spaces about the axis, "
            f"outside dia {m.outside_dia:.3f} mm"
        ),
        polylines=lines,
        legend=[
            ("outer", "outer end space"),
            ("inner", "inner end space"),
            ("neighbour", "the other spaces"),
            ("pitch", "pitch circle"),
            ("reference", "tip and inner circles"),
        ],
    )


def blank_scene(geo: SetGeometry, member: str) -> Scene:
    """The blank's meridian section, gear axis horizontal, pitch apex at the origin.

    Plotted as (z, R): the axis runs to the right, radius is up, and the
    section is mirrored below the axis so the revolve is easy to read. The
    pitch and root cone generators both start at the origin, which is a direct
    check that the Gleason root cone apexes at the pitch apex.
    """
    m = geo.member(member)
    outline = [(z, R) for R, z in blank_outline(geo, member)]

    z_max = max(z for z, _ in outline)
    r_max = max(R for _, R in outline)

    delta = m.pitch_angle
    pitch_end = (geo.outer_cone_dist * math.cos(delta), geo.outer_cone_dist * math.sin(delta))
    root_end = (
        geo.outer_cone_dist * math.cos(m.root_angle),
        geo.outer_cone_dist * math.sin(m.root_angle),
    )

    lines: list[Polyline] = [
        Polyline([(0.0, 0.0), (1.05 * z_max, 0.0)], "axis"),
        Polyline([(0.0, 0.0), pitch_end], "pitch"),
        Polyline([(0.0, 0.0), (pitch_end[0], -pitch_end[1])], "pitch"),
        Polyline([(0.0, 0.0), root_end], "cone"),
        Polyline([(0.0, 0.0), (root_end[0], -root_end[1])], "cone"),
        Polyline(outline, "blank", True),
        Polyline([(z, -R) for z, R in outline], "blank", True),
    ]

    return Scene(
        key="blank",
        title=(
            f"{member} - blank section, crown at z = {m.crown_to_apex:.3f} mm, "
            f"max radius {r_max:.3f} mm"
        ),
        polylines=lines,
        legend=[
            ("blank", "revolved outline"),
            ("pitch", "pitch cone"),
            ("cone", "root cone"),
            ("axis", "gear axis"),
        ],
    )


SCENE_BUILDERS = {
    "developed": developed_scene,
    "axial": axial_scene,
    "blank": blank_scene,
}

SCENE_LABELS = [
    ("developed", "Developed section"),
    ("axial", "Down the axis"),
    ("blank", "Blank section"),
]


def build_scene(geo: SetGeometry, member: str, key: str) -> Scene:
    try:
        builder = SCENE_BUILDERS[key]
    except KeyError:
        raise ValueError(f"unknown scene {key!r}") from None
    return builder(geo, member)


# ---------------------------------------------------------------------------
# Drawing
# ---------------------------------------------------------------------------


def nice_length(px_per_mm: float, target_px: float = 110.0) -> float:
    """A round number of millimetres about `target_px` wide, for the scale bar."""
    raw = target_px / max(px_per_mm, 1e-9)
    exponent = math.floor(math.log10(raw)) if raw > 0 else 0
    for mult in (1.0, 2.0, 5.0):
        candidate = mult * 10.0**exponent
        if candidate >= raw:
            return candidate
    return 10.0 ** (exponent + 1)


def draw_scene(canvas, scene: Scene, view: View, legend: bool = True) -> None:
    """Draw a scene onto anything with `create_line` and `create_text`.

    The caller clears the canvas and decides the view; this only draws, which
    keeps it usable from a test with a stub canvas.
    """
    for pl in scene.polylines:
        pts = pl.drawn_points()
        if len(pts) < 2:
            continue
        st = style_for(pl.style)
        kwargs = {"fill": st.colour, "width": st.width}
        if st.dash:
            kwargs["dash"] = st.dash
        canvas.create_line(*view.flatten(pts), **kwargs)

    if legend:
        _draw_legend(canvas, scene)


def _draw_legend(canvas, scene: Scene, x: float = 12.0, y: float = 12.0) -> None:
    for i, (style_name, label) in enumerate(scene.legend):
        st = style_for(style_name)
        row_y = y + 14.0 * i + 6.0
        kwargs = {"fill": st.colour, "width": max(st.width, 1.6)}
        if st.dash:
            kwargs["dash"] = st.dash
        canvas.create_line(x, row_y, x + 22.0, row_y, **kwargs)
        canvas.create_text(
            x + 28.0, row_y, text=label, anchor="w", fill="#444444", font=("TkDefaultFont", 7)
        )


def draw_scale_bar(canvas, view: View, width: float, height: float) -> None:
    """A round-number scale bar in the bottom right corner."""
    length_mm = nice_length(view.scale)
    length_px = length_mm * view.scale
    x1, y1 = width - 16.0, height - 16.0
    x0 = x1 - length_px
    st = style_for("scale")
    canvas.create_line(x0, y1, x1, y1, fill=st.colour, width=st.width)
    canvas.create_line(x0, y1 - 4.0, x0, y1 + 4.0, fill=st.colour, width=st.width)
    canvas.create_line(x1, y1 - 4.0, x1, y1 + 4.0, fill=st.colour, width=st.width)
    canvas.create_text(
        0.5 * (x0 + x1),
        y1 - 8.0,
        text=f"{length_mm:g} mm",
        anchor="s",
        fill=st.colour,
        font=("TkDefaultFont", 7),
    )


# ---------------------------------------------------------------------------
# Read-only derived values
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class Row:
    """One line of the derived-value readout."""

    label: str
    pinion: str = ""
    gear: str = ""
    unit: str = ""
    header: bool = False


def _n(value: float, places: int = 4) -> str:
    return f"{value:.{places}f}"


def derived_rows(geo: SetGeometry) -> list[Row]:
    """The whole read-only readout, in display order."""
    p = geo.params
    a, b = geo.pinion, geo.gear

    return [
        Row("SET", header=True),
        Row("ratio", _n(p.ratio)),
        Row("outer cone distance Ao", _n(geo.outer_cone_dist), unit="mm"),
        Row("mean cone distance Am", _n(geo.mean_cone_dist), unit="mm"),
        Row("inner cone distance Ai", _n(geo.inner_cone_dist), unit="mm"),
        Row("section scale k = Ai/Ao", _n(geo.section_scale)),
        Row("circular pitch", _n(geo.circular_pitch), unit="mm"),
        Row("working depth", _n(geo.working_depth), unit="mm"),
        Row("whole depth", _n(geo.whole_depth), unit="mm"),
        Row("clearance", _n(geo.clearance), unit="mm"),
        Row("MEMBERS", "pinion", "gear", header=True),
        Row("teeth", str(a.z), str(b.z)),
        Row("pitch cone angle", _n(a.pitch_angle_deg), _n(b.pitch_angle_deg), "deg"),
        Row("face cone angle", _n(a.face_angle_deg), _n(b.face_angle_deg), "deg"),
        Row("root cone angle", _n(a.root_angle_deg), _n(b.root_angle_deg), "deg"),
        Row(
            "addendum angle",
            _n(math.degrees(a.addendum_angle)),
            _n(math.degrees(b.addendum_angle)),
            "deg",
        ),
        Row(
            "dedendum angle",
            _n(math.degrees(a.dedendum_angle)),
            _n(math.degrees(b.dedendum_angle)),
            "deg",
        ),
        Row("pitch diameter", _n(a.pitch_dia), _n(b.pitch_dia), "mm"),
        Row("outside diameter", _n(a.outside_dia), _n(b.outside_dia), "mm"),
        Row("addendum", _n(a.addendum), _n(b.addendum), "mm"),
        Row("dedendum", _n(a.dedendum), _n(b.dedendum), "mm"),
        Row("virtual teeth z_v", _n(a.virtual_teeth), _n(b.virtual_teeth)),
        Row("back cone distance", _n(a.virtual_pitch_r), _n(b.virtual_pitch_r), "mm"),
        Row("tip radius, outer", _n(a.virtual_tip_r), _n(b.virtual_tip_r), "mm"),
        Row(
            "tip radius, inner",
            _n(a.virtual_tip_r_inner),
            _n(b.virtual_tip_r_inner),
            "mm",
        ),
        Row("crown to apex", _n(a.crown_to_apex), _n(b.crown_to_apex), "mm"),
        Row("outer root radius", _n(a.outer_root_radius), _n(b.outer_root_radius), "mm"),
        Row("outer root to apex", _n(a.root_to_apex), _n(b.root_to_apex), "mm"),
        Row("mounting distance", _n(a.mounting_distance), _n(b.mounting_distance), "mm"),
    ]


# ---------------------------------------------------------------------------
# Export
# ---------------------------------------------------------------------------

# Group-code pairs for a minimal AutoCAD R12 file. R12 has no LWPOLYLINE, and
# POLYLINE / VERTEX / SEQEND is the form every CAD package still reads.
_DXF_HEADER = [
    (0, "SECTION"), (2, "HEADER"),
    (9, "$ACADVER"), (1, "AC1009"),
    (9, "$INSUNITS"), (70, 4),          # 4 = millimetres
    (0, "ENDSEC"),
    (0, "SECTION"), (2, "ENTITIES"),
]


def dxf_lines(scene: Scene) -> list[str]:
    """The scene as DXF text lines: one group code and one value per line."""
    pairs: list[tuple[int, object]] = list(_DXF_HEADER)

    for pl in scene.polylines:
        pts = pl.points if pl.closed else pl.drawn_points()
        if len(pts) < 2:
            continue
        layer = pl.style
        pairs += [
            (0, "POLYLINE"), (8, layer),
            (66, 1),                        # vertices follow
            (70, 1 if pl.closed else 0),
            (10, 0.0), (20, 0.0), (30, 0.0),
        ]
        for x, y in pts:
            pairs += [(0, "VERTEX"), (8, layer), (10, x), (20, y), (30, 0.0)]
        pairs += [(0, "SEQEND"), (8, layer)]

    pairs += [(0, "ENDSEC"), (0, "EOF")]

    lines: list[str] = []
    for code, value in pairs:
        lines.append(str(code))
        lines.append(f"{value:.6f}" if isinstance(value, float) else str(value))
    return lines


def write_dxf(path: str | Path, scene: Scene) -> Path:
    """Write a scene to DXF, one layer per style name."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(dxf_lines(scene)) + "\n", encoding="ascii")
    return path


CSV_HEADER = "member,end,loop,index,x_dev,y_dev,x,y,z"


def csv_lines(geo: SetGeometry, member: str) -> list[str]:
    """Both end sections as CSV rows: developed 2D and real 3D coordinates.

    Each section appears twice: `loop=space` is the nominal tooth space closed
    at the tip circle, `loop=cut` is the boundary the loft cut uses, which runs
    past the tip and past both ends of the face width.
    """
    lines = [CSV_HEADER]
    for end in ("outer", "inner"):
        section = tooth_space_section(geo, member, end)
        for loop_name, loop in (("space", space_loop(section)), ("cut", section.loop_2d)):
            for i, (xd, yd) in enumerate(loop):
                x, y, z = to_cone_3d(xd, yd, section.pitch_angle, section.cone_apex_z)
                lines.append(
                    f"{member},{end},{loop_name},{i},"
                    f"{xd:.6f},{yd:.6f},{x:.6f},{y:.6f},{z:.6f}"
                )
    return lines


def write_csv(path: str | Path, geo: SetGeometry, member: str) -> Path:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(csv_lines(geo, member)) + "\n", encoding="utf-8")
    return path
