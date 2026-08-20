"""Drawing primitives shared by every gear type's preview.

Three jobs live here:

* **Scene** - a bundle of named-style polylines in millimetres, ready to be
  drawn. Each gear type builds its own scenes out of these pieces.
* **View** - the fit/pan/zoom arithmetic that turns scene millimetres into
  canvas pixels.
* **Export** - the same polylines to DXF, and the table format the readout uses.

`draw_scene` does touch a canvas, but only through `create_line` and
`create_text`, so tkinter is never imported and the whole module stays unit
testable against a recording stub.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path

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


def polar(r: float, phi: float) -> Point2:
    return r * math.cos(phi), r * math.sin(phi)


def arc(radius: float, a0: float, a1: float, n: int = 64) -> list[Point2]:
    """`n` points along a circular arc about the origin, inclusive of both ends."""
    n = max(2, n)
    return [polar(radius, a0 + (a1 - a0) * i / (n - 1)) for i in range(n)]


def circle(radius: float, n: int = 181) -> list[Point2]:
    return arc(radius, 0.0, TAU, n)


def rotate(points, angle: float) -> list[Point2]:
    c, s = math.cos(angle), math.sin(angle)
    return [(x * c - y * s, x * s + y * c) for x, y in points]


def join_pieces(pieces) -> list[Point2]:
    """Concatenate polyline pieces, dropping repeated and closing points."""
    loop: list[Point2] = []
    for piece in pieces:
        for pt in piece:
            if not loop or math.dist(loop[-1], pt) > 1e-9:
                loop.append(pt)
    if len(loop) > 1 and math.dist(loop[0], loop[-1]) < 1e-9:
        loop.pop()
    return loop


# ---------------------------------------------------------------------------
# The two boundaries a section can be drawn as
# ---------------------------------------------------------------------------
#
# `involute.tooth_space_loop` returns the tooth **space**, because that is the
# shape the SOLIDWORKS loft cut sketches and circular-patterns z times. Both
# functions below take that same named-segments dict; they differ only in which
# closed boundary they assemble out of it, and neither knows or cares which gear
# type built it.


def space_boundary(segments: dict, tip_points: int = 9) -> list[Point2]:
    """The tooth space closed across the tip circle instead of out past it.

    Swaps the cut-clearance pieces (`riser_neg`, the cap, `riser_pos`) for an
    arc at the tip radius, which is where the real tooth's top land is. The cut
    profile runs past the tip on purpose so it clears the blank; drawn as-is
    that is a notch outside the tip circle where there is no notch.

    Works unchanged on a ring gear. It reads the tip radius off the flank's own
    endpoint rather than being told it, so "the arc where the flanks stop" is
    the right answer whether that is the outermost radius or the innermost.
    """
    flank_pos = segments.get("flank_pos") or []
    if not flank_pos:
        return []

    tip_x, tip_y = flank_pos[0]
    phi_tip = math.atan2(tip_y, tip_x)
    top_land = arc(math.hypot(tip_x, tip_y), -phi_tip, phi_tip, tip_points)

    return join_pieces(
        (
            segments.get("fillet_neg") or [],
            segments.get("flank_neg") or [],
            top_land,
            flank_pos,
            segments.get("fillet_pos") or [],
            segments.get("root") or [],
        )
    )


def tooth_boundary(
    segments: dict, pitch_step: float, arc_points: int = 9
) -> list[Point2]:
    """The TOOTH between this space and the next one, as a closed loop.

    A tooth is the complement of two adjacent spaces, so it needs one thing the
    section does not carry: the angular pitch to the next space. Each type
    supplies its own - `2*pi/z` for a spur gear, `2*pi/z_v` for a bevel gear in
    the developed plane, where the spaces repeat at the *virtual* tooth count.

    The loop runs counter-clockwise, out of one space and back into the next:

        fillet_pos, flank_pos     reversed - root up to tip, positive side
        top land                  an arc at the tip radius, across the tooth
        flank_neg, fillet_neg     reversed and turned a pitch - tip back to root
        root                      an arc at the root radius, back to the start

    `riser_neg`, the cap and `riser_pos` are skipped for the same reason
    `space_boundary` skips them: they are cut clearance standing outside the
    blank, not tooth.

    **The tooth is centred on half an angular pitch, not on zero.** The space is
    the thing centred on zero - every generator in `involute` builds it that way
    - so the tooth sits `pitch_step / 2` along, which is where the real gear's
    tooth is. Do not rotate it home to zero for tidiness: every scene repeats
    its loop by `i * angular_pitch`, so a tooth centred on zero would be drawn
    exactly where the spaces are. Nothing would look obviously wrong - a ring of
    teeth is a ring of teeth - but the planetary train's meshes would be drawn
    interlocking when they are not. There is a test that mis-centres it on
    purpose and counts the overlap.

    Read the radii off the segments rather than off the section, like
    `space_boundary` does, and a ring gear needs no special case: its tip is
    simply its innermost radius and the arcs still span the same angles.
    """
    flank_pos = segments.get("flank_pos") or []
    root = segments.get("root") or []
    if not flank_pos or not root:
        return []

    def turned(pts: list[Point2]) -> list[Point2]:
        """The far side of the tooth: the next space's piece, a pitch along."""
        return rotate(pts[::-1], pitch_step)

    tip_x, tip_y = flank_pos[0]
    r_tip, phi_tip = math.hypot(tip_x, tip_y), math.atan2(tip_y, tip_x)
    root_x, root_y = root[0]
    r_root, phi_root = math.hypot(root_x, root_y), math.atan2(root_y, root_x)

    return join_pieces(
        (
            (segments.get("fillet_pos") or [])[::-1],
            flank_pos[::-1],
            arc(r_tip, phi_tip, pitch_step - phi_tip, arc_points),
            turned(segments.get("flank_neg") or []),
            turned(segments.get("fillet_neg") or []),
            arc(r_root, pitch_step - phi_root, phi_root, arc_points),
        )
    )


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
        draw_legend(canvas, scene)


def draw_legend(canvas, scene: Scene, x: float = 12.0, y: float = 12.0) -> None:
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
    """One line of the derived-value readout.

    Two value columns, because a gear pair has two members - and a third that a
    pair simply never sets. A planetary train has three members, and rather than
    give it a readout of its own it fills in `third`; the window then shows the
    extra column only for a type that populates it, so the pair-shaped types are
    completely unaffected.
    """

    label: str
    pinion: str = ""
    gear: str = ""
    unit: str = ""
    header: bool = False
    third: str = ""

    @property
    def values(self) -> tuple[str, ...]:
        """The value columns this row actually carries, in display order."""
        return (self.pinion, self.gear, self.third)


def fmt(value: float, places: int = 4) -> str:
    return f"{value:.{places}f}"


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


