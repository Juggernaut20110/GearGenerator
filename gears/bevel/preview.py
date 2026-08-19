"""Bevel scenes: what the GUI shows for a bevel set, without importing tkinter.

The drawing primitives - Scene, View, styles, DXF - are shared and live in
`gears.preview`. What is here is the three scenes a bevel gear is looked at
through, the derived-value table, and the tooth-space CSV.

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
from pathlib import Path

from ..preview import (
    STYLES,
    TAU,
    Point2,
    Polyline,
    Row,
    Scene,
    Style,
    View,
    arc,
    circle,
    draw_scale_bar,
    draw_scene,
    dxf_lines,
    fmt as _n,
    join_pieces as _join,
    nice_length,
    rotate,
    style_for,
    write_dxf,
)
from .geometry import (
    SetGeometry,
    ToothSpaceSection,
    blank_outline,
    phase_at_cone_distance,
    to_cone_3d,
    tooth_space_section,
)

__all__ = [
    "STYLES", "Polyline", "Row", "Scene", "Style", "View",
    "arc", "axial_scene", "blank_scene", "build_scene", "circle",
    "csv_lines", "derived_rows", "developed_scene", "draw_scale_bar",
    "draw_scene", "dxf_lines", "nice_length", "rotate", "space_loop",
    "style_for", "to_axial", "trace_scene", "write_csv", "write_dxf",
]


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
    """Developed points onto the real cone, then projected down the gear axis.

    The section's phase comes along, so a spiral gear's two ends land where the
    trace actually puts them - which is what makes the angle between them in
    `axial_scene` the spiral's sweep rather than nothing at all.
    """
    return [
        (x, y)
        for x, y, _ in (
            to_cone_3d(
                px, py, section.pitch_angle, section.cone_apex_z, section.phase
            )
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

    sweep_note = ""
    if geo.trace is not None:
        sweep = math.degrees(
            phase_at_cone_distance(geo, member, geo.outer_cone_dist)
            - phase_at_cone_distance(geo, member, geo.inner_cone_dist)
        )
        sweep_note = f", spiral sweep {sweep:+.3f} deg"

    return Scene(
        key="axial",
        title=(
            f"{member} - {m.z} tooth spaces about the axis, "
            f"outside dia {m.outside_dia:.3f} mm{sweep_note}"
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


def trace_scene(geo: SetGeometry, member: str, neighbours: int = 2) -> Scene:
    """The tooth trace as an arc in the generating crown gear's plane.

    The one view that shows what the cutter radius did. Everything else looks at
    a section; this looks at the curve those sections are strung along, in the
    plane where it is a plain circular arc rather than something wrapped onto a
    cone.

    Both members' traces are the *same* curve here - that is the whole
    construction - so this scene does not depend on which member is selected,
    and says so in its title rather than pretending otherwise. What differs
    between them is only how far round their own axis that curve carries the
    tooth, which is what `axial_scene` shows.

    A radial line is drawn alongside for reference: that is the trace a straight
    bevel gear has, and the gap between the two at the toe and the heel is the
    spiral.
    """
    trace = geo.trace
    a_i, a_m, a_o = geo.inner_cone_dist, geo.mean_cone_dist, geo.outer_cone_dist

    def polar_pt(radius: float, angle: float) -> Point2:
        return radius * math.cos(angle), radius * math.sin(angle)

    def trace_points(offset: float, n: int = 81) -> list[Point2]:
        pts = []
        for i in range(n):
            a = a_i + (a_o - a_i) * i / (n - 1)
            angle = offset + (
                trace.sign * trace.theta_at(a) if trace is not None else 0.0
            )
            pts.append(polar_pt(a, angle))
        return pts

    # The crown gear carries as many teeth as it has pitch; its angular pitch is
    # the arc length of one pitch over the mean cone distance.
    crown_pitch = geo.circular_pitch * (a_m / a_o) / a_m
    span = (neighbours + 0.75) * crown_pitch

    lines: list[Polyline] = [
        Polyline(arc(a_m, -span, span), "pitch"),
        Polyline(arc(a_i, -span, span), "reference"),
        Polyline(arc(a_o, -span, span), "reference"),
        # What a straight bevel gear's trace is, for comparison.
        Polyline([polar_pt(a_i, 0.0), polar_pt(a_o, 0.0)], "cut"),
    ]

    for i in range(1, neighbours + 1):
        for sign in (-1, 1):
            lines.append(
                Polyline(trace_points(sign * i * crown_pitch), "neighbour")
            )
    lines.append(Polyline(trace_points(0.0), "outer"))

    if trace is None:
        title = "straight teeth - the trace is the cone generator itself"
    else:
        title = (
            f"tooth trace in the crown plane, {trace.cutter_radius:.3f} mm cutter "
            f"at {math.degrees(abs(trace.spiral_angle_at(a_i))):.2f} / "
            f"{math.degrees(abs(trace.spiral_angle_at(a_m))):.2f} / "
            f"{math.degrees(abs(trace.spiral_angle_at(a_o))):.2f} deg "
            "(toe / mean / heel)"
        )

    return Scene(
        key="trace",
        title=f"both members - {title}",
        polylines=lines,
        legend=[
            ("outer", "tooth trace"),
            ("neighbour", "adjacent traces"),
            ("cut", "a straight tooth, for comparison"),
            ("pitch", "mean cone distance Am"),
            ("reference", "toe Ai and heel Ao"),
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
    "trace": trace_scene,
    "blank": blank_scene,
}

SCENE_LABELS = [
    ("developed", "Developed section"),
    ("axial", "Down the axis"),
    ("trace", "Tooth trace"),
    ("blank", "Blank section"),
]


def build_scene(geo: SetGeometry, member: str, key: str) -> Scene:
    try:
        builder = SCENE_BUILDERS[key]
    except KeyError:
        raise ValueError(f"unknown scene {key!r}") from None
    return builder(geo, member)


# ---------------------------------------------------------------------------
# Read-only derived values
# ---------------------------------------------------------------------------


def _trace_rows(geo: SetGeometry) -> list[Row]:
    """The spiral block of the readout. Empty for a straight set.

    Empty rather than a row of dashes: a straight bevel gear does not have a
    cutter radius that happens to be blank, it has no trace at all, and a table
    that shows the field anyway invites someone to fill it in.
    """
    trace = geo.trace
    if trace is None:
        return []
    return [
        Row("SPIRAL", header=True),
        Row("hand (pinion)", geo.params.hand),
        Row("mean spiral angle", _n(abs(geo.params.spiral_angle)), unit="deg"),
        Row(
            "spiral angle at toe",
            _n(math.degrees(abs(trace.spiral_angle_at(geo.inner_cone_dist)))),
            unit="deg",
        ),
        Row(
            "spiral angle at heel",
            _n(math.degrees(abs(trace.spiral_angle_at(geo.outer_cone_dist)))),
            unit="deg",
        ),
        Row("cutter radius", _n(trace.cutter_radius), unit="mm"),
        Row("cutter offset rho", _n(trace.centre_distance), unit="mm"),
        Row("contact ratio, face", _n(geo.face_contact_ratio)),
    ]


def _sweep_rows(geo: SetGeometry) -> list[Row]:
    """How far each member's tooth carries round its own axis. Straight: none."""
    if geo.trace is None:
        return []

    def sweep(member: str) -> float:
        """Radians, signed - the direction is as much of the answer as the size."""
        return phase_at_cone_distance(
            geo, member, geo.outer_cone_dist
        ) - phase_at_cone_distance(geo, member, geo.inner_cone_dist)

    return [
        Row(
            "sweep over the face",
            _n(math.degrees(sweep("pinion"))),
            _n(math.degrees(sweep("gear"))),
            "deg",
        ),
        # Against each member's own pitch, which is the number that says whether
        # the tooth wraps past its neighbour's start - and they differ, because
        # the two members have different tooth counts as well as different sweeps.
        Row(
            "as angular pitches",
            _n(abs(sweep("pinion")) / geo.pinion.angular_pitch),
            _n(abs(sweep("gear")) / geo.gear.angular_pitch),
        ),
    ]


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
        Row("min root thickness", _n(p.min_root_thickness), unit="mm"),
        *_trace_rows(geo),
        Row("MEMBERS", "pinion", "gear", header=True),
        Row("teeth", str(a.z), str(b.z)),
        *_sweep_rows(geo),
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
