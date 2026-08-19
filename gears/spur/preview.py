"""Spur scenes: what the GUI shows for a spur set, without importing tkinter.

The drawing primitives - Scene, View, styles, DXF - are shared and live in
`gears.preview`. What is here is the three scenes a spur gear is looked at
through, the derived-value table, and the tooth-space CSV.

Two boundaries of a tooth space
-------------------------------
`geometry.tooth_space_section` returns the loop the SOLIDWORKS loft cut needs:
it runs *past* the tip radius so the cut clears the blank. That extension is
correct for cutting and misleading to look at - it draws a notch outside the tip
circle where the real tooth has a top land. So the preview shows both: the
nominal space, closed across the tip circle by `space_loop`, and the cut
boundary as a faint dashed outline behind it.

Showing a helix in two dimensions
---------------------------------
There is no honest single view of a twisted tooth. The transverse scene shows
the profile, which is a true involute and the same at every height; the twist
scene overlays the two end sections so the total twist is visible as the angle
between them, which is the number that actually differs between the members of a
pair. Neither on its own would say what the part is.
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
    SpurSetGeometry,
    ToothSpaceSection,
    blank_outline,
    end_overshoot,
    rim_radius,
    tooth_space_section,
)

__all__ = [
    "STYLES", "Polyline", "Row", "Scene", "Style", "View",
    "arc", "blank_scene", "build_scene", "circle", "csv_lines",
    "derived_rows", "draw_scale_bar", "draw_scene", "dxf_lines",
    "SCENE_BUILDERS", "SCENE_LABELS",
    "nice_length", "rotate", "space_loop", "style_for", "transverse_scene",
    "twist_scene", "write_csv", "write_dxf",
]


def space_loop(section: ToothSpaceSection, tip_points: int = 9) -> list[Point2]:
    """The tooth space closed across the tip circle instead of out past it.

    Takes the section's own segments and swaps the cut-clearance pieces
    (`riser_neg`, the cap, `riser_pos`) for an arc at the tip radius, which is
    where the real tooth's top land is. See the module docstring.

    Works unchanged on a ring gear. It reads the tip radius off the flank's own
    endpoint rather than being told it, so "the arc where the flanks stop" is
    the right answer whether that is the outermost radius or the innermost.
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


# ---------------------------------------------------------------------------
# Scenes
# ---------------------------------------------------------------------------


def transverse_scene(geo: SpurSetGeometry, member: str, neighbours: int = 1) -> Scene:
    """The tooth space in the transverse plane, where the involute lives.

    Unlike the bevel view, this is the real profile and not a development of
    one: a spur gear's transverse section *is* an involute, at the transverse
    module and pressure angle. So this is the view that shows a kinked flank, a
    fillet that failed to fit, or a tip clamped for top land.
    """
    m = geo.member(member)
    section = tooth_space_section(geo, member, z=0.0)

    pitch_step = TAU / m.z
    span = (neighbours + 0.75) * pitch_step

    lines: list[Polyline] = []

    # Reference circles, drawn first so the profiles sit on top of them. The
    # ring's rim goes in too - without it a ring gear's view stops at its root
    # circle and looks like a gear with nothing holding it together.
    lines.append(Polyline(arc(m.pitch_r, -span, span), "pitch"))
    radii = [m.base_r, m.root_r, m.tip_r]
    if m.internal:
        radii.append(rim_radius(geo, member))
    for radius in radii:
        lines.append(Polyline(arc(radius, -span, span), "reference"))

    loop = space_loop(section)
    for i in range(1, neighbours + 1):
        for sign in (-1, 1):
            lines.append(
                Polyline(rotate(loop, sign * i * pitch_step), "neighbour", True)
            )

    # What the loft cut actually sketches, behind the nominal space.
    lines.append(Polyline(list(section.loop_2d), "cut", True))
    lines.append(Polyline(loop, "outer", True))

    kind = "internal ring" if m.internal else member
    return Scene(
        key="transverse",
        title=(
            f"{member} - transverse tooth space, {kind}, {m.z} teeth at "
            f"m_t = {geo.transverse_module:.4f} mm, "
            f"alpha_t = {math.degrees(geo.transverse_pressure_angle):.3f} deg"
        ),
        polylines=lines,
        legend=[
            ("outer", "tooth space"),
            ("neighbour", "adjacent spaces"),
            ("cut", "loft cut boundary"),
            ("pitch", "pitch circle"),
            ("reference", "base / root / tip"),
        ],
    )


def twist_scene(geo: SpurSetGeometry, member: str) -> Scene:
    """Every tooth space of both end faces, looking straight down the axis.

    For straight teeth the two ends coincide exactly and this is the plain end
    view. For helical teeth the angle between them is the total twist, which is
    the thing worth seeing: it differs between the two members of a pair even
    though their helix angles are equal and opposite, because their radii
    differ.

    Drawing every space rather than one also checks that the spaces close
    around 360 degrees, which is the cheapest way to catch a tooth count that
    disagrees with the angular pitch.
    """
    m = geo.member(member)
    b = geo.params.face_width

    front = tooth_space_section(geo, member, z=0.0)
    back = tooth_space_section(geo, member, z=b)

    lines: list[Polyline] = [
        Polyline(circle(m.pitch_r), "pitch"),
        Polyline(circle(m.base_r), "reference"),
        Polyline(circle(m.root_r), "reference"),
        Polyline(circle(m.tip_r), "reference"),
    ]
    if m.internal:
        lines.append(Polyline(circle(rim_radius(geo, member)), "blank"))

    front_loop = space_loop(front)
    back_loop = space_loop(back)
    step = m.angular_pitch
    for i in range(m.z):
        angle = i * step
        lines.append(Polyline(rotate(front_loop, angle), "outer", True))
        if abs(m.twist) > 1e-12:
            lines.append(
                Polyline(rotate(back_loop, angle + m.twist), "inner", True)
            )

    twist_note = (
        f"twist {m.twist_deg:+.3f} deg, {m.hand} hand"
        if abs(m.twist) > 1e-12
        else "straight teeth"
    )
    legend = [("outer", "front face, z = 0")]
    if abs(m.twist) > 1e-12:
        legend.append(("inner", f"back face, z = {b:.3f} mm"))
    legend += [("pitch", "pitch circle"), ("reference", "base / root / tip")]

    return Scene(
        key="twist",
        title=f"{member} - {m.z} tooth spaces down the axis, {twist_note}",
        polylines=lines,
        legend=legend,
    )


def blank_scene(geo: SpurSetGeometry, member: str) -> Scene:
    """The blank's meridian half-section, as the revolve sketch draws it.

    Plotted with R across and z up, which is how the sketch sits on the Top
    Plane. The tooth depth is drawn as a pair of reference lines so it is
    obvious how much of the blank the teeth eat into and how much wall is left
    over the bore.
    """
    p = geo.params
    m = geo.member(member)
    outline = blank_outline(geo, member)

    z_lo = min(z for _, z in outline)
    z_hi = max(z for _, z in outline)
    r_hi = max(r for r, _ in outline)

    lines: list[Polyline] = [
        Polyline([(0.0, z_lo - 2.0), (0.0, z_hi + 2.0)], "axis"),
        Polyline([(m.root_r, 0.0), (m.root_r, p.face_width)], "reference"),
        Polyline([(m.pitch_r, 0.0), (m.pitch_r, p.face_width)], "pitch"),
        Polyline(list(outline), "blank", True),
    ]

    # The cut's axial reach, which is what the end overshoot buys.
    overshoot = end_overshoot(geo)
    lines.append(
        Polyline(
            [(m.tip_r + 1.0, -overshoot), (m.tip_r + 1.0, p.face_width + overshoot)],
            "cut",
        )
    )

    outer = (
        f"rim diameter {2.0 * rim_radius(geo, member):.3f} mm" if m.internal
        else f"outside diameter {m.outside_dia:.3f} mm"
    )
    return Scene(
        key="blank",
        title=(
            f"{member} - blank meridian, R x z, front face at z = 0, {outer}"
        ),
        polylines=lines,
        legend=[
            ("blank", "revolved outline"),
            ("pitch", "pitch radius"),
            ("reference", "root radius"),
            ("cut", "cut reach along z"),
            ("axis", "gear axis"),
        ],
    )


SCENE_BUILDERS = {
    "transverse": transverse_scene,
    "twist": twist_scene,
    "blank": blank_scene,
}

SCENE_LABELS = [
    ("transverse", "Transverse section"),
    ("twist", "Down the axis"),
    ("blank", "Blank section"),
]


def build_scene(geo: SpurSetGeometry, member: str, key: str) -> Scene:
    try:
        return SCENE_BUILDERS[key](geo, member)
    except KeyError:
        raise ValueError(f"unknown scene {key!r}") from None


# ---------------------------------------------------------------------------
# Read-only derived values
# ---------------------------------------------------------------------------


def derived_rows(geo: SpurSetGeometry) -> list[Row]:
    """The readout table: what the inputs came out as."""
    p = geo.params
    a, b = geo.pinion, geo.gear

    axial = "inf" if math.isinf(geo.axial_pitch) else _n(geo.axial_pitch)

    return [
        Row("SET", header=True),
        Row("arrangement", "internal" if p.internal else "external"),
        Row("ratio", _n(p.ratio)),
        Row("centre distance", _n(geo.centre_distance), unit="mm"),
        Row("transverse module", _n(geo.transverse_module), unit="mm"),
        Row(
            "transverse pressure angle",
            _n(math.degrees(geo.transverse_pressure_angle)),
            unit="deg",
        ),
        Row("circular pitch", _n(geo.circular_pitch), unit="mm"),
        Row("axial pitch", axial, unit="mm"),
        Row("whole depth", _n(geo.whole_depth), unit="mm"),
        Row("contact ratio, transverse", _n(geo.transverse_contact_ratio)),
        Row("contact ratio, axial", _n(geo.axial_contact_ratio)),
        Row("contact ratio, total", _n(geo.total_contact_ratio)),
        Row("MEMBERS", "PINION", "RING" if p.internal else "GEAR", header=True),
        Row("teeth", str(a.z), str(b.z)),
        Row("hand", a.hand, b.hand),
        Row("pitch diameter", _n(2.0 * a.pitch_r), _n(2.0 * b.pitch_r), "mm"),
        Row("outside diameter", _n(a.outside_dia), _n(b.outside_dia), "mm"),
        Row("base radius", _n(a.base_r), _n(b.base_r), "mm"),
        # Tip and root are the other way round on a ring, so they are labelled
        # by what they are rather than by which is bigger.
        Row("tip radius", _n(a.tip_r), _n(b.tip_r), "mm"),
        Row("root radius", _n(a.root_r), _n(b.root_r), "mm"),
        *(
            [Row("rim radius", "", _n(rim_radius(geo, "gear")), "mm")]
            if p.internal else []
        ),
        Row("virtual teeth z_v", _n(a.virtual_teeth), _n(b.virtual_teeth)),
        Row("twist over the face", _n(a.twist_deg), _n(b.twist_deg), "deg"),
    ]


# ---------------------------------------------------------------------------
# Export
# ---------------------------------------------------------------------------

CSV_HEADER = "member,section,index,x,y,z"


def csv_lines(geo: SpurSetGeometry, member: str) -> list[str]:
    """Both end sections of the tooth space, as the builder sketches them.

    Both ends rather than one, because for a helical gear they differ - and the
    difference between them *is* the twist, so a file with one of them in it
    would not say what the tooth is.
    """
    overshoot = end_overshoot(geo)
    b = geo.params.face_width

    lines = [CSV_HEADER]
    for label, z in (("front", -overshoot), ("back", b + overshoot)):
        section = tooth_space_section(geo, member, z=z)
        for i, (x, y, za) in enumerate(section.loop_3d()):
            lines.append(f"{member},{label},{i},{x:.6f},{y:.6f},{za:.6f}")
    return lines


def write_csv(path: str | Path, geo: SpurSetGeometry, member: str) -> Path:
    out = Path(path)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text("\n".join(csv_lines(geo, member)) + "\n", encoding="utf-8")
    return out
