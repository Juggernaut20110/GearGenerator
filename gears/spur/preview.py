"""Spur scenes: what the GUI shows for a spur set, without importing tkinter.

The drawing primitives - Scene, View, styles, DXF - are shared and live in
`gears.preview`. What is here is the three scenes a spur gear is looked at
through, the derived-value table, and the tooth-space CSV.

The tooth, and the space it is cut out of
-----------------------------------------
`geometry.tooth_space_section` returns the loop the SOLIDWORKS loft cut needs:
the tooth **space**, run *past* the tip radius so the cut clears the blank.
That is what gets cut and it is not what anyone wants to look at - a gear drawn
as its spaces is a ring of holes. So the scenes draw `tooth_loop`, the tooth
between one space and the next, and keep the cut boundary behind it as a faint
dashed outline: the two cuts either side are what leave the tooth.

Both boundaries are assembled by `gears.preview` out of the same named segments,
and `space_loop` is still here because the CSV export and the tests want the
shape the builder actually sketches.

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
    nice_length,
    rotate,
    space_boundary,
    style_for,
    tooth_boundary,
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
    "nice_length", "rotate", "space_loop", "style_for", "tooth_loop",
    "transverse_scene", "twist_scene", "write_csv", "write_dxf",
]


def space_loop(section: ToothSpaceSection, tip_points: int = 9) -> list[Point2]:
    """The tooth space closed across the tip circle instead of out past it."""
    return space_boundary(section.segments, tip_points) or list(section.loop_2d)


def tooth_loop(section: ToothSpaceSection, pitch_step: float) -> list[Point2]:
    """The tooth between this space and the next, centred on half a pitch.

    `pitch_step` is the member's angular pitch, `2*pi/z`. See
    `gears.preview.tooth_boundary` for why the tooth is not centred on zero.
    """
    return tooth_boundary(section.segments, pitch_step) or list(section.loop_2d)


# ---------------------------------------------------------------------------
# Scenes
# ---------------------------------------------------------------------------


def transverse_scene(geo: SpurSetGeometry, member: str, neighbours: int = 1) -> Scene:
    """The tooth in the transverse plane, where the involute lives.

    Unlike the bevel view, this is the real profile and not a development of
    one: a spur gear's transverse section *is* an involute, at the transverse
    module and pressure angle. So this is the view that shows a kinked flank, a
    fillet that failed to fit, or a tip clamped for top land.

    Everything is centred on the tooth rather than on angle 0, because the tooth
    sits half a pitch along from the space the generator builds.
    """
    m = geo.member(member)
    section = tooth_space_section(geo, member, z=0.0)

    pitch_step = TAU / m.z
    half = pitch_step / 2.0
    span = (neighbours + 0.75) * pitch_step

    lines: list[Polyline] = []

    # Reference circles, drawn first so the profiles sit on top of them. The
    # ring's rim goes in too - without it a ring gear's view stops at its root
    # circle and looks like a gear with nothing holding it together.
    lines.append(
        Polyline(arc(m.reference_r, half - span, half + span), "pitch")
    )
    lines.append(
        Polyline(arc(m.working_r, half - span, half + span), "working")
    )
    radii = [m.base_r, m.root_r, m.tip_r]
    if m.internal:
        radii.append(rim_radius(geo, member))
    for radius in radii:
        lines.append(Polyline(arc(radius, half - span, half + span), "reference"))

    loop = tooth_loop(section, pitch_step)
    for i in range(1, neighbours + 1):
        for sign in (-1, 1):
            lines.append(
                Polyline(rotate(loop, sign * i * pitch_step), "neighbour", True)
            )

    # What the loft cut actually sketches, behind the tooth: the two spaces
    # either side of it, which are the two cuts that leave it standing.
    cut = list(section.loop_2d)
    lines.append(Polyline(cut, "cut", True))
    lines.append(Polyline(rotate(cut, pitch_step), "cut", True))
    lines.append(Polyline(loop, "outer", True))

    kind = "internal ring" if m.internal else member
    return Scene(
        key="transverse",
        title=(
            f"{member} - transverse tooth, {kind}, {m.z} teeth at "
            f"m_t = {geo.transverse_module:.4f} mm, "
            f"alpha_t = {math.degrees(geo.transverse_pressure_angle):.3f} deg"
        ),
        polylines=lines,
        legend=[
            ("outer", "tooth"),
            ("neighbour", "adjacent teeth"),
            ("cut", "loft cut boundary"),
            ("pitch", "reference pitch circle"),
            ("working", "working pitch circle"),
            ("reference", "base / root / tip"),
        ],
    )


def twist_scene(geo: SpurSetGeometry, member: str) -> Scene:
    """Every tooth of both end faces, looking straight down the axis.

    For straight teeth the two ends coincide exactly and this is the plain end
    view. For helical teeth the angle between them is the total twist, which is
    the thing worth seeing: it differs between the two members of a pair even
    though their helix angles are equal and opposite, because their radii
    differ.

    Drawing every tooth rather than one also checks that they close around 360
    degrees, which is the cheapest way to catch a tooth count that disagrees
    with the angular pitch - the root arcs of consecutive teeth join into the
    root circle only if they do.
    """
    m = geo.member(member)
    b = geo.params.face_width

    front = tooth_space_section(geo, member, z=0.0)
    back = tooth_space_section(geo, member, z=b)

    lines: list[Polyline] = [
        Polyline(circle(m.reference_r), "pitch"),
        Polyline(circle(m.working_r), "working"),
        Polyline(circle(m.base_r), "reference"),
        Polyline(circle(m.root_r), "reference"),
        Polyline(circle(m.tip_r), "reference"),
    ]
    if m.internal:
        lines.append(Polyline(circle(rim_radius(geo, member)), "blank"))

    step = m.angular_pitch
    front_loop = tooth_loop(front, step)
    back_loop = tooth_loop(back, step)
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
    legend += [
        ("pitch", "reference pitch circle"),
        ("working", "working pitch circle"),
        ("reference", "base / root / tip"),
    ]

    return Scene(
        key="twist",
        title=f"{member} - {m.z} teeth down the axis, {twist_note}",
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
        Polyline([(m.reference_r, 0.0), (m.reference_r, p.face_width)], "pitch"),
        Polyline([(m.working_r, 0.0), (m.working_r, p.face_width)], "working"),
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
            ("pitch", "reference pitch radius"),
            ("working", "working pitch radius"),
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
        Row(
            "reference centre distance a",
            _n(geo.reference_centre_distance), unit="mm"
        ),
        Row("transverse module m_t", _n(geo.transverse_module), unit="mm"),
        Row(
            "transverse pressure angle alpha_t",
            _n(math.degrees(geo.reference_pressure_angle)),
            unit="deg",
        ),
        Row("circular pitch (transverse)", _n(geo.circular_pitch), unit="mm"),
        Row("axial pitch", axial, unit="mm"),
        Row("whole depth", _n(geo.whole_depth), unit="mm"),
        Row("WORKING / OPERATING GEOMETRY", header=True),
        Row("working centre distance a_w", _n(geo.working_centre_distance), unit="mm"),
        Row("centre-distance modification y", _n(geo.centre_distance_modification), unit="m_n"),
        Row(
            "working pressure angle alpha_wt",
            _n(math.degrees(geo.working_pressure_angle)),
            unit="deg",
        ),
        Row("epsilon_alpha (transverse)", _n(geo.transverse_contact_ratio)),
        Row("epsilon_beta (overlap)", _n(geo.overlap_ratio)),
        Row("epsilon_gamma (total)", _n(geo.total_contact_ratio)),
        Row("MEMBERS", "PINION", "RING" if p.internal else "GEAR", header=True),
        Row("teeth", str(a.z), str(b.z)),
        Row("hand", a.hand, b.hand),
        Row("profile shift x", _n(a.profile_shift), _n(b.profile_shift)),
        Row("reference diameter d", _n(a.reference_d), _n(b.reference_d), "mm"),
        Row("base diameter d_b", _n(a.base_d), _n(b.base_d), "mm"),
        Row("working pitch diameter d_w", _n(a.working_d), _n(b.working_d), "mm"),
        Row("tip diameter d_a", _n(a.tip_d), _n(b.tip_d), "mm"),
        Row("root diameter d_f", _n(a.root_d), _n(b.root_d), "mm"),
        Row(
            "reference tooth thickness s_t",
            _n(a.reference_tooth_thickness),
            _n(b.reference_tooth_thickness),
            "mm",
        ),
        Row(
            "normal tooth thickness s_n",
            _n(a.normal_tooth_thickness),
            _n(b.normal_tooth_thickness),
            "mm",
        ),
        Row("outside diameter", _n(a.outside_dia), _n(b.outside_dia), "mm"),
        Row("base radius", _n(a.base_r), _n(b.base_r), "mm"),
        # Tip and root are the other way round on a ring, so they are labelled
        # by what they are rather than by which is bigger.
        Row(
            f"tip radius d_a/2 ({'inner for ring' if p.internal else 'outer'})",
            _n(a.tip_r), _n(b.tip_r), "mm",
        ),
        Row(
            f"root radius d_f/2 ({'outer for ring' if p.internal else 'inner'})",
            _n(a.root_r), _n(b.root_r), "mm",
        ),
        Row(
            "generated root diameter d_fE",
            "not available" if a.generated_root_d is None else _n(a.generated_root_d),
            "not available" if b.generated_root_d is None else _n(b.generated_root_d),
            "mm",
        ),
        Row(
            "undercut",
            "not available" if a.undercut is None else ("yes" if a.undercut else "no"),
            "not available" if b.undercut is None else ("yes" if b.undercut else "no"),
        ),
        Row(
            "root form diameter d_Ff / SOI diameter",
            "not available" if a.root_form_d is None else _n(a.root_form_d),
            "not available" if b.root_form_d is None else _n(b.root_form_d),
            "mm",
        ),
        Row(
            "start of involute angle",
            "not available" if a.start_of_involute_angle is None else _n(a.start_of_involute_angle),
            "not available" if b.start_of_involute_angle is None else _n(b.start_of_involute_angle),
            "rad",
        ),
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
