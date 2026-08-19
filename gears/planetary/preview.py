"""Planetary scenes: what the GUI shows for a train, without importing tkinter.

The drawing primitives - Scene, View, styles, DXF - are shared and live in
`gears.preview`. The tooth-level scenes are the spur type's own, because every
member of a planetary train *is* a spur gear and there is nothing new to draw
about one; what is here is the scene that only a train has, and the
derived-value table.

The train scene
---------------
`train_scene` is the point of this module and the only view in the codebase that
shows more than two gears at once. It draws every tooth of every member at the
placement and clocking the assembly builder writes, looking down the axis, which
makes it the visual counterpart of the interlock test in `tests/test_planetary.py`:
if a clocking is wrong, teeth visibly cross here.

It ignores the selected member, and its title says so, because a train is not a
view of one gear.
"""

from __future__ import annotations

from pathlib import Path

from ..preview import (
    STYLES,
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
    style_for,
    write_dxf,
)
from ..spur.geometry import tooth_space_section
from ..spur.preview import space_loop
from . import mesh
from .geometry import PlanetarySetGeometry

__all__ = [
    "STYLES", "Polyline", "Row", "Scene", "Style", "View",
    "SCENE_BUILDERS", "SCENE_LABELS",
    "arc", "blank_scene", "build_scene", "circle", "csv_lines", "derived_rows",
    "draw_scale_bar", "draw_scene", "dxf_lines", "nice_length", "rotate",
    "style_for", "train_scene", "transverse_scene", "write_csv", "write_dxf",
]


def _placed_teeth(pair, role, clocking, centre, style) -> list[Polyline]:
    """Every tooth space of one member, clocked and moved to where it sits."""
    member = pair.member(role)
    loop = space_loop(tooth_space_section(pair, role, z=0.0))
    placed: list[Polyline] = []
    for i in range(member.z):
        turned = rotate(loop, i * member.angular_pitch + clocking)
        placed.append(
            Polyline(
                [(x + centre[0], y + centre[1]) for x, y in turned], style, True
            )
        )
    return placed


def train_scene(geo: PlanetarySetGeometry, member: str = "sun") -> Scene:
    """The whole train down the axis: sun, every planet, and the ring.

    Built from the same placements and clockings the assembly builder writes, so
    a phase error shows as teeth crossing rather than as a number in a table.
    """
    p = geo.params
    lines: list[Polyline] = [
        Polyline(circle(geo.ring_rim_radius), "blank"),
        Polyline(circle(geo.sun.pitch_r), "pitch"),
        Polyline(circle(geo.ring.pitch_r), "pitch"),
        # The circle the planets' own axes run on.
        Polyline(circle(geo.centre_distance), "reference"),
    ]

    lines += _placed_teeth(
        geo.sun_planet, "pinion", mesh.sun_clocking(), (0.0, 0.0), "outer"
    )
    lines += _placed_teeth(
        geo.planet_ring, "gear", mesh.ring_clocking(geo, 0), (0.0, 0.0), "inner"
    )
    for k in range(p.n_planets):
        centre = mesh.planet_translation(geo, k)[:2]
        lines += _placed_teeth(
            geo.sun_planet, "gear", mesh.planet_clocking(geo, k), centre, "neighbour"
        )
        lines.append(
            Polyline(
                [
                    (x + centre[0], y + centre[1])
                    for x, y in circle(geo.planet.pitch_r)
                ],
                "pitch",
            )
        )

    return Scene(
        key="train",
        title=(
            f"the whole train - sun {p.z_sun}, {p.n_planets} planets of "
            f"{p.z_planet}, ring {p.z_ring}, at m_n = {p.module:g} mm"
        ),
        polylines=lines,
        legend=[
            ("outer", "sun"),
            ("neighbour", "planets"),
            ("inner", "ring"),
            ("pitch", "pitch circles"),
            ("reference", "planet orbit"),
            ("blank", "ring rim"),
        ],
    )


def _relabelled(scene: Scene, key: str, member: str) -> Scene:
    """A spur scene with its title's leading member name swapped for ours.

    The spur scenes say "pinion" or "gear" because that is what a pair has. A
    planetary member is the same gear either way, so the drawing is right and
    only the name is not.
    """
    _, _, rest = scene.title.partition(" - ")
    return Scene(
        key=key,
        title=f"{member} - {rest or scene.title}",
        polylines=scene.polylines,
        legend=scene.legend,
    )


def transverse_scene(geo: PlanetarySetGeometry, member: str) -> Scene:
    """One member's tooth space, in the transverse plane where the involute lives.

    Delegated to the spur type's own scene: a planetary member is a spur gear,
    and being part of a train changes nothing about its profile.
    """
    from ..spur.preview import transverse_scene as spur_transverse

    return _relabelled(
        spur_transverse(geo.mesh_for(member), geo.role_of(member)),
        "transverse",
        member,
    )


def blank_scene(geo: PlanetarySetGeometry, member: str) -> Scene:
    """One member's blank meridian, again delegated to the spur type."""
    from ..spur.preview import blank_scene as spur_blank

    return _relabelled(
        spur_blank(geo.mesh_for(member), geo.role_of(member)), "blank", member
    )


SCENE_BUILDERS = {
    "train": train_scene,
    "transverse": transverse_scene,
    "blank": blank_scene,
}

SCENE_LABELS = [
    ("train", "The whole train"),
    ("transverse", "Transverse section"),
    ("blank", "Blank section"),
]


def build_scene(geo: PlanetarySetGeometry, member: str, key: str) -> Scene:
    try:
        return SCENE_BUILDERS[key](geo, member)
    except KeyError:
        raise ValueError(f"unknown scene {key!r}") from None


# ---------------------------------------------------------------------------
# Read-only derived values
# ---------------------------------------------------------------------------


def derived_rows(geo: PlanetarySetGeometry) -> list[Row]:
    """The readout table. Three member columns rather than two."""
    p = geo.params
    sun, planet, ring = geo.sun, geo.planet, geo.ring

    def three(label, get, unit=""):
        return Row(label, get(sun), get(planet), unit, third=get(ring))

    return [
        Row("TRAIN", header=True),
        Row("ring teeth (derived)", str(p.z_ring)),
        Row("planets", str(p.n_planets)),
        Row("centre distance", _n(geo.centre_distance), unit="mm"),
        Row(
            "assembly condition",
            "ok" if p.assembly_remainder == 0
            else f"FAILS by {p.assembly_remainder}",
        ),
        Row("neighbour spacing", _n(geo.neighbour_spacing), unit="mm"),
        Row(
            "gap between planet tips",
            _n(geo.neighbour_spacing - 2.0 * geo.planet.tip_r),
            unit="mm",
        ),
        Row("transverse module", _n(geo.transverse_module), unit="mm"),
        Row("whole depth", _n(geo.whole_depth), unit="mm"),
        Row("contact ratio, sun-planet", _n(geo.sun_planet_contact_ratio)),
        Row("contact ratio, planet-ring", _n(geo.planet_ring_contact_ratio)),
        Row("RATIOS", header=True),
        Row("ring held: sun / carrier", _n(p.ratio_carrier_to_sun)),
        Row("carrier held: ring / sun", _n(p.ratio_ring_to_sun)),
        Row("MEMBERS", "SUN", "PLANET", header=True, third="RING"),
        three("teeth", lambda m: str(m.z)),
        three("hand", lambda m: m.hand),
        three("pitch diameter", lambda m: _n(2.0 * m.pitch_r), "mm"),
        three("base radius", lambda m: _n(m.base_r), "mm"),
        three("tip radius", lambda m: _n(m.tip_r), "mm"),
        three("root radius", lambda m: _n(m.root_r), "mm"),
        three("twist over the face", lambda m: _n(m.twist_deg), "deg"),
        Row("ring rim radius", "", "", "mm", third=_n(geo.ring_rim_radius)),
    ]


# ---------------------------------------------------------------------------
# Export
# ---------------------------------------------------------------------------

CSV_HEADER = "member,index,x,y,z"


def csv_lines(geo: PlanetarySetGeometry, member: str) -> list[str]:
    """One member's tooth-space section, as the builder sketches it."""
    section = tooth_space_section(
        geo.mesh_for(member), geo.role_of(member), z=0.0
    )
    lines = [CSV_HEADER]
    for i, (x, y, z) in enumerate(section.loop_3d()):
        lines.append(f"{member},{i},{x:.6f},{y:.6f},{z:.6f}")
    return lines


def write_csv(path: str | Path, geo: PlanetarySetGeometry, member: str) -> Path:
    out = Path(path)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text("\n".join(csv_lines(geo, member)) + "\n", encoding="utf-8")
    return out
