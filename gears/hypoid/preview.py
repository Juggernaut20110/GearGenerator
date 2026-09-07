"""Tk-free hypoid scenes and export helpers."""

from __future__ import annotations

import math
from pathlib import Path

from ..preview import (
    Polyline, Row, Scene, STYLES, TAU, arc, circle, dxf_lines, nice_length,
    space_boundary, style_for, tooth_boundary, write_dxf,
)
from .geometry import HypoidSetGeometry, blank_outline, section_cone_distances, tooth_space_section

SCENE_LABELS = [("contact", "Contact geometry"), ("section", "Tooth section"), ("blank", "Blank section")]
SCENE_BUILDERS = {}


def _tooth(section, step):
    return tooth_boundary(section.segments, step) or list(section.loop_2d)


def contact_scene(geo: HypoidSetGeometry, member: str) -> Scene:
    m = geo.member(member)
    section = tooth_space_section(geo, member)
    step = TAU / m.z
    loop = _tooth(section, step)
    lines = [Polyline(circle(m.pitch_radius), "pitch")]
    for i in range(-1, 2):
        rotated = []
        c, s = math.cos(i * step), math.sin(i * step)
        for x, y in loop:
            rotated.append((x * c - y * s, x * s + y * c))
        lines.append(Polyline(rotated, "outer", True))
    return Scene(
        "contact",
        f"{member} - hypoid contact, {m.z} teeth, offset {geo.params.offset:g} mm",
        lines,
        [("outer", "tooth"), ("pitch", "mean pitch circle")],
    )


def section_scene(geo: HypoidSetGeometry, member: str) -> Scene:
    m = geo.member(member)
    section = tooth_space_section(geo, member)
    step = TAU / m.z
    return Scene(
        "section",
        f"{member} - Tredgold section, {m.z} teeth",
        [
            Polyline(circle(m.pitch_radius), "pitch"),
            Polyline(circle(m.virtual_base_r), "reference"),
            Polyline(circle(m.root_r), "reference"),
            Polyline(circle(m.tip_r), "reference"),
            Polyline(_tooth(section, step), "outer", True),
            Polyline(section.loop_2d, "cut", True),
        ],
        [("outer", "tooth"), ("cut", "tooth-space cut"), ("pitch", "mean pitch")],
    )


def blank_scene(geo: HypoidSetGeometry, member: str) -> Scene:
    outline = blank_outline(geo, member)
    m = geo.member(member)
    return Scene(
        "blank",
        f"{member} - hypoid blank meridian",
        [
            Polyline([(0.0, min(z for _, z in outline) - 2), (0.0, max(z for _, z in outline) + 2)], "axis"),
            Polyline(list(outline), "blank", True),
            Polyline([(m.pitch_radius, min(z for _, z in outline)), (m.pitch_radius, max(z for _, z in outline))], "pitch"),
        ],
        [("blank", "revolved outline"), ("pitch", "mean pitch radius"), ("axis", "axis")],
    )


SCENE_BUILDERS.update({"contact": contact_scene, "section": section_scene, "blank": blank_scene})


def build_scene(geo: HypoidSetGeometry, member: str, key: str) -> Scene:
    try:
        return SCENE_BUILDERS[key](geo, member)
    except KeyError:
        raise ValueError(f"unknown scene {key!r}") from None


def derived_rows(geo: HypoidSetGeometry) -> list[Row]:
    p = geo.params
    a, b = geo.pinion, geo.gear
    return [
        Row("SET", header=True),
        Row("ratio", f"{p.ratio:.4f}"),
        Row("shaft angle", f"{p.shaft_angle:.4f}", unit="deg"),
        Row("hypoid offset", f"{p.offset:.4f}", unit="mm"),
        Row("pitch-plane offset", f"{geo.pitch_plane_offset:.4f}", unit="mm"),
        Row("offset angle", f"{geo.offset_angle_deg:.4f}", unit="deg"),
        Row("mean normal module", f"{geo.mean_normal_module:.4f}", unit="mm"),
        Row("outer transverse backlash", f"{geo.outer_transverse_backlash:.4f}", unit="mm"),
        Row("backlash convention", "outer transverse at wheel outer cone"),
        Row("mean transverse backlash", f"{geo.mean_transverse_backlash:.4f}", unit="mm"),
        Row("mean normal backlash", f"{geo.mean_normal_backlash:.4f}", unit="mm"),
        *([] if geo.method1.mean_tooth_curvature is None else [
            Row("cutter curvature", f"{geo.method1.mean_tooth_curvature:.4f}", unit="mm"),
            Row("limit curvature", f"{geo.method1.limit_radius_of_curvature:.4f}", unit="mm"),
        ]),
        Row("face contact ratio", f"{geo.face_contact_ratio:.4f}"),
        Row("MEMBERS", "PINION", "GEAR", header=True),
        Row("teeth", str(a.z), str(b.z)),
        Row("pitch cone angle", f"{a.pitch_angle_deg:.4f}", f"{b.pitch_angle_deg:.4f}", "deg"),
        Row("mean spiral angle", f"{a.mean_spiral_angle_deg:.4f}", f"{b.mean_spiral_angle_deg:.4f}", "deg"),
        Row("mean pitch radius", f"{a.pitch_radius:.4f}", f"{b.pitch_radius:.4f}", "mm"),
        Row("mean cone distance", f"{a.cone_distance:.4f}", f"{b.cone_distance:.4f}", "mm"),
        Row("face width", f"{a.face_width:.4f}", f"{b.face_width:.4f}", "mm"),
        Row("addendum", f"{a.addendum:.4f}", f"{b.addendum:.4f}", "mm"),
        Row("dedendum", f"{a.dedendum:.4f}", f"{b.dedendum:.4f}", "mm"),
        Row("face angle", f"{a.face_angle_deg:.4f}", f"{b.face_angle_deg:.4f}", "deg"),
        Row("root angle", f"{a.root_angle_deg:.4f}", f"{b.root_angle_deg:.4f}", "deg"),
        Row("thickness modification coefficient", f"{a.x_sm:.4f}", f"{b.x_sm:.4f}"),
        Row("mean normal tooth thickness", f"{a.mean_normal_tooth_thickness:.4f}", f"{b.mean_normal_tooth_thickness:.4f}", "mm"),
        Row("mean transverse tooth thickness", f"{a.mean_transverse_tooth_thickness:.4f}", f"{b.mean_transverse_tooth_thickness:.4f}", "mm"),
        Row("outside diameter", f"{a.outside_dia:.4f}", f"{b.outside_dia:.4f}", "mm"),
    ]


def csv_lines(geo: HypoidSetGeometry, member: str) -> list[str]:
    lines = ["member,section,index,x_dev,y_dev,x,y,z"]
    m = geo.member(member)
    for label, cone_dist in zip(("inner", "outer"), section_cone_distances(geo, member, 2)):
        section = tooth_space_section(geo, member, cone_dist, split_cap=True)
        for i, ((xd, yd), (x, y, z)) in enumerate(zip(section.loop_2d, section.loop_3d())):
            lines.append(f"{member},{label},{i},{xd:.6f},{yd:.6f},{x:.6f},{y:.6f},{z:.6f}")
    return lines


def write_csv(path: str | Path, geo: HypoidSetGeometry, member: str) -> Path:
    path = Path(path)
    path.write_text("\n".join(csv_lines(geo, member)) + "\n", encoding="utf-8")
    return path


__all__ = ["SCENE_LABELS", "build_scene", "csv_lines", "derived_rows", "write_csv", "STYLES"]
