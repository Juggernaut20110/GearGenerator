"""Tk-free hypoid scenes and export helpers."""

from __future__ import annotations

import math
from pathlib import Path

from ..preview import (
    Polyline, Row, Scene, STYLES, TAU, arc, circle, dxf_lines, nice_length,
    space_boundary, style_for, tooth_boundary, write_dxf,
)
from .geometry import (
    HypoidSetGeometry,
    blank_outline,
    section_cone_bounds,
    section_cone_distances,
    tooth_space_section,
)

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
        f"{member} - approximate Tredgold hypoid contact, {m.z} teeth, "
        f"offset {geo.params.offset:g} mm",
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
    pinion_bounds = section_cone_bounds(geo, "pinion")
    gear_bounds = section_cone_bounds(geo, "gear")
    return [
        Row("INPUT", header=True),
        Row("shaft angle", f"{p.shaft_angle:.4f}", unit="deg"),
        Row("hypoid offset", f"{p.offset:.4f}", unit="mm"),
        Row("outer transverse backlash (j_et2)", f"{p.backlash:.4f}", unit="mm"),
        Row("backlash convention", "wheel outer transverse cone"),
        Row("METHOD 1 CALCULATED", header=True),
        Row("ratio", f"{p.ratio:.4f}"),
        Row("pitch-plane offset", f"{geo.pitch_plane_offset:.4f}", unit="mm"),
        Row("offset angle", f"{geo.offset_angle_deg:.4f}", unit="deg"),
        Row("mean normal module", f"{geo.mean_normal_module:.4f}", unit="mm"),
        Row("generated drive normal pressure angle", f"{geo.method1.generated_drive_normal_pressure_angle_deg:.4f}", unit="deg"),
        Row("generated coast normal pressure angle", f"{geo.method1.generated_coast_normal_pressure_angle_deg:.4f}", unit="deg"),
        Row("outer transverse backlash", f"{geo.outer_transverse_backlash:.4f}", unit="mm"),
        Row("backlash convention", "outer transverse at wheel outer cone"),
        Row("mean transverse backlash", f"{geo.mean_transverse_backlash:.4f}", unit="mm"),
        Row("mean normal backlash", f"{geo.mean_normal_backlash:.4f}", unit="mm"),
        Row("Method 1 profile-shift coefficient x_hm1", f"{geo.method1_profile_shift_coefficient:.4f}"),
        *([] if geo.method1.mean_tooth_curvature is None else [
            Row("cutter curvature", f"{geo.method1.mean_tooth_curvature:.4f}", unit="mm"),
            Row("limit curvature", f"{geo.method1.limit_radius_of_curvature:.4f}", unit="mm"),
        ]),
        Row("wheel outer transverse module m_et2", f"{geo.wheel_outer_transverse_module:.4f}", unit="mm"),
        Row("wheel mean transverse module m_mt2", f"{geo.wheel_mean_transverse_module:.4f}", unit="mm"),
        Row("wheel physical facewidth b2", f"{geo.wheel_face_width:.4f}", unit="mm"),
        Row("wheel outer cone distance Re2", f"{geo.gear.outer_cone_distance:.4f}", unit="mm"),
        Row("wheel mean cone distance Rm2", f"{geo.gear.cone_distance:.4f}", unit="mm"),
        Row("wheel mean spiral angle beta_m2", f"{geo.gear.mean_spiral_angle_deg:.4f}", unit="deg"),
        Row("face overlap ratio estimate epsilon_beta", f"{geo.face_overlap_ratio_estimate:.4f}"),
        Row("MEMBERS", "PINION", "GEAR", header=True),
        Row("teeth", str(a.z), str(b.z)),
        Row("pitch cone angle", f"{a.pitch_angle_deg:.4f}", f"{b.pitch_angle_deg:.4f}", "deg"),
        Row("mean spiral angle", f"{a.mean_spiral_angle_deg:.4f}", f"{b.mean_spiral_angle_deg:.4f}", "deg"),
        Row("mean transverse drive pressure angle", f"{a.generated_drive_transverse_pressure_angle_deg:.4f}", f"{b.generated_drive_transverse_pressure_angle_deg:.4f}", "deg"),
        Row("mean transverse coast pressure angle", f"{a.generated_coast_transverse_pressure_angle_deg:.4f}", f"{b.generated_coast_transverse_pressure_angle_deg:.4f}", "deg"),
        Row("inner spiral angle", f"{math.degrees(a.inner_spiral_angle):.4f}", f"{math.degrees(b.inner_spiral_angle):.4f}", "deg"),
        Row("outer spiral angle", f"{math.degrees(a.outer_spiral_angle):.4f}", f"{math.degrees(b.outer_spiral_angle):.4f}", "deg"),
        Row("mean pitch radius", f"{a.pitch_radius:.4f}", f"{b.pitch_radius:.4f}", "mm"),
        Row("mean cone distance", f"{a.cone_distance:.4f}", f"{b.cone_distance:.4f}", "mm"),
        Row("Method 1 member facewidth (b_reri1 / b2)", f"{a.face_width:.4f}", f"{b.face_width:.4f}", "mm"),
        Row("physical pitch-cone tooth-face span (b_e + b_i)", f"{a.face_width_along_pitch_cone:.4f}", f"{b.face_width_along_pitch_cone:.4f}", "mm"),
        Row("tooth face inner cone distance", f"{a.tooth_face_inner_cone_distance:.4f}", f"{b.tooth_face_inner_cone_distance:.4f}", "mm"),
        Row("tooth face outer cone distance", f"{a.tooth_face_outer_cone_distance:.4f}", f"{b.tooth_face_outer_cone_distance:.4f}", "mm"),
        Row("addendum", f"{a.addendum:.4f}", f"{b.addendum:.4f}", "mm"),
        Row("dedendum", f"{a.dedendum:.4f}", f"{b.dedendum:.4f}", "mm"),
        Row("face angle", f"{a.face_angle_deg:.4f}", f"{b.face_angle_deg:.4f}", "deg"),
        Row("root angle", f"{a.root_angle_deg:.4f}", f"{b.root_angle_deg:.4f}", "deg"),
        Row("thickness modification coefficient", f"{a.x_sm:.4f}", f"{b.x_sm:.4f}"),
        Row("mean normal tooth thickness", f"{a.mean_normal_tooth_thickness:.4f}", f"{b.mean_normal_tooth_thickness:.4f}", "mm"),
        Row("mean transverse tooth thickness", f"{a.mean_transverse_tooth_thickness:.4f}", f"{b.mean_transverse_tooth_thickness:.4f}", "mm"),
        Row("physical outer tip diameter", f"{a.outer_tip_diameter:.4f}", f"{b.outer_tip_diameter:.4f}", "mm"),
        Row("TREDGOLD APPROXIMATION", header=True),
        Row("developed tip radius", f"{a.tredgold_tip_radius:.4f}", f"{b.tredgold_tip_radius:.4f}", "mm"),
        Row("developed mean root radius", f"{a.tredgold_mean_root_radius:.4f}", f"{b.tredgold_mean_root_radius:.4f}", "mm"),
        Row("SOLIDWORKS CONSTRUCTION-ONLY", header=True),
        Row("physical tooth-face inner", f"{pinion_bounds.tooth_face_inner:.4f}", f"{gear_bounds.tooth_face_inner:.4f}", "mm"),
        Row("physical tooth-face outer", f"{pinion_bounds.tooth_face_outer:.4f}", f"{gear_bounds.tooth_face_outer:.4f}", "mm"),
        Row("loft-only inner extension", f"{pinion_bounds.inner_overshoot:.4f}", f"{gear_bounds.inner_overshoot:.4f}", "mm"),
        Row("loft-only outer extension", f"{pinion_bounds.outer_overshoot:.4f}", f"{gear_bounds.outer_overshoot:.4f}", "mm"),
    ]


def csv_lines(geo: HypoidSetGeometry, member: str) -> list[str]:
    lines = ["member,section_kind,index,x_dev,y_dev,x,y,z"]
    bounds = section_cone_bounds(geo, member)
    for cone_dist in section_cone_distances(geo, member, 2):
        if abs(cone_dist - bounds.tooth_face_inner) < 1e-9:
            label = "physical-tooth-face-inner"
        elif abs(cone_dist - bounds.calculation_point) < 1e-9:
            label = "method1-calculation-point"
        elif abs(cone_dist - bounds.tooth_face_outer) < 1e-9:
            label = "physical-tooth-face-outer"
        elif cone_dist < bounds.tooth_face_inner:
            label = "loft-only-inner-extension"
        elif cone_dist > bounds.tooth_face_outer:
            label = "loft-only-outer-extension"
        else:
            label = "physical-face-intermediate"
        section = tooth_space_section(geo, member, cone_dist, split_cap=True)
        for i, ((xd, yd), (x, y, z)) in enumerate(zip(section.loop_2d, section.loop_3d())):
            lines.append(f"{member},{label},{i},{xd:.6f},{yd:.6f},{x:.6f},{y:.6f},{z:.6f}")
    return lines


def write_csv(path: str | Path, geo: HypoidSetGeometry, member: str) -> Path:
    path = Path(path)
    path.write_text("\n".join(csv_lines(geo, member)) + "\n", encoding="utf-8")
    return path


__all__ = ["SCENE_LABELS", "build_scene", "csv_lines", "derived_rows", "write_csv", "STYLES"]
