"""Terminal report and CLI argument handling for hypoid sets."""

from __future__ import annotations

import math

from ..report_format import emit_issues, row as _row
from .geometry import blank_outline, compute_set, section_count, tooth_space_section
from .params import HypoidSetParams
from .validate import validate

FLAGS = (
    "alpha", "sigma", "offset", "spiral", "cutter_radius", "face_width",
    "bore", "hub", "min_root", "backlash", "member", "end",
)


def add_arguments(ap) -> None:
    ap.add_argument("--offset", type=float, default=0.0, help="signed hypoid offset, mm")


def params_from_args(args) -> HypoidSetParams:
    overrides = {
        "pressure_angle": args.alpha, "shaft_angle": args.sigma,
        "offset": args.offset, "spiral_angle": args.spiral,
        "hand": args.hand, "backlash": args.backlash,
    }
    for source, target in (("cutter_radius", "cutter_radius"), ("face_width", "face_width"),
                           ("bore", "bore"), ("hub", "hub_thickness"), ("min_root", "min_root_thickness")):
        value = getattr(args, source, None)
        if value is not None:
            overrides[target] = value
    return HypoidSetParams.with_defaults(args.module, args.z1, args.z2, **overrides)


def print_report(geo) -> None:
    p = geo.params
    f = lambda value: f"{value:.4f}"
    print("INPUT")
    print(_row("outer transverse module", f(p.module), "", "mm"))
    print(_row("teeth", p.z1, p.z2))
    print(_row("pressure angle", f(p.pressure_angle), "", "deg"))
    print(_row("shaft angle", f(p.shaft_angle), "", "deg"))
    print(_row("hypoid offset", f(p.offset), "", "mm"))
    print(_row("pinion spiral angle", f(p.spiral_angle), "", "deg"))
    print(_row("cutter radius", f(p.cutter_radius or 0), "", "mm"))
    print(_row("face width", f(p.face_width), "", "mm"))
    print(_row("backlash", f(p.backlash), "", "mm"))
    print("\nSET")
    print(_row("ratio", f(p.ratio)))
    print(_row("mean normal module", f(geo.mean_normal_module), "", "mm"))
    print(_row("pitch-plane offset", f(geo.pitch_plane_offset), "", "mm"))
    print(_row("offset angle", f(geo.offset_angle_deg), "", "deg"))
    if geo.method1.mean_tooth_curvature is not None:
        print(_row("cutter curvature", f(geo.method1.mean_tooth_curvature), "", "mm"))
        print(_row("limit curvature", f(geo.method1.limit_radius_of_curvature), "", "mm"))
        print(_row("curvature residual", f(geo.method1.curvature_residual), "", "mm"))
    print(_row("face contact ratio", f(geo.face_contact_ratio)))
    print(f"\nMEMBERS{'':<21}{'PINION':>14}{'GEAR':>14}")
    a, b = geo.pinion, geo.gear
    for label, left, right, unit in (
        ("teeth", a.z, b.z, ""),
        ("pitch cone angle", f(a.pitch_angle_deg), f(b.pitch_angle_deg), "deg"),
        ("mean spiral angle", f(a.mean_spiral_angle_deg), f(b.mean_spiral_angle_deg), "deg"),
        ("mean pitch radius", f(a.pitch_radius), f(b.pitch_radius), "mm"),
        ("mean cone distance", f(a.cone_distance), f(b.cone_distance), "mm"),
        ("normal tooth thickness", f(a.normal_tooth_thickness), f(b.normal_tooth_thickness), "mm"),
        ("transverse tooth thickness", f(a.transverse_tooth_thickness), f(b.transverse_tooth_thickness), "mm"),
        ("outside diameter", f(a.outside_dia), f(b.outside_dia), "mm"),
        ("root radius", f(a.root_r), f(b.root_r), "mm"),
        ("loft sections", section_count(geo, "pinion"), section_count(geo, "gear"), ""),
    ):
        print(_row(label, left, right, unit))


def report(args):
    p = params_from_args(args)
    if not emit_issues(validate(p)):
        return None, None, []
    geo = compute_set(p)
    print_report(geo)
    section = tooth_space_section(geo, args.member, geo.member(args.member).cone_distance)
    loop3d = section.loop_3d()
    print(f"\nTOOTH SPACE SECTION ({args.member}, {args.end})")
    print(_row("boundary points", len(section.loop_2d)))
    print(_row("root radius", f"{section.r_root:.4f}", "", "mm"))
    print(_row("tip radius", f"{section.r_tip:.4f}", "", "mm"))
    print("\nBLANK OUTLINE")
    for radius, z in blank_outline(geo, args.member):
        print(f"    R={radius:9.4f}   z={z:9.4f}")
    return geo, section, loop3d
