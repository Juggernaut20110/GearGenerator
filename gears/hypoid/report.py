"""Terminal report and CLI argument handling for hypoid sets."""

from __future__ import annotations

import math

from ..report_format import emit_issues, row as _row
from .geometry import (
    blank_outline,
    compute_set,
    section_cone_bounds,
    section_count,
    tooth_space_section,
)
from .params import HypoidSetParams
from .validate import validate

FLAGS = (
    "alpha", "sigma", "offset", "spiral", "cutter_radius", "face_width",
    "bore", "hub", "min_root", "root_fillet_radius", "backlash", "member", "end",
)


def add_arguments(ap) -> None:
    ap.add_argument("--offset", type=float, default=0.0, help="signed hypoid offset, mm")
    ap.add_argument(
        "--root-fillet-radius", type=float, default=None,
        help="approximate Tredgold tooth-root fillet radius, mm (default: 0.1 module)",
    )


def params_from_args(args) -> HypoidSetParams:
    overrides = {
        "pressure_angle": args.alpha, "shaft_angle": args.sigma,
        "offset": args.offset, "spiral_angle": args.spiral,
        "hand": args.hand, "backlash": args.backlash,
    }
    for source, target in (("cutter_radius", "cutter_radius"), ("face_width", "face_width"),
                           ("bore", "bore"), ("hub", "hub_thickness"),
                           ("min_root", "min_root_thickness"),
                           ("root_fillet_radius", "root_fillet_radius")):
        value = getattr(args, source, None)
        if value is not None:
            overrides[target] = value
    return HypoidSetParams.with_defaults(args.module, args.z1, args.z2, **overrides)


def print_report(geo) -> None:
    p = geo.params
    f = lambda value: f"{value:.4f}"
    optional = lambda value: "auto" if value is None else f(value)
    print("INPUT")
    print(_row("outer transverse module", f(p.module), "", "mm"))
    print(_row("teeth", p.z1, p.z2))
    print(_row("pressure angle", f(p.pressure_angle), "", "deg"))
    print(_row("shaft angle", f(p.shaft_angle), "", "deg"))
    print(_row("hypoid offset", f(p.offset), "", "mm"))
    print(_row("pinion spiral-angle magnitude", f(p.spiral_angle), "", "deg"))
    print(_row("pinion hand", p.hand))
    print(_row("cutter radius", optional(p.cutter_radius), "", "mm"))
    print(_row("approximate Tredgold root fillet radius", optional(p.root_fillet_radius), "", "mm"))
    print(_row("input wheel facewidth b2", f(p.face_width), "", "mm"))
    print(_row("outer transverse backlash (j_et2)", f(geo.outer_transverse_backlash), "", "mm"))
    print(_row("backlash convention", "outer transverse at wheel outer cone"))
    print(_row("gear addendum angle input", f(p.gear_addendum_angle), "", "deg"))
    print(_row("gear dedendum angle input", f(p.gear_dedendum_angle), "", "deg"))

    print("\nMETHOD 1 CALCULATED")
    print(_row("ratio", f(p.ratio)))
    print(_row("mean normal module", f(geo.mean_normal_module), "", "mm"))
    print(_row("mean normal pressure angle", f(geo.thickness.mean_normal_pressure_angle_deg), "", "deg"))
    print(_row("generated drive normal pressure angle", f(geo.method1.generated_drive_normal_pressure_angle_deg), "", "deg"))
    print(_row("generated coast normal pressure angle", f(geo.method1.generated_coast_normal_pressure_angle_deg), "", "deg"))
    print(_row("mean transverse backlash", f(geo.mean_transverse_backlash), "", "mm"))
    print(_row("mean normal backlash", f(geo.mean_normal_backlash), "", "mm"))
    print(_row("pitch-plane offset", f(geo.pitch_plane_offset), "", "mm"))
    print(_row("offset angle", f(geo.offset_angle_deg), "", "deg"))
    print(_row("Method 1 profile-shift coefficient x_hm1", f(geo.method1_profile_shift_coefficient)))
    if geo.method1.mean_tooth_curvature is not None:
        print(_row("cutter curvature", f(geo.method1.mean_tooth_curvature), "", "mm"))
        print(_row("limit curvature", f(geo.method1.limit_radius_of_curvature), "", "mm"))
        print(_row("curvature residual", f(geo.method1.curvature_residual), "", "mm"))
    print(_row("wheel outer transverse module m_et2", f(geo.wheel_outer_transverse_module), "", "mm"))
    print(_row("wheel mean transverse module m_mt2", f(geo.wheel_mean_transverse_module), "", "mm"))
    print(_row("wheel physical facewidth b2", f(geo.wheel_face_width), "", "mm"))
    print(_row("wheel outer cone distance Re2", f(geo.gear.outer_cone_distance), "", "mm"))
    print(_row("wheel mean spiral angle beta_m2", f(geo.gear.mean_spiral_angle_deg), "", "deg"))
    print(_row("spiral-bevel face overlap estimate epsilon_beta", f(geo.spiral_bevel_face_overlap_estimate)))
    print(f"\nMEMBERS{'':<21}{'PINION':>14}{'GEAR':>14}")
    a, b = geo.pinion, geo.gear
    for label, left, right, unit in (
        ("teeth", a.z, b.z, ""),
        ("pitch cone angle", f(a.pitch_angle_deg), f(b.pitch_angle_deg), "deg"),
        ("mean spiral angle", f(a.mean_spiral_angle_deg), f(b.mean_spiral_angle_deg), "deg"),
        ("mean transverse drive pressure angle", f(a.generated_drive_transverse_pressure_angle_deg), f(b.generated_drive_transverse_pressure_angle_deg), "deg"),
        ("mean transverse coast pressure angle", f(a.generated_coast_transverse_pressure_angle_deg), f(b.generated_coast_transverse_pressure_angle_deg), "deg"),
        ("inner spiral angle", f(math.degrees(a.inner_spiral_angle)), f(math.degrees(b.inner_spiral_angle)), "deg"),
        ("outer spiral angle", f(math.degrees(a.outer_spiral_angle)), f(math.degrees(b.outer_spiral_angle)), "deg"),
        ("mean pitch radius", f(a.pitch_radius), f(b.pitch_radius), "mm"),
        ("mean cone distance", f(a.cone_distance), f(b.cone_distance), "mm"),
        ("Method 1 member facewidth (b_reri1 / b2)", f(a.face_width), f(b.face_width), "mm"),
        ("physical pitch-cone tooth-face span (b_e + b_i)", f(a.face_width_along_pitch_cone), f(b.face_width_along_pitch_cone), "mm"),
        ("tooth face inner cone distance", f(a.tooth_face_inner_cone_distance), f(b.tooth_face_inner_cone_distance), "mm"),
        ("tooth face outer cone distance", f(a.tooth_face_outer_cone_distance), f(b.tooth_face_outer_cone_distance), "mm"),
        ("outer boundary from calculation point (b_e)", f(a.outer_face_width), f(b.outer_face_width), "mm"),
        ("inner boundary from calculation point (b_i)", f(a.inner_face_width), f(b.inner_face_width), "mm"),
        ("addendum", f(a.addendum), f(b.addendum), "mm"),
        ("dedendum", f(a.dedendum), f(b.dedendum), "mm"),
        ("working depth", f(a.working_depth), f(b.working_depth), "mm"),
        ("clearance", f(a.clearance), f(b.clearance), "mm"),
        ("whole depth", f(a.whole_depth), f(b.whole_depth), "mm"),
        ("face angle", f(a.face_angle_deg), f(b.face_angle_deg), "deg"),
        ("root angle", f(a.root_angle_deg), f(b.root_angle_deg), "deg"),
        ("thickness modification coefficient", f(a.x_sm), f(b.x_sm), ""),
        ("mean normal tooth thickness", f(a.mean_normal_tooth_thickness), f(b.mean_normal_tooth_thickness), "mm"),
        ("mean transverse tooth thickness", f(a.mean_transverse_tooth_thickness), f(b.mean_transverse_tooth_thickness), "mm"),
        ("physical outer tip diameter", f(a.outer_tip_diameter), f(b.outer_tip_diameter), "mm"),
        ("outer root diameter", f(a.outer_root_diameter), f(b.outer_root_diameter), "mm"),
        ("Tredgold developed tip radius", f(a.tredgold_tip_radius), f(b.tredgold_tip_radius), "mm"),
        ("Tredgold developed mean root radius", f(a.tredgold_mean_root_radius), f(b.tredgold_mean_root_radius), "mm"),
        ("physical Method 1 outer root radius", f(a.outer_root_radius), f(b.outer_root_radius), "mm"),
        ("loft sections", section_count(geo, "pinion"), section_count(geo, "gear"), ""),
    ):
        print(_row(label, left, right, unit))

    print("\nSOLIDWORKS CONSTRUCTION-ONLY LOFT EXTENSIONS")
    print(_row("quantity", "PINION", "GEAR", "mm"))
    pinion_bounds = section_cone_bounds(geo, "pinion")
    gear_bounds = section_cone_bounds(geo, "gear")
    for label, left, right in (
        ("physical tooth-face inner boundary", pinion_bounds.tooth_face_inner, gear_bounds.tooth_face_inner),
        ("physical tooth-face outer boundary", pinion_bounds.tooth_face_outer, gear_bounds.tooth_face_outer),
        ("construction loft inner boundary", pinion_bounds.loft_inner, gear_bounds.loft_inner),
        ("construction loft outer boundary", pinion_bounds.loft_outer, gear_bounds.loft_outer),
        ("inner construction overshoot", pinion_bounds.inner_overshoot, gear_bounds.inner_overshoot),
        ("outer construction overshoot", pinion_bounds.outer_overshoot, gear_bounds.outer_overshoot),
    ):
        print(_row(label, f(left), f(right), "mm"))


def report(args):
    p = params_from_args(args)
    if not emit_issues(validate(p)):
        return None, None, []
    geo = compute_set(p)
    print_report(geo)
    section = tooth_space_section(geo, args.member, geo.member(args.member).cone_distance)
    loop3d = section.loop_3d()
    print(f"\nTREDGOLD APPROXIMATE TOOTH-SPACE SECTION ({args.member}, {args.end})")
    print(_row("boundary points", len(section.loop_2d)))
    print(_row("root radius", f"{section.r_root:.4f}", "", "mm"))
    print(_row("tip radius", f"{section.r_tip:.4f}", "", "mm"))
    print("\nMETHOD 1 BLANK OUTLINE")
    for radius, z in blank_outline(geo, args.member):
        print(f"    R={radius:9.4f}   z={z:9.4f}")
    return geo, section, loop3d
