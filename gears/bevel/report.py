"""The terminal report for a bevel set. No SOLIDWORKS, no GUI.

Split out of `gears.__main__` when a second gear type arrived, so the dispatcher
stays a dispatcher. The row format is shared - `gears.report_format._row` - but
the rows themselves are bevel quantities and belong next to the geometry that
produces them.
"""

from __future__ import annotations

import math

from ..report_format import emit_issues, row as _row
from .geometry import (
    blank_outline,
    compute_set,
    phase_at_cone_distance,
    section_count,
    tooth_space_section,
)
from .params import BevelSetParams
from .validate import validate

# Flags this type owns. The dispatcher uses these to refuse a flag that belongs
# to the other type rather than silently ignoring it.
FLAGS = (
    "alpha", "sigma", "spiral", "cutter_radius", "face_width", "bore", "hub",
    "min_root", "member", "end",
)


def add_arguments(ap) -> None:
    ap.add_argument("--sigma", type=float, default=90.0, help="shaft angle, deg")
    ap.add_argument(
        "--spiral",
        type=float,
        default=0.0,
        help="mean spiral angle, deg (0 = straight bevel)",
    )
    ap.add_argument(
        "--cutter-radius",
        type=float,
        help="face-milling cutter radius, mm (default: Am). "
             "Given with --spiral 0 this is a Zerol set",
    )
    ap.add_argument(
        "--min-root",
        type=float,
        help="axial rim kept behind the outer root point, mm (default: auto)",
    )
    ap.add_argument("--end", choices=("outer", "inner"), default="outer")


def params_from_args(args) -> BevelSetParams:
    overrides = {
        "pressure_angle": args.alpha,
        "shaft_angle": args.sigma,
        "spiral_angle": args.spiral,
        "hand": args.hand,
    }
    if args.cutter_radius is not None:
        overrides["cutter_radius"] = args.cutter_radius
    if args.face_width is not None:
        overrides["face_width"] = args.face_width
    if args.bore is not None:
        overrides["bore"] = args.bore
    if args.hub is not None:
        overrides["hub_thickness"] = args.hub
    if args.min_root is not None:
        overrides["min_root_thickness"] = args.min_root
    return BevelSetParams.with_defaults(args.module, args.z1, args.z2, **overrides)


def print_report(geo) -> None:
    p = geo.params
    f = lambda v: f"{v:.4f}"  # noqa: E731

    print("INPUT")
    print(_row("module", f(p.module), "", "mm"))
    print(_row("teeth", p.z1, p.z2))
    print(_row("pressure angle", f(p.pressure_angle), "", "deg"))
    print(_row("shaft angle", f(p.shaft_angle), "", "deg"))
    print(_row("spiral angle (mean)", f(p.spiral_angle), "", "deg"))
    if p.is_curved:
        print(_row("hand (pinion)", p.hand))
        print(_row("cutter radius", f(p.cutter_radius or 0.0), "", "mm"))
    print(_row("face width", f(p.face_width), "", "mm"))
    print(_row("bore", f(p.bore), "", "mm"))
    print(_row("hub thickness", f(p.hub_thickness), "", "mm"))
    print(_row("min root thickness", f(p.min_root_thickness), "", "mm"))

    print("\nSET")
    print(_row("ratio", f(p.ratio)))
    print(_row("outer cone distance Ao", f(geo.outer_cone_dist), "", "mm"))
    print(_row("mean cone distance", f(geo.mean_cone_dist), "", "mm"))
    print(_row("inner cone distance Ai", f(geo.inner_cone_dist), "", "mm"))
    print(_row("section scale k = Ai/Ao", f(geo.section_scale)))
    print(_row("circular pitch", f(geo.circular_pitch), "", "mm"))
    print(_row("working depth", f(geo.working_depth), "", "mm"))
    print(_row("whole depth", f(geo.whole_depth), "", "mm"))
    print(_row("clearance", f(geo.clearance), "", "mm"))

    if geo.trace is not None:
        t = geo.trace
        deg = lambda A: f(math.degrees(abs(t.spiral_angle_at(A))))  # noqa: E731
        print("\nSPIRAL")
        print(_row("cutter radius", f(t.cutter_radius), "", "mm"))
        print(_row("cutter offset rho", f(t.centre_distance), "", "mm"))
        print(_row("spiral angle at toe", deg(geo.inner_cone_dist), "", "deg"))
        print(_row("spiral angle at mean", deg(geo.mean_cone_dist), "", "deg"))
        print(_row("spiral angle at heel", deg(geo.outer_cone_dist), "", "deg"))
        print(_row("contact ratio, face", f(geo.face_contact_ratio)))

    print(f"\nMEMBERS{'':<21}{'PINION':>14}{'GEAR':>14}")
    a, b = geo.pinion, geo.gear
    print(_row("teeth", a.z, b.z))
    print(_row("pitch cone angle", f(a.pitch_angle_deg), f(b.pitch_angle_deg), "deg"))
    print(_row("face cone angle", f(a.face_angle_deg), f(b.face_angle_deg), "deg"))
    print(_row("root cone angle", f(a.root_angle_deg), f(b.root_angle_deg), "deg"))
    print(_row("pitch diameter", f(a.pitch_dia), f(b.pitch_dia), "mm"))
    print(_row("outside diameter", f(a.outside_dia), f(b.outside_dia), "mm"))
    print(_row("addendum", f(a.addendum), f(b.addendum), "mm"))
    print(_row("dedendum", f(a.dedendum), f(b.dedendum), "mm"))
    print(_row("virtual teeth z_v", f(a.virtual_teeth), f(b.virtual_teeth)))
    print(_row("back cone distance", f(a.virtual_pitch_r), f(b.virtual_pitch_r), "mm"))
    print(_row("tip radius, outer", f(a.virtual_tip_r), f(b.virtual_tip_r), "mm"))
    print(_row("tip radius, inner", f(a.virtual_tip_r_inner), f(b.virtual_tip_r_inner), "mm"))
    print(_row("crown to apex", f(a.crown_to_apex), f(b.crown_to_apex), "mm"))
    print(_row("outer root to apex", f(a.root_to_apex), f(b.root_to_apex), "mm"))
    print(_row("outer root radius", f(a.outer_root_radius), f(b.outer_root_radius), "mm"))
    print(_row("mounting distance", f(a.mounting_distance), f(b.mounting_distance), "mm"))

    if geo.trace is not None:
        sweeps = [
            phase_at_cone_distance(geo, name, geo.outer_cone_dist)
            - phase_at_cone_distance(geo, name, geo.inner_cone_dist)
            for name in ("pinion", "gear")
        ]
        print(_row("sweep over the face",
                   f(math.degrees(sweeps[0])), f(math.degrees(sweeps[1])), "deg"))
        print(_row("as angular pitches",
                   f(abs(sweeps[0]) / a.angular_pitch),
                   f(abs(sweeps[1]) / b.angular_pitch)))
        print(_row("loft sections",
                   section_count(geo, "pinion"), section_count(geo, "gear")))


def report(args) -> tuple[object, object, list]:
    """Print the whole report; return (geo, section, loop_3d) for the CSV dump."""
    p = params_from_args(args)
    if not emit_issues(validate(p)):
        return None, None, []

    geo = compute_set(p)
    print_report(geo)

    section = tooth_space_section(geo, args.member, args.end)
    loop3d = section.loop_3d()
    f = lambda v: f"{v:.4f}"  # noqa: E731
    print(f"\nTOOTH SPACE SECTION  ({args.member}, {args.end} end)")
    print(_row("boundary points", len(section.loop_2d)))
    print(_row("root fillet fitted", "yes" if section.filleted else "no (sharp)"))
    print(_row("root radius (developed)", f(section.r_root), "", "mm"))
    print(_row("tip radius (developed)", f(section.r_tip), "", "mm"))
    print(_row("axial span z", f"{min(q[2] for q in loop3d):.4f}"
               f" .. {max(q[2] for q in loop3d):.4f}", "", "mm"))

    print(f"\nBLANK OUTLINE  ({args.member}, meridian R/z, apex at origin)")
    for R, z in blank_outline(geo, args.member):
        print(f"    R={R:9.4f}   z={z:9.4f}")

    return geo, section, loop3d
