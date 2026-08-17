"""Dump derived bevel gear geometry to the terminal, with no SOLIDWORKS involved.

    .venv\\Scripts\\python.exe -m bevelgear --module 2 --z1 17 --z2 43
    .venv\\Scripts\\python.exe -m bevelgear --module 2 --z1 17 --z2 43 \\
        --csv out/pinion_outer.csv --member pinion --end outer
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

from .geometry import blank_outline, compute_set, tooth_space_section
from .params import BevelSetParams
from .validate import validate


def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(prog="bevelgear", description=__doc__)
    ap.add_argument("--module", type=float, required=True, help="outer module, mm")
    ap.add_argument("--z1", type=int, required=True, help="pinion tooth count")
    ap.add_argument("--z2", type=int, required=True, help="gear tooth count")
    ap.add_argument("--alpha", type=float, default=20.0, help="pressure angle, deg")
    ap.add_argument("--sigma", type=float, default=90.0, help="shaft angle, deg")
    ap.add_argument("--face-width", type=float, help="mm (default: auto)")
    ap.add_argument("--bore", type=float, help="mm (default: auto)")
    ap.add_argument("--hub", type=float, help="hub/backing thickness, mm (default: auto)")
    ap.add_argument("--member", choices=("pinion", "gear"), default="pinion")
    ap.add_argument("--end", choices=("outer", "inner"), default="outer")
    ap.add_argument("--csv", type=Path, help="write the section's points to this file")
    return ap


def params_from_args(args) -> BevelSetParams:
    overrides = {"pressure_angle": args.alpha, "shaft_angle": args.sigma}
    if args.face_width is not None:
        overrides["face_width"] = args.face_width
    if args.bore is not None:
        overrides["bore"] = args.bore
    if args.hub is not None:
        overrides["hub_thickness"] = args.hub
    return BevelSetParams.with_defaults(args.module, args.z1, args.z2, **overrides)


def _row(label: str, pinion, gear="", unit="") -> str:
    return f"  {label:<28}{pinion:>14}{gear:>14}  {unit}"


def print_report(geo) -> None:
    p = geo.params
    f = lambda v: f"{v:.4f}"  # noqa: E731

    print("INPUT")
    print(_row("module", f(p.module), "", "mm"))
    print(_row("teeth", p.z1, p.z2))
    print(_row("pressure angle", f(p.pressure_angle), "", "deg"))
    print(_row("shaft angle", f(p.shaft_angle), "", "deg"))
    print(_row("face width", f(p.face_width), "", "mm"))
    print(_row("bore", f(p.bore), "", "mm"))
    print(_row("hub thickness", f(p.hub_thickness), "", "mm"))

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


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    p = params_from_args(args)

    result = validate(p)
    for issue in result.errors:
        print(f"ERROR    {issue}", file=sys.stderr)
    for issue in result.warnings:
        print(f"WARNING  {issue}", file=sys.stderr)
    if not result.ok:
        return 1
    if result.warnings:
        print(file=sys.stderr)

    geo = compute_set(p)
    print_report(geo)

    section = tooth_space_section(geo, args.member, args.end)
    loop3d = section.loop_3d()
    print(f"\nTOOTH SPACE SECTION  ({args.member}, {args.end} end)")
    print(_row("boundary points", len(section.loop_2d)))
    print(_row("root fillet fitted", "yes" if section.filleted else "no (sharp)"))
    print(_row("root radius (developed)", f"{section.r_root:.4f}", "", "mm"))
    print(_row("tip radius (developed)", f"{section.r_tip:.4f}", "", "mm"))
    print(_row("axial span z", f"{min(q[2] for q in loop3d):.4f}"
               f" .. {max(q[2] for q in loop3d):.4f}", "", "mm"))

    outline = blank_outline(geo, args.member)
    print(f"\nBLANK OUTLINE  ({args.member}, meridian R/z, apex at origin)")
    for R, z in outline:
        print(f"    R={R:9.4f}   z={z:9.4f}")

    if args.csv:
        args.csv.parent.mkdir(parents=True, exist_ok=True)
        lines = ["x_dev,y_dev,x,y,z"]
        for (xd, yd), (x, y, z) in zip(section.loop_2d, loop3d):
            lines.append(f"{xd:.6f},{yd:.6f},{x:.6f},{y:.6f},{z:.6f}")
        args.csv.write_text("\n".join(lines) + "\n", encoding="utf-8")
        print(f"\nwrote {len(loop3d)} points to {args.csv}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
