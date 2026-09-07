"""Build a Gleason/ISO Method 1 hypoid pair in SOLIDWORKS."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from gears.hypoid.geometry import compute_set  # noqa: E402
from gears.hypoid.params import HypoidSetParams  # noqa: E402
from gears.hypoid.validate import validate  # noqa: E402
from gears.sw import SwError, SwSession  # noqa: E402
from gears.sw.hypoid_assembly import build_hypoid_set  # noqa: E402


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--module", type=float, default=170.0 / 42.0)
    ap.add_argument("--z1", type=int, default=13)
    ap.add_argument("--z2", type=int, default=42)
    ap.add_argument("--alpha", type=float, default=20.0)
    ap.add_argument("--sigma", type=float, default=90.0)
    ap.add_argument("--offset", type=float, default=15.0)
    ap.add_argument("--spiral", type=float, default=50.0)
    ap.add_argument("--cutter-radius", type=float, default=63.5)
    ap.add_argument("--hand", choices=("right", "left"), default="right")
    ap.add_argument("--face-width", type=float, default=30.0)
    ap.add_argument("--bore", type=float)
    ap.add_argument("--hub", type=float)
    ap.add_argument("--backlash", type=float, default=0.0)
    ap.add_argument("--out", default="out")
    ap.add_argument("--no-save", action="store_true")
    ap.add_argument("--no-mates", action="store_true")
    ap.add_argument("--reverse-gear", action="store_true")
    args = ap.parse_args(argv)

    overrides = {
        "pressure_angle": args.alpha, "shaft_angle": args.sigma,
        "offset": args.offset, "spiral_angle": args.spiral,
        "cutter_radius": args.cutter_radius, "hand": args.hand,
        "face_width": args.face_width, "backlash": args.backlash,
    }
    if args.bore is not None:
        overrides["bore"] = args.bore
    if args.hub is not None:
        overrides["hub_thickness"] = args.hub
    p = HypoidSetParams.with_defaults(args.module, args.z1, args.z2, **overrides)
    verdict = validate(p)
    for issue in verdict.warnings:
        print(f"WARNING  {issue}")
    if not verdict.ok:
        for issue in verdict.errors:
            print(f"ERROR    {issue}")
        return 1
    geo = compute_set(p)
    print(
        f"building hypoid {p.z1}x{p.z2}, offset {p.offset:g} mm, "
        f"pitch cones {geo.pinion.pitch_angle_deg:.3f} / {geo.gear.pitch_angle_deg:.3f} deg"
    )
    with SwSession() as session:
        try:
            result = build_hypoid_set(
                session, geo, Path(args.out).resolve(),
                save_assembly=not args.no_save,
                mate=not args.no_mates,
                reverse_gear_mate=args.reverse_gear,
            )
        except SwError as exc:
            print(f"BUILD FAILED: {exc}")
            return 1
    print("\nPARTS")
    for part in result.parts:
        print(
            f"  {part.member:<7} {part.teeth:>3} teeth  "
            f"{part.body_count} body  {part.face_count:>4} faces  "
            f"r_a {part.max_radius_mm:.3f} mm"
        )
        if part.path:
            print(f"          saved: {part.path}")

    print("\nASSEMBLY")
    print(f"  shaft angle       {result.measured_shaft_angle_deg:.4f} deg")
    print(f"  angle error       {result.shaft_angle_error_deg:+.2e} deg")
    print(f"  axis offset       {result.measured_offset_mm:.4f} mm")
    print(f"  offset error      {result.offset_error_mm:+.2e} mm")
    print(f"  gear clocking     {result.clocking_deg:.4f} deg")
    if result.interference_count >= 0:
        print(
            f"  interference     {result.interference_count} region(s), "
            f"{result.interference_volume_mm3:.4f} mm3"
        )
    if result.assembly_path:
        print(f"  saved             {result.assembly_path}")

    print("\nMATES")
    for name in result.mates:
        print(f"  {name}")
    if result.gear_ratio:
        print(f"  ratio             {result.gear_ratio[0]:g}:{result.gear_ratio[1]:g}")
    if result.gear_mate_name:
        print(f"  feature           {result.gear_mate_name}")
    print(f"  pinion            {result.pinion_status}")
    print(f"  gear              {result.gear_status}")
    print(f"  articulates       {'yes' if result.articulates else 'NO'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
