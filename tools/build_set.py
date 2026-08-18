"""Milestone 4: build a complete bevel gear set in SOLIDWORKS.

Produces two parts and an assembly with the teeth meshing.

    .venv\\Scripts\\python.exe tools\\build_set.py --module 2 --z1 17 --z2 43
    .venv\\Scripts\\python.exe tools\\build_set.py --module 3 --z1 20 --z2 20 \\
        --sigma 60 --out out\\miter60
"""

from __future__ import annotations

import argparse
import sys
import traceback
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from gears.bevel.geometry import compute_set          # noqa: E402
from gears.bevel.params import BevelSetParams         # noqa: E402
from gears.bevel.validate import validate             # noqa: E402
from gears.sw import SwError, SwSession, build_set  # noqa: E402


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--module", type=float, default=2.0)
    ap.add_argument("--z1", type=int, default=17)
    ap.add_argument("--z2", type=int, default=43)
    ap.add_argument("--alpha", type=float, default=20.0)
    ap.add_argument("--sigma", type=float, default=90.0)
    ap.add_argument("--face-width", type=float)
    ap.add_argument("--bore", type=float)
    ap.add_argument("--hub", type=float)
    ap.add_argument("--min-root", type=float)
    ap.add_argument("--out", default="out", help="output directory")
    ap.add_argument("--no-save", action="store_true")
    ap.add_argument(
        "--no-mates", action="store_true",
        help="place the pair but leave it floating, with no mates at all",
    )
    ap.add_argument(
        "--reverse-gear", action="store_true",
        help="flip the gear mate, if the pair turns the wrong way when dragged",
    )
    args = ap.parse_args(argv)

    overrides = {"pressure_angle": args.alpha, "shaft_angle": args.sigma}
    for key, value in (
        ("face_width", args.face_width),
        ("bore", args.bore),
        ("hub_thickness", args.hub),
        ("min_root_thickness", args.min_root),
    ):
        if value is not None:
            overrides[key] = value

    p = BevelSetParams.with_defaults(args.module, args.z1, args.z2, **overrides)
    result = validate(p)
    for issue in result.warnings:
        print(f"WARNING  {issue}")
    if not result.ok:
        for issue in result.errors:
            print(f"ERROR    {issue}")
        return 1

    geo = compute_set(p)
    print(
        f"building set: m={p.module} {p.z1}:{p.z2} teeth, "
        f"shaft angle {p.shaft_angle:g} deg, "
        f"cone angles {geo.pinion.pitch_angle_deg:.3f} / "
        f"{geo.gear.pitch_angle_deg:.3f} deg"
    )

    with SwSession() as session:
        try:
            built = build_set(
                session, geo, Path(args.out).resolve(),
                save_assembly=not args.no_save,
                mate=not args.no_mates,
                reverse_gear_mate=args.reverse_gear,
            )
        except SwError as exc:
            print(f"\nBUILD FAILED: {exc}")
            return 1

        print("\nPARTS")
        for part in (built.pinion, built.gear):
            print(
                f"  {part.member:<7} {part.teeth:>3} teeth  "
                f"{part.body_count} body  {part.face_count:>4} faces  "
                f"d_a/2 {part.expected_radius_mm:7.3f} mm "
                f"({part.radius_error_pct:+.2f} % vs approximate box)"
            )
            if part.path:
                print(f"          saved: {part.path}")

        print("\nASSEMBLY")
        print(f"  document          {built.assembly_title}")
        print(f"  shaft angle       {built.shaft_angle_deg:.4f} deg requested")
        print(f"  measured          {built.measured_shaft_angle_deg:.4f} deg")
        print(f"  error             {built.shaft_angle_error_deg:+.2e} deg")
        print(f"  gear clocking     {built.clocking_deg:.4f} deg")
        if built.assembly_path:
            print(f"  saved             {built.assembly_path}")

        print("\nMATES")
        if not built.mates:
            print("  none - the pair is placed but floating")
        else:
            for name in built.mates:
                print(f"  {name}")
            print(f"  pinion            {built.pinion_status}")
            print(f"  gear              {built.gear_status}")
            if built.articulates:
                # Under defined is the healthy answer here: each member keeps
                # the spin about its own axis, and the gear mate joins the two.
                print("  the set turns: drag either member and the other follows")
            else:
                print(
                    "  WARNING: expected both members to come back under "
                    "defined, one spin each"
                )
            # The coupling only acts on a hand drag - no API route runs it -
            # so this is the one result the build cannot check for itself.
            print(
                "  sense              Reverse off, verified by hand on the "
                "anchor set"
            )
            if built.gear_mate_reversed:
                print("  WARNING: --reverse-gear is on, against the verified sense")

        if built.interference_count < 0:
            print("  interference      not available")
        elif built.interference_count == 0:
            print("  interference      none")
        else:
            print(
                f"  interference      {built.interference_count} region(s), "
                f"{built.interference_volume_mm3:.4f} mm3 total"
            )

        if abs(built.shaft_angle_error_deg) > 1e-6:
            print("\nWARNING: the components are not at the requested shaft angle.")
        else:
            print("\nComponent axes are at the requested shaft angle.")

    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception:
        traceback.print_exc()
        sys.exit(1)
