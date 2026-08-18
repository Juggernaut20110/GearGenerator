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

from bevelgear.geometry import compute_set          # noqa: E402
from bevelgear.params import BevelSetParams         # noqa: E402
from bevelgear.validate import validate             # noqa: E402
from bevelgear.sw import SwError, SwSession, build_set  # noqa: E402


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
