"""Build a meshing pair of involute spur gears in SOLIDWORKS.

    .venv\\Scripts\\python.exe tools\\build_spur_set.py --z1 17 --z2 43
    .venv\\Scripts\\python.exe tools\\build_spur_set.py --z1 17 --z2 43 --beta 15
    .venv\\Scripts\\python.exe tools\\build_spur_set.py --no-mates

Both parts plus an assembly with the pair clocked, meshing and coupled by a gear
mate, so dragging either member turns the other.

`--reverse-gear` flips the gear mate's sense. **Which setting is right has not
been measured for a spur pair** - the bevel answer was found by hand and does
not transfer, because its axes stand at 90 degrees and these are parallel. Drag
the pinion and watch which way the gear goes; if it is wrong, pass the flag and
record the answer in the module docstring of `gears/sw/spur_assembly.py`.
"""

from __future__ import annotations

import argparse
import sys
import traceback
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from gears.spur.geometry import compute_set             # noqa: E402
from gears.spur.params import SpurSetParams             # noqa: E402
from gears.spur.validate import validate                # noqa: E402
from gears.sw import SwError, SwSession                 # noqa: E402
from gears.sw.spur_assembly import build_spur_set       # noqa: E402


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--module", type=float, default=2.0, help="normal module, mm")
    ap.add_argument("--z1", type=int, default=17)
    ap.add_argument("--z2", type=int, default=43)
    ap.add_argument("--alpha", type=float, default=20.0, help="normal pressure angle")
    ap.add_argument("--beta", type=float, default=0.0, help="helix angle, deg")
    ap.add_argument("--hand", choices=("right", "left"), default="right")
    ap.add_argument("--face-width", type=float)
    ap.add_argument("--bore", type=float)
    ap.add_argument("--hub", type=float)
    ap.add_argument("--backlash", type=float)
    ap.add_argument("--out", default="out", help="directory for the parts and assembly")
    ap.add_argument("--no-save", action="store_true", help="do not save the assembly")
    ap.add_argument("--no-mates", action="store_true", help="place but do not constrain")
    ap.add_argument(
        "--reverse-gear", action="store_true", help="flip the gear mate's sense"
    )
    args = ap.parse_args(argv)

    overrides = {
        "pressure_angle": args.alpha,
        "helix_angle": args.beta,
        "hand": args.hand,
    }
    for key, value in (
        ("face_width", args.face_width),
        ("bore", args.bore),
        ("hub_thickness", args.hub),
        ("backlash", args.backlash),
    ):
        if value is not None:
            overrides[key] = value

    p = SpurSetParams.with_defaults(args.module, args.z1, args.z2, **overrides)
    result = validate(p)
    for issue in result.warnings:
        print(f"WARNING  {issue}")
    if not result.ok:
        for issue in result.errors:
            print(f"ERROR    {issue}")
        return 1

    geo = compute_set(p)
    print(
        f"building {geo.pinion.z}x{geo.gear.z} at m_n={p.module}, "
        f"beta={p.helix_angle:g} deg, centre distance {geo.centre_distance:.4f} mm"
    )
    if p.helix_angle:
        print(
            f"  hands: pinion {geo.pinion.hand}, gear {geo.gear.hand} "
            "(opposite, or they will not mesh)"
        )

    with SwSession() as session:
        try:
            built = build_spur_set(
                session,
                geo,
                args.out,
                save_assembly=not args.no_save,
                mate=not args.no_mates,
                reverse_gear_mate=args.reverse_gear,
            )
        except SwError as exc:
            print(f"\nBUILD FAILED: {exc}")
            return 1

        print("\nPARTS")
        for member, part in (("pinion", built.pinion), ("gear", built.gear)):
            print(
                f"  {member:<7} {part.teeth:>3} teeth  {part.face_count:>4} faces  "
                f"r_a {part.max_radius_mm:7.3f} mm ({part.radius_error_pct:+.2f} %)"
            )
            if part.path:
                print(f"          saved to {part.path}")

        print("\nASSEMBLY")
        print(f"  document              {built.assembly_title}")
        print(f"  centre distance       {built.centre_distance_mm:.4f} mm")
        print(f"  measured              {built.measured_centre_distance_mm:.4f} mm")
        print(f"  error                 {built.centre_distance_error_mm:+.6f} mm")
        print(f"  axes measured apart   {built.measured_axis_angle_deg:.6f} deg")
        print(f"  gear clocking         {built.clocking_deg:.6f} deg")
        if built.interference_count >= 0:
            print(
                f"  interference          {built.interference_count} volume "
                f"{built.interference_volume_mm3:.4f} mm3"
            )
        if built.assembly_path:
            print(f"  saved to              {built.assembly_path}")

        print("\nMATES")
        for name in built.mates:
            print(f"  {name}")
        if built.gear_ratio:
            print(
                f"  ratio {built.gear_ratio[0]:g}:{built.gear_ratio[1]:g}"
                + ("  reversed" if built.gear_mate_reversed else "")
            )
        print(f"  pinion is {built.pinion_status}, gear is {built.gear_status}")
        print(f"  articulates: {'yes' if built.articulates else 'NO'}")

        # The axes are parallel by construction, so any measurable angle between
        # them means a mate solved somewhere the transform did not put it.
        if built.measured_axis_angle_deg > 1e-6:
            print(
                f"\nWARNING: the axes are {built.measured_axis_angle_deg:.6f} deg "
                "apart but should be parallel."
            )
        if abs(built.centre_distance_error_mm) > 1e-6:
            print(
                f"\nWARNING: centre distance is off by "
                f"{built.centre_distance_error_mm:+.6f} mm."
            )
        if not built.articulates and not args.no_mates:
            print(
                "\nWARNING: the set does not articulate. A member reported fully "
                "defined has had six degrees of freedom removed, not five."
            )

    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception:
        traceback.print_exc()
        sys.exit(1)
