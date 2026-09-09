"""Build one involute spur gear in SOLIDWORKS.

    .venv\\Scripts\\python.exe tools\\build_spur.py --module 2 --z1 17 --z2 43
    .venv\\Scripts\\python.exe tools\\build_spur.py --module 2 --z1 17 --z2 43 \\
        --beta 15 --member gear --save out\\spur_gear.sldprt

A helical set is worth building both members of: they are cut with opposite
hands, and a build that quietly gave them the same one would look right on its
own and refuse to mesh.
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
from gears.sw.spur_part import build_spur               # noqa: E402


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
    ap.add_argument(
        "--x1", type=float, default=0.0,
        help="pinion profile-shift coefficient x1 (normal-module units)",
    )
    ap.add_argument(
        "--x2", type=float, default=0.0,
        help="gear profile-shift coefficient x2 (normal-module units)",
    )
    ap.add_argument("--member", choices=("pinion", "gear"), default="pinion")
    ap.add_argument("--save", help="path to save the part to")
    ap.add_argument("--close", action="store_true", help="close the part afterwards")
    args = ap.parse_args(argv)

    overrides = {
        "pressure_angle": args.alpha,
        "helix_angle": args.beta,
        "hand": args.hand,
        "profile_shift_1": args.x1,
        "profile_shift_2": args.x2,
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
    m = geo.member(args.member)
    print(
        f"building {args.member}: m_n={p.module} z={m.z} "
        f"beta={m.helix_angle_deg:.3f} deg {m.hand}-hand, "
        f"twist {m.twist_deg:.3f} deg over {p.face_width:.3f} mm"
    )

    save_path = str(Path(args.save).resolve()) if args.save else None
    if save_path:
        Path(save_path).parent.mkdir(parents=True, exist_ok=True)

    with SwSession() as session:
        try:
            built = build_spur(session, geo, args.member, save_path)
        except SwError as exc:
            print(f"\nBUILD FAILED: {exc}")
            return 1

        print("\nRESULT")
        print(f"  document        {built.title}")
        print(f"  solid bodies    {built.body_count}")
        print(f"  faces           {built.face_count}")
        print(f"  teeth requested {built.teeth}")
        print(
            "  bounding box    "
            + ", ".join(f"{v:.3f}" for v in built.box_mm)
            + "  mm"
        )
        # GetBodyBox is tessellation-based and only guaranteed to *contain* the
        # body, so treat a sub-percent discrepancy as agreement, not as error.
        print(f"  max radius      {built.max_radius_mm:.3f} mm (box is approximate)")
        print(f"  expected r_a    {built.expected_radius_mm:.3f} mm")
        print(f"  error           {built.radius_error_pct:+.2f} %")
        if built.path:
            print(f"  saved to        {built.path}")

        # A blank alone has roughly 6 faces; each tooth space adds several.
        if built.face_count < 4 * built.teeth:
            print(
                f"\nWARNING: only {built.face_count} faces for {built.teeth} teeth - "
                "the pattern may not have cut every tooth."
            )
        if abs(built.radius_error_pct) > 3.0:
            print("\nWARNING: outside diameter is off by more than 3%.")

        if args.close:
            session.close(built.model)

    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception:
        traceback.print_exc()
        sys.exit(1)
