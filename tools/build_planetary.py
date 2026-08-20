"""Build a complete planetary gear set in SOLIDWORKS.

    .venv\\Scripts\\python.exe tools\\build_planetary.py --z-sun 24 --z-planet 18
    .venv\\Scripts\\python.exe tools\\build_planetary.py --z-sun 24 --z-planet 18 \\
        --planets 4 --beta 15
    .venv\\Scripts\\python.exe tools\\build_planetary.py --no-mates

Three parts - a sun, one planet and a ring - and an assembly with the planet
inserted as many times as asked for, every member clocked so the whole train
meshes at once, and gear mates coupling it.

The ring's tooth count is **not** an input. `z_ring = z_sun + 2 z_planet` is
forced by the sun-planet and planet-ring centre distances having to be the same
distance, so asking for a different ring is asking for two centre distances at
once.

`--planets` is limited by the **assembly condition**: `(z_sun + z_ring)` has to
divide by it, or there is no ring clocking that meshes with all of them and the
last planet cannot be fitted. The validator says which counts would work.

**The carrier is held, not modelled.** Every member spins about an axis fixed in
space, which is the configuration that gives the ring-to-sun ratio of -z_r/z_s.
A carrier part with orbiting planets is a different assembly and is not built
here.

**The gear mates' senses are measured, and they differ.** Dragging the sun on
the anchor set: sun:planet is right with Reverse off, and planet:ring needed
Reverse *on* - it turned the ring backwards without it. Each is a named constant
in `gears/sw/planetary_assembly.py`, and `--reverse-gear` flips both *away* from
those measured values, so it is the escape hatch for the day one stops holding
rather than a setting with a right value of its own.
"""

from __future__ import annotations

import argparse
import math
import sys
import traceback
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from gears.planetary import mesh                              # noqa: E402
from gears.planetary.geometry import compute_set              # noqa: E402
from gears.planetary.params import PlanetarySetParams         # noqa: E402
from gears.planetary.validate import validate                 # noqa: E402
from gears.sw import SwError, SwSession                       # noqa: E402
from gears.sw.planetary_assembly import build_planetary_set   # noqa: E402


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--module", type=float, default=2.0, help="normal module, mm")
    ap.add_argument("--z-sun", type=int, default=24)
    ap.add_argument("--z-planet", type=int, default=18)
    ap.add_argument("--planets", type=int, default=3)
    ap.add_argument("--alpha", type=float, default=20.0, help="normal pressure angle")
    ap.add_argument("--beta", type=float, default=0.0, help="helix angle, deg")
    ap.add_argument("--hand", choices=("right", "left"), default="right")
    ap.add_argument("--face-width", type=float)
    ap.add_argument("--bore", type=float, help="the SUN's bore, mm")
    ap.add_argument("--hub", type=float)
    ap.add_argument("--rim", type=float, help="ring rim thickness, mm")
    ap.add_argument("--backlash", type=float)
    ap.add_argument("--out", default="out", help="directory for the parts and assembly")
    ap.add_argument("--no-save", action="store_true", help="do not save the assembly")
    ap.add_argument(
        "--no-mates", action="store_true", help="place but do not constrain"
    )
    ap.add_argument(
        "--reverse-gear", action="store_true",
        help="flip both meshes away from their measured senses",
    )
    args = ap.parse_args(argv)

    overrides = {
        "pressure_angle": args.alpha,
        "helix_angle": args.beta,
        "hand": args.hand,
        "n_planets": args.planets,
    }
    for key, value in (
        ("face_width", args.face_width),
        ("bore", args.bore),
        ("hub_thickness", args.hub),
        ("rim_thickness", args.rim),
        ("backlash", args.backlash),
    ):
        if value is not None:
            overrides[key] = value

    p = PlanetarySetParams.with_defaults(
        args.module, args.z_sun, args.z_planet, **overrides
    )
    result = validate(p)
    for issue in result.warnings:
        print(f"WARNING  {issue}")
    if not result.ok:
        for issue in result.errors:
            print(f"ERROR    {issue}")
        return 1

    geo = compute_set(p)
    print(
        f"building sun {p.z_sun} / {p.n_planets} planets of {p.z_planet} / "
        f"ring {p.z_ring} at m_n={p.module}, orbit radius "
        f"{geo.centre_distance:.4f} mm"
    )
    print(
        f"  assembly condition: {p.z_sun} + {p.z_ring} = {p.z_sun + p.z_ring}, "
        f"divisible by {p.n_planets}"
    )
    print(
        f"  ratios: {p.ratio_carrier_to_sun:.4f}:1 with the ring held, "
        f"{p.ratio_ring_to_sun:.4f}:1 with the carrier held"
    )
    if p.helix_angle:
        print(
            f"  hands: sun {geo.sun.hand}, planets {geo.planet.hand}, "
            f"ring {geo.ring.hand} (sun opposes the planets, ring matches them)"
        )
    print("  clockings:")
    for k in range(p.n_planets):
        x, y, _ = mesh.planet_translation(geo, k)
        print(
            f"    planet {k}: at ({x:9.4f}, {y:9.4f}), "
            f"clocked {math.degrees(mesh.planet_clocking(geo, k)) % 360.0:9.4f} deg"
        )
    print(
        f"    ring    : clocked "
        f"{math.degrees(mesh.ring_clocking(geo, 0)) % 360.0:9.4f} deg"
    )

    with SwSession() as session:
        try:
            built = build_planetary_set(
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
        for part in built.parts:
            print(
                f"  {part.member:<7} {part.teeth:>3} teeth  {part.face_count:>4} faces  "
                f"r {part.max_radius_mm:7.3f} mm ({part.radius_error_pct:+.2f} %)"
            )
            if part.path:
                print(f"          saved to {part.path}")
        print(f"  the planet part is inserted {built.planet_count} times")

        print("\nASSEMBLY")
        print(f"  document              {built.assembly_title}")
        print(f"  orbit radius          {built.centre_distance_mm:.4f} mm")
        for k, (x, y, z) in enumerate(built.measured_positions_mm):
            print(
                f"  planet {k} measured at ({x:9.4f}, {y:9.4f}, {z:8.4f}), "
                f"radius {math.hypot(x, y):.4f} mm"
            )
        print(f"  worst position error  {built.worst_position_error_mm:+.6f} mm")
        print(
            "  axes vs the sun's     "
            + ", ".join(f"{a:.6f}" for a in built.measured_axis_angles_deg)
            + " deg (parallel is 0)"
        )
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
        for member, status in built.statuses.items():
            print(f"  {member} is {status}")
        print(f"  articulates: {'yes' if built.articulates else 'NO'}")
        if not args.no_mates:
            # The coupling only acts on a hand drag - no API route runs it - so
            # this is the one result the build cannot check for itself.
            print(
                "  sense: sun:planet Reverse off, planet:ring Reverse on, "
                "both verified by hand on the anchor set"
            )
            if built.gear_mate_reversed:
                print(
                    "  WARNING: --reverse-gear is on, so both meshes run "
                    "against the verified sense"
                )

        if abs(built.worst_position_error_mm) > 1e-6:
            print(
                f"\nWARNING: a planet is off its orbit radius by "
                f"{built.worst_position_error_mm:+.6f} mm."
            )
        if any(a > 1e-6 for a in built.measured_axis_angles_deg):
            print(
                "\nWARNING: some axis is not parallel to the sun's, but every "
                "axis in a planetary train should be."
            )
        if not built.articulates and not args.no_mates:
            print(
                "\nWARNING: the train does not articulate. A member reported "
                "fully defined has had six degrees of freedom removed, not five."
            )

    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception:
        traceback.print_exc()
        sys.exit(1)
