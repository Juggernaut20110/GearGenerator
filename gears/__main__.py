"""Dump derived gear geometry to the terminal, with no SOLIDWORKS involved.

    .venv\\Scripts\\python.exe -m gears --module 2 --z1 17 --z2 43
    .venv\\Scripts\\python.exe -m gears --module 2 --z1 17 --z2 43 --spiral 35
    .venv\\Scripts\\python.exe -m gears --type spur --module 2 --z1 17 --z2 43 --beta 15
    .venv\\Scripts\\python.exe -m gears --type spur --module 2 --z1 18 --z2 60 --internal
    .venv\\Scripts\\python.exe -m gears --type planetary --module 2 --z1 24 --z2 18
    .venv\\Scripts\\python.exe -m gears --module 2 --z1 17 --z2 43 \\
        --csv out/pinion_outer.csv --member pinion --end outer

`--type` picks the gear type and defaults to bevel, which is what the tool
generated before there was a choice. Flags belonging to another type are refused
rather than ignored, so a mistyped `--sigma` on a spur set is not silently
dropped - and a flag two types genuinely share, like `--beta`, is refused only
by the third.

For a planetary set `--z1` and `--z2` are the sun and the planet; `--z-sun` and
`--z-planet` say the same thing more plainly. The ring is derived from them and
has no flag of its own.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from .bevel import report as bevel_report
from .hypoid import report as hypoid_report
from .planetary import report as planetary_report
from .spur import report as spur_report

TYPES = {
    "bevel": bevel_report,
    "hypoid": hypoid_report,
    "spur": spur_report,
    "planetary": planetary_report,
}

# Flags every type shares, and so never belong to only one of them.
#
# `hand` joined this list when spiral bevel arrived. It means the same thing in
# every type - which way the first member's teeth wind, with the rest following
# - so it is defined once here rather than twice, which argparse would refuse
# anyway.
COMMON_FLAGS = ("type", "module", "z1", "z2", "csv", "hand")


def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(prog="gears", description=__doc__)
    ap.add_argument("--type", choices=sorted(TYPES), default="bevel")
    ap.add_argument("--module", type=float, required=True, help="module, mm")
    ap.add_argument(
        "--z1", type=int, required=True,
        help="pinion tooth count (the SUN, for a planetary set)",
    )
    ap.add_argument(
        "--z2", type=int, required=True,
        help="gear tooth count (a PLANET, for a planetary set)",
    )
    ap.add_argument("--alpha", type=float, default=20.0, help="pressure angle, deg")
    ap.add_argument("--face-width", type=float, help="mm (default: auto)")
    ap.add_argument("--bore", type=float, help="mm (default: auto)")
    ap.add_argument("--hub", type=float, help="hub/backing thickness, mm (default: auto)")
    # Every member name of every type. Which ones are meaningful depends on
    # the type, and each report module says so - a planetary set has a sun,
    # planets and a ring, and maps the pair-shaped default onto its own.
    ap.add_argument(
        "--member",
        choices=("pinion", "gear", "sun", "planet", "ring"),
        default="pinion",
    )
    ap.add_argument(
        "--hand",
        choices=("right", "left"),
        default="right",
        help="the PINION's hand; the gear always takes the other",
    )
    ap.add_argument("--csv", type=Path, help="write the section's points to this file")

    for module in TYPES.values():
        module.add_arguments(ap)
    return ap


def _reject_foreign_flags(ap, args, argv) -> None:
    """Refuse a flag that belongs to a different gear type.

    Checked against what was actually typed rather than against the parsed
    values, because a flag with a default is indistinguishable from an unset one
    after parsing. `--sigma 90` on a spur set means the user believes something
    untrue about what they are building, and saying so beats ignoring it.
    """
    own = set(COMMON_FLAGS) | set(TYPES[args.type].FLAGS)
    for other, module in TYPES.items():
        if other == args.type:
            continue
        for flag in module.FLAGS:
            if flag in own:
                continue
            typed = "--" + flag.replace("_", "-")
            if any(a == typed or a.startswith(typed + "=") for a in argv):
                ap.error(f"{typed} is a {other} option; this is a {args.type} set")


def main(argv=None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    ap = build_parser()
    args = ap.parse_args(argv)
    _reject_foreign_flags(ap, args, argv)

    geo, section, loop3d = TYPES[args.type].report(args)
    if geo is None:
        return 1

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
