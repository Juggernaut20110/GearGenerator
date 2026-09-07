"""Probe one hypoid tooth-space loft in a live SOLIDWORKS part."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from gears.hypoid.geometry import compute_set, section_cone_distances  # noqa: E402
from gears.hypoid.params import HypoidSetParams  # noqa: E402
from gears.sw import SwError, SwSession  # noqa: E402
from gears.sw.common import draw_curves_3d, loft_cut  # noqa: E402
from gears.sw.hypoid_part import _blank, _guide_points, _section  # noqa: E402
from gears.sw.session import MARK_LOFT_GUIDE  # noqa: E402


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--member", choices=("pinion", "gear"), default="pinion")
    ap.add_argument("--sections", type=int, default=2)
    ap.add_argument("--guide", action="store_true")
    args = ap.parse_args(argv)

    params = HypoidSetParams.with_defaults(
        170.0 / 42.0, 13, 42, offset=15.0, face_width=30.0,
        spiral_angle=50.0, cutter_radius=63.5,
    )
    geo = compute_set(params)
    full = section_cone_distances(geo, args.member, max(2, args.sections))
    distances = full if args.sections > 2 else [full[0], full[-1]]

    try:
        with SwSession() as session:
            model = session.new_part()
            _blank(session.app, model, geo, args.member)
            profiles = [_section(model, geo, args.member, a) for a in distances]
            guides = ()
            if args.guide:
                guide = draw_curves_3d(
                    model,
                    [("hypoid cap guide", _guide_points(geo, args.member, distances))],
                    "hypoid loft guide",
                )
                guides = (guide,)
            loft_cut(model, profiles, guides, guide_mark=MARK_LOFT_GUIDE)
    except SwError as exc:
        print(f"FAIL {args.member}, {len(distances)} sections, guide={args.guide}: {exc}")
        return 1

    print(f"PASS {args.member}, {len(distances)} sections, guide={args.guide}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
