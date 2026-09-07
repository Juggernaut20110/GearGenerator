"""Sweep a floating live SOLIDWORKS hypoid assembly for contact clearance.

The active assembly must have been built with ``build_hypoid_set.py --no-mates``.
This development probe preserves the requested shaft angle and axis offset,
then reports interference while shifting the wheel in the contact-normal XZ
direction and optionally changing its tooth clocking.
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from gears.hypoid import mesh  # noqa: E402
from gears.hypoid.geometry import compute_set  # noqa: E402
from gears.hypoid.params import HypoidSetParams  # noqa: E402
from gears.sw.assembly_common import check_interference, place  # noqa: E402
from gears.sw.session import SwSession  # noqa: E402


def _values(text: str) -> list[float]:
    return [float(value) for value in text.split(",")]


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--module", type=float, default=170.0 / 42.0)
    ap.add_argument("--z1", type=int, default=13)
    ap.add_argument("--z2", type=int, default=42)
    ap.add_argument("--sigma", type=float, default=90.0)
    ap.add_argument("--offset", type=float, default=15.0)
    ap.add_argument("--face-width", type=float, default=30.0)
    ap.add_argument("--spiral", type=float, default=50.0)
    ap.add_argument("--cutter-radius", type=float, default=63.5)
    ap.add_argument("--separation", default="-2,-1.5,-1,-.5,0,.5,1,1.5,2,2.5,3")
    ap.add_argument("--clock-delta", default="0", help="comma-separated degrees")
    args = ap.parse_args(argv)

    params = HypoidSetParams.with_defaults(
        args.module, args.z1, args.z2,
        face_width=args.face_width, shaft_angle=args.sigma, offset=args.offset,
        spiral_angle=args.spiral, cutter_radius=args.cutter_radius,
    )
    geo = compute_set(params)
    base_translation = mesh.gear_translation(geo)
    base_clock = mesh.gear_clocking(geo)
    theta1, _ = mesh.contact_azimuths(geo)
    nx = math.cos(geo.pinion.pitch_angle) * math.cos(theta1)
    nz = -math.sin(geo.pinion.pitch_angle)
    norm = math.hypot(nx, nz)
    direction = (nx / norm, 0.0, nz / norm)

    with SwSession() as session:
        model = session.app.ActiveDoc
        if model is None or int(model.GetType) != 2:
            raise RuntimeError("the active SOLIDWORKS document is not an assembly")
        components = list(model.GetComponents(False) or [])
        pinion = next(c for c in components if "pinion" in c.GetPathName.lower())
        gear = next(c for c in components if "gear" in c.GetPathName.lower())
        place(session, pinion, mesh.pinion_placement(geo), "pinion")

        print(
            "separation_mm,clock_delta_deg,interference_count,"
            "interference_volume_mm3"
        )
        for separation in _values(args.separation):
            translation = tuple(
                base_translation[i] + separation * direction[i]
                for i in range(3)
            )
            for delta_deg in _values(args.clock_delta):
                clock = base_clock + math.radians(delta_deg)
                place(
                    session, gear,
                    mesh.gear_placement(params.sigma, clock), "gear",
                    translation=translation,
                )
                # Late-bound SW2026 exposes this no-argument method as a
                # property; reading it performs the rebuild.
                model.EditRebuild3
                count, volume = check_interference(model)
                print(f"{separation:g},{delta_deg:g},{count},{volume:.9g}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
