"""Answer the guide-curve loft questions before a build depends on them.

    .venv\\Scripts\\python.exe tools\\probe_helix_loft.py
    .venv\\Scripts\\python.exe tools\\probe_helix_loft.py --marks 2 4 8

Four questions, each on its own throwaway part, in the same spirit as
`probe_dimension.py` and `probe_mate.py` - answered in isolation so that when
`build_spur` fails it is not also the thing under test.

1. **Which selection mark does `InsertCutBlend` want for a guide curve?**
   Profiles use MARK_LOFT_PROFILE = 1, which was read out of swconst.tlb.
   Guide curves have no enum: the mark is just a number the feature looks for,
   and 2 is what the API reference gives. This tries 2, then every other small
   mark, and reports which ones produce a cut whose shape actually follows the
   guide.

2. **Does a sampled 3D spline work as a guide, or must it be a real helix?**
   `build_spur` writes its guide as a spline through sampled points, because
   that needs no agreement with SOLIDWORKS about pitch, start angle or hand.
   If only `InsertHelix` is accepted, that has to change.

3. **Does a guided loft cut still survive the circular pattern?**
   `GeometryPattern` must be on for a loft cut driven by 3D sketch profiles -
   off, SOLIDWORKS re-solves the cut at every instance and rejects the pattern
   outright. Whether a guide changes that answer is cheaper to measure than to
   reason about.

4. **May the guide overrun the profiles, or must it stop on them?**
   Both sections are pushed past the end faces by `end_overshoot`, and the
   guide currently spans exactly the same range. If it must stop short - or
   must run longer - that is a one-line change, but only if it is known.

Nothing here builds a real gear. Each case is the smallest thing that can
answer its question, so a failure names the API behaviour and not the geometry.
"""

from __future__ import annotations

import argparse
import math
import sys
import traceback
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from gears.spur.geometry import (                            # noqa: E402
    compute_set,
    end_overshoot,
    guide_helix,
    tooth_space_section,
)
from gears.spur.params import SpurSetParams                  # noqa: E402
from gears.sw import SwError, SwSession                      # noqa: E402
from gears.sw.common import (                                # noqa: E402
    create_axis,
    draw_curves_3d,
    drop_offcut,
    pattern_teeth,
    select_feature,
)
from gears.sw.session import MARK_LOFT_PROFILE, SW_SOLID_BODY  # noqa: E402
from gears.sw.spur_part import _section_curves, build_blank  # noqa: E402

# A small helical set, so a failure is quick and the twist is unmistakable.
PROBE = SpurSetParams.with_defaults(2.0, 17, 43, helix_angle=20.0)


def _volume(model) -> float:
    """Total solid volume, mm3. The cheapest shape measurement there is."""
    bodies = model.GetBodies2(SW_SOLID_BODY, True)
    bodies = list(bodies) if bodies else []
    return sum(b.GetMassProperties(1.0)[3] for b in bodies) * 1e9


def _blank_with_sections(session, geo, member, split_cap, z_span=None):
    """A blank plus the two end sections, ready for a loft. Returns the pieces."""
    model = session.new_part()
    axis = create_axis(model)
    build_blank(session.app, model, geo, member)

    overshoot = end_overshoot(geo)
    z_lo, z_hi = -overshoot, geo.params.face_width + overshoot

    sections = []
    for z in (z_lo, z_hi):
        section = tooth_space_section(geo, member, z=z, split_cap=split_cap)
        sections.append(
            draw_curves_3d(model, _section_curves(section), f"section z={z:.3f}")
        )
    guide_span = z_span if z_span is not None else (z_lo, z_hi)
    return model, axis, sections, guide_span


def _try_loft(model, profiles, guide, mark):
    """One InsertCutBlend attempt. Returns (cut_feature | None, message)."""
    model.ClearSelection2(True)
    for i, sketch in enumerate(profiles):
        select_feature(sketch, f"profile {i}", append=i > 0, mark=MARK_LOFT_PROFILE)
    if guide is not None:
        select_feature(guide, "guide", append=True, mark=mark)
    try:
        cut = model.FeatureManager.InsertCutBlend(
            False, False, False, 1.0, 0, 0, False, 0.0, 0.0, 0, True, True
        )
    except Exception as exc:                       # noqa: BLE001 - reporting a probe
        return None, f"raised {type(exc).__name__}: {exc}"
    if cut is None:
        return None, "InsertCutBlend returned None"
    return cut, "ok"


# --- question 1 + 2: which mark, and does a sampled spline serve? -----------


def probe_guide_mark(session, marks) -> dict:
    """Loft with the guide under each candidate mark; compare the volume removed.

    The unguided loft is the control. A guide that is genuinely being used
    changes how much material comes out, because the ruled surface between two
    rotated sections is not the helicoid - it cuts inside it. A mark that
    SOLIDWORKS ignores gives exactly the control's volume back.
    """
    geo = compute_set(PROBE)
    results = {}

    model, _, profiles, span = _blank_with_sections(session, geo, "pinion", True)
    blank_volume = _volume(model)
    cut, message = _try_loft(model, profiles, None, 0)
    control = None
    if cut is not None:
        drop_offcut(model)
        control = blank_volume - _volume(model)
    results["no guide"] = (message, control)
    session.close(model)

    for mark in marks:
        model, _, profiles, span = _blank_with_sections(session, geo, "pinion", True)
        guide = draw_curves_3d(
            model,
            [("guide", guide_helix(geo, "pinion", span[0], span[1]))],
            "guide helix",
        )
        blank_volume = _volume(model)
        cut, message = _try_loft(model, profiles, guide, mark)
        removed = None
        if cut is not None:
            drop_offcut(model)
            removed = blank_volume - _volume(model)
        results[f"mark {mark}"] = (message, removed)
        session.close(model)

    return results


# --- question 3: does a guided cut still pattern? ---------------------------


def probe_pattern_survives(session, mark: int) -> str:
    geo = compute_set(PROBE)
    model, axis, profiles, span = _blank_with_sections(session, geo, "pinion", True)
    guide = draw_curves_3d(
        model,
        [("guide", guide_helix(geo, "pinion", span[0], span[1]))],
        "guide helix",
    )
    cut, message = _try_loft(model, profiles, guide, mark)
    if cut is None:
        session.close(model)
        return f"loft itself failed: {message}"

    drop_offcut(model)
    try:
        pattern_teeth(model, cut, axis, geo.pinion.z)
    except SwError as exc:
        session.close(model)
        return f"pattern REFUSED: {exc}"

    bodies = model.GetBodies2(SW_SOLID_BODY, True)
    faces = list(bodies)[0].GetFaces() if bodies else []
    session.close(model)
    return f"pattern ok, {len(faces) if faces else 0} faces for {geo.pinion.z} teeth"


# --- question 4: how far may the guide run? --------------------------------


def probe_guide_span(session, mark: int) -> dict:
    """Try a guide that overruns the profiles, one that stops exactly on them,
    and one that stops short. Whichever are accepted, the builder may use."""
    geo = compute_set(PROBE)
    overshoot = end_overshoot(geo)
    b = geo.params.face_width
    spans = {
        "exactly on the profiles": (-overshoot, b + overshoot),
        "overrunning by 2 mm": (-overshoot - 2.0, b + overshoot + 2.0),
        "stopping 0.5 mm short": (-overshoot + 0.5, b + overshoot - 0.5),
    }

    results = {}
    for label, (z_lo, z_hi) in spans.items():
        model, _, profiles, _ = _blank_with_sections(session, geo, "pinion", True)
        guide = draw_curves_3d(
            model,
            [("guide", guide_helix(geo, "pinion", z_lo, z_hi))],
            "guide helix",
        )
        cut, message = _try_loft(model, profiles, guide, mark)
        results[label] = message
        session.close(model)
    return results


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument(
        "--marks",
        type=int,
        nargs="+",
        default=[2, 4, 8, 16],
        help="candidate selection marks for the guide curve",
    )
    ap.add_argument(
        "--mark",
        type=int,
        default=2,
        help="mark to use for questions 3 and 4 (default: the documented 2)",
    )
    args = ap.parse_args(argv)

    geo = compute_set(PROBE)
    print(
        f"probe set: m={PROBE.module} z={PROBE.z1} beta={PROBE.helix_angle} deg, "
        f"twist {math.degrees(geo.pinion.twist):.3f} deg over "
        f"{PROBE.face_width:.3f} mm of face"
    )

    with SwSession() as session:
        print("\n1+2. GUIDE CURVE MARK  (a used guide changes the volume removed)")
        for label, (message, removed) in probe_guide_mark(session, args.marks).items():
            volume = f"{removed:9.3f} mm3" if removed is not None else "        -"
            print(f"     {label:<12} {volume}   {message}")

        print(f"\n3.   PATTERN AFTER A GUIDED CUT  (mark {args.mark})")
        print(f"     {probe_pattern_survives(session, args.mark)}")

        print(f"\n4.   GUIDE SPAN  (mark {args.mark})")
        for label, message in probe_guide_span(session, args.mark).items():
            print(f"     {label:<26} {message}")

    print(
        "\nRecord the answers in gears/sw/session.py next to MARK_LOFT_GUIDE and in\n"
        "the README, the way probe_gear_sense.py's answers were recorded."
    )
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception:
        traceback.print_exc()
        sys.exit(1)
