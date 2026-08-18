"""Probe: which way does the gear mate turn the gear? Answer: ask a human.

The idea was to open a built set, spin the pinion about its own axis by a known
angle, and read both components' transforms back. Each member's spin comes
straight out of its own placement matrix - the pinion's is `rot_z(theta)` and
the gear's is `rot_y(S) * rot_z(clocking + phi)`, so peeling off the known
shaft-angle rotation leaves phi, and the ratio settles the sense:

    d(phi) / d(theta) = -z1 / z2      teeth interleave, the pair drives
    d(phi) / d(theta) = +z1 / z2      the mate needs its Reverse flag set

**It does not work, and the reason is the finding.** All three ways of turning
the pinion from here were measured on the anchor set, and every one of them
moved the pinion exactly as asked and left the gear at +0.0000 degrees:

    --drive transform   write Transform2, EditRebuild3
    --drive mate        add an angle mate driving the pinion's last freedom
    --drive drag        IDragOperator: AddComponent, BeginDrag, Drag, EndDrag

A gear mate is applied by the interactive drag solver and nothing else reaches
it. The sense was settled by Carl dragging the pinion in SOLIDWORKS instead:
with the pinion axis selected first and Reverse off, the pair turns correctly.

Kept so nobody spends the afternoon rediscovering those three dead ends, and
because the measurement half is right and would work the moment a drive that
does couple turns up. The assembly is opened and never saved, so nothing this
does is permanent.

    .venv\\Scripts\\python.exe tools\\probe_gear_sense.py out\\mates\\set_m2_z17x43.sldasm
"""

from __future__ import annotations

import argparse
import math
import sys
import traceback
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from gears.bevel import mesh                            # noqa: E402
from gears.sw.session import (                    # noqa: E402
    MARK_MATE_ENTITY,
    RIGHT_PLANE_NAMES,
    SW_ADD_MATE_NO_ERROR,
    SW_MATE_ALIGN_CLOSEST,
    SW_MATE_ANGLE,
    SwError,
    SwSession,
    doubles,
    flag_methods,
    int_byref,
    require,
    select_first,
)

SW_DOC_ASSEMBLY = 2            # swDocumentTypes_e
SW_OPEN_DOC_SILENT = 1         # swOpenDocOptions_e

SPIN_DEG = 20.0                # big enough to read, small enough to stay put


def open_assembly(app, path: Path):
    errors, warnings = int_byref(), int_byref()
    doc = app.OpenDoc6(
        str(path), SW_DOC_ASSEMBLY, SW_OPEN_DOC_SILENT, "", errors, warnings
    )
    if doc is None:
        raise SwError(
            f"could not open {path} (error {errors.value}, warning {warnings.value})"
        )
    return doc


def matrix_of(comp):
    """The component's rotation as rows, from the column-major ArrayData."""
    d = list(comp.Transform2.ArrayData)
    return tuple(tuple(d[3 * j + i] for j in range(3)) for i in range(3))


def translation_of(comp):
    return tuple(list(comp.Transform2.ArrayData)[9:12])


def _rot_z(a: float):
    c, s = math.cos(a), math.sin(a)
    return ((c, -s, 0.0), (s, c, 0.0), (0.0, 0.0, 1.0))


def _rot_y(a: float):
    c, s = math.cos(a), math.sin(a)
    return ((c, 0.0, s), (0.0, 1.0, 0.0), (-s, 0.0, c))


def _mul(a, b):
    return tuple(
        tuple(sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3))
        for i in range(3)
    )


def spin_of(matrix, shaft_angle: float = 0.0) -> float:
    """The rotation about the member's own axis, in radians.

    Undo the shaft-angle tilt first - `rot_y(-S) * M` leaves a plain rotation
    about Z - then read the angle out of its first column.
    """
    m = _mul(_rot_y(-shaft_angle), matrix)
    return math.atan2(m[1][0], m[0][0])


def wrap(a: float) -> float:
    """Nearest representation of an angle to zero, so 359 deg reads as -1."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def find_components(model):
    comps = list(flag_methods(model, "GetComponents").GetComponents(True) or [])
    found = {}
    for comp in comps:
        name = str(comp.Name2).lower()
        for member in ("pinion", "gear"):
            if name.startswith(member):
                found[member] = comp
    missing = {"pinion", "gear"} - set(found)
    if missing:
        raise SwError(
            f"could not find the {', '.join(sorted(missing))} in "
            f"{[str(c.Name2) for c in comps]}"
        )
    return found["pinion"], found["gear"]


def spin_by_transform(session, model, comp, delta: float) -> None:
    """Turn the pinion by writing its transform, the way `build_set` places it.

    Measured: the pinion goes exactly where it is put and the gear does not
    move at all. A gear mate is a coupling the *solver* applies, so it acts on
    a drag or on a re-solve - writing a transform and rebuilding walks straight
    past it. Kept here because that is worth knowing: it is also why the
    clocking written by `build_set` survives having a gear mate on top of it.
    """
    matrix = _mul(_rot_z(delta), matrix_of(comp))
    data = mesh.to_array_data(matrix, translation=translation_of(comp))
    xform = session.math_utility().CreateTransform(doubles(data))
    if xform is None:
        raise SwError("could not build the transform for the spin")
    comp.Transform2 = xform
    model.EditRebuild3()


def spin_by_drag(session, model, comp, delta: float) -> None:
    """Turn the pinion through the drag operator - the closest thing to a hand.

    Measured on the anchor set: the pinion turns by exactly the angle asked for
    and the gear stays where it is. So not even this reaches the gear mate.
    """
    drag = require(
        flag_methods(model, "GetDragOperator").GetDragOperator(),
        "get the drag operator",
    )
    flag_methods(drag, "BeginDrag", "EndDrag", "Drag", "AddComponent")
    drag.AddComponent(comp, False)
    drag.BeginDrag()
    try:
        xform = require(
            session.math_utility().CreateTransform(
                doubles(mesh.to_array_data(_rot_z(delta)))
            ),
            "build the drag transform",
        )
        drag.Drag(xform)
    finally:
        drag.EndDrag()
    model.EditRebuild3()


def _component_plane(comp, names):
    """One of a component's default planes, by name."""
    for name in names:
        try:
            feat = comp.FeatureByName(name)
        except Exception:
            feat = None
        if feat is not None:
            return feat
    raise SwError(f"none of {names} is a plane in {comp.Name2}")


def spin_by_mate(session, model, comp, delta: float) -> None:
    """Turn the pinion by constraining it, so the solver has to do the work.

    An angle mate between the pinion's Right plane and the assembly's forces
    the one degree of freedom the pinion has left, which runs the whole mate
    set rather than walking past it. Measured: the pinion goes to the angle
    given - negated, since the alignment resolved the other way - and the gear
    still does not move.

    The mate is temporary in the only sense that matters: the caller never
    saves the assembly.
    """
    plane = _component_plane(comp, RIGHT_PLANE_NAMES)
    model.ClearSelection2(True)
    if not plane.Select2(False, MARK_MATE_ENTITY):
        raise SwError("could not select the pinion Right plane")
    select_first(model, RIGHT_PLANE_NAMES, "PLANE", append=True,
                 mark=MARK_MATE_ENTITY)

    errors = int_byref()
    mate = model.AddMate5(
        SW_MATE_ANGLE, SW_MATE_ALIGN_CLOSEST, False,
        0.0, 0.0, 0.0,
        1.0, 1.0,
        float(delta), 0.0, 0.0,
        False, False, 0,
        errors,
    )
    model.ClearSelection2(True)
    if mate is None or int(errors.value) != SW_ADD_MATE_NO_ERROR:
        raise SwError(
            f"could not add the driving angle mate (status {errors.value})"
        )
    model.EditRebuild3()


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("assembly", nargs="?",
                    default=r"out\mates\set_m2_z17x43.sldasm")
    ap.add_argument("--z1", type=int, default=17)
    ap.add_argument("--z2", type=int, default=43)
    ap.add_argument("--sigma", type=float, default=90.0)
    ap.add_argument("--spin", type=float, default=SPIN_DEG)
    ap.add_argument(
        "--drive", choices=("drag", "mate", "transform"), default="drag",
        help="how to turn the pinion: through the solver, or by writing its "
             "transform (which does not couple - see spin_by_transform)",
    )
    args = ap.parse_args(argv)

    path = Path(args.assembly).resolve()
    if not path.exists():
        print(f"no such assembly: {path}")
        return 1

    sigma = math.radians(args.sigma)
    expected = -args.z1 / args.z2

    with SwSession() as session:
        model = open_assembly(session.app, path)
        flag_methods(model, "EditRebuild3", "GetComponents")
        try:
            pinion, gear = find_components(model)

            before = (
                spin_of(matrix_of(pinion)),
                spin_of(matrix_of(gear), sigma),
            )
            drive = {
                "drag": spin_by_drag,
                "mate": spin_by_mate,
                "transform": spin_by_transform,
            }[args.drive]
            drive(session, model, pinion, math.radians(args.spin))
            after = (
                spin_of(matrix_of(pinion)),
                spin_of(matrix_of(gear), sigma),
            )

            d_pinion = wrap(after[0] - before[0])
            d_gear = wrap(after[1] - before[1])

            print(f"assembly   {path.name}")
            print(f"asked for  {args.spin:+.4f} deg on the pinion")
            print(f"pinion     {math.degrees(d_pinion):+.4f} deg measured")
            print(f"gear       {math.degrees(d_gear):+.4f} deg measured")

            if abs(d_pinion) < 1e-9:
                print("\nthe pinion did not move - the transform write was "
                      "ignored, so this tells us nothing")
                return 1
            if abs(d_gear) < 1e-9:
                print("\nthe gear did not follow at all - either the gear mate "
                      "is missing or the pinion is not actually free")
                return 1

            measured = d_gear / d_pinion
            print(f"\nratio      {measured:+.6f} measured")
            print(f"           {expected:+.6f} wanted (-z1/z2, opposite senses)")
            if abs(measured - expected) < 1e-3:
                print("\nthe gear mate as built turns the pair the right way.")
            elif abs(measured + expected) < 1e-3:
                print("\nthe gear mate as built turns the pair the WRONG way - "
                      "it needs its Reverse flag set.")
            else:
                print("\nthe magnitude does not match the tooth ratio either "
                      "way; the mate is not doing what it should.")
        finally:
            # Never saved, so the spin and anything else here is discarded.
            session.app.CloseDoc(model.GetTitle)
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception:
        traceback.print_exc()
        sys.exit(1)
