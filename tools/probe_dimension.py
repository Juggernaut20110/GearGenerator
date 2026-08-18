"""Probe the dimensioning API before trusting it inside a build.

The type library tells us these calls exist and what their enum values are. It
does not tell us how they behave under late binding, and these can only be
settled by asking SOLIDWORKS:

* does `ISketchSegment.Select4` marshal, or does win32com resolve it wrongly
* are the polyline corners separate points, or does a line started where the
  last one ended share its point even with `AddToDB` on
* is `AddDimension2`'s position argument in model space or sketch space
* which of the four angles around the intersection does an angular dimension
  land on, given where the text is placed
* does binding a dimension to a global variable leave the sketch fully defined,
  and does editing that variable actually move the geometry

Run it with the venv interpreter:

    .venv\\Scripts\\python.exe tools\\probe_dimension.py

It builds only the blank sketch - no revolve, no teeth - then deliberately
drives the crown radius to a new value from its equation, and leaves the part
open and unsaved so the result can be inspected by hand. The crown will be at
the test value, not the gear's own, which is the point.
"""

import math
import sys
import traceback
from pathlib import Path

import pythoncom

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from bevelgear.geometry import blank_outline, compute_set          # noqa: E402
from bevelgear.params import BevelSetParams                        # noqa: E402
from bevelgear.sw.part import (                                    # noqa: E402
    _angle_position,
    _axis_apex,
    _constrain_blank,
    _dimension_blank,
    _endpoint_methods,
    _link_blank_equations,
    _model_to_sketch,
)
from bevelgear.sw.session import (                                 # noqa: E402
    CONSTRAINED_STATUS_NAMES,
    NULL_DISPATCH,
    SW_DIM_DRIVING,
    SW_FULLY_CONSTRAINED,
    TOP_PLANE_NAMES,
    SketchFlags,
    SwSession,
    equation_manager,
    flag_methods,
    mm,
    require,
    select_first,
    value_of,
)

MEMBER = "pinion"
CROWN_TEST_MM = 22.0    # a value the crown is definitely not already at
stage_no = 0
failures = 0


def stage(label):
    global stage_no
    stage_no += 1
    print(f"\n[{stage_no}] {label}")


def ok(msg):
    print(f"    PASS  {msg}")


def fail(msg):
    global failures
    failures += 1
    print(f"    FAIL  {msg}")


def check(condition, good, bad):
    if condition:
        ok(good)
    else:
        fail(bad)
    return condition


def draw_blank_sketch(model, outline):
    """The same drawing pass `build_blank` does, and nothing else."""
    mgr = model.SketchManager
    z_hi = max(z for _, z in outline)

    model.ClearSelection2(True)
    select_first(model, TOP_PLANE_NAMES, "PLANE")
    mgr.InsertSketch(True)
    to_sketch = _model_to_sketch(mgr.ActiveSketch)

    with SketchFlags(model):
        c1 = to_sketch(0.0, 0.0, 0.0)
        c2 = to_sketch(0.0, 0.0, mm(z_hi + 5.0))
        axis = _endpoint_methods(
            require(
                mgr.CreateCenterLine(c1[0], c1[1], 0.0, c2[0], c2[1], 0.0), "centreline"
            )
        )
        lines = []
        closed = outline + [outline[0]]
        for i, ((r1, za), (r2, zb)) in enumerate(zip(closed, closed[1:])):
            a = to_sketch(mm(r1), 0.0, mm(za))
            b = to_sketch(mm(r2), 0.0, mm(zb))
            lines.append(
                _endpoint_methods(
                    require(
                        mgr.CreateLine(a[0], a[1], 0.0, b[0], b[1], 0.0), f"segment {i}"
                    )
                )
            )
    return axis, lines


def main():
    p = BevelSetParams(
        module=2.0, z1=18, z2=24, face_width=12.0, bore=8.0, hub_thickness=0.0
    )
    geo = compute_set(p)
    m = geo.member(MEMBER)
    outline = blank_outline(geo, MEMBER)

    with SwSession() as session:
        stage("Draw the blank outline on the Top Plane")
        model = session.new_part()
        axis, lines = draw_blank_sketch(model, outline)
        sketch = model.SketchManager.ActiveSketch
        ok(f"{len(lines)} outline segments plus one centreline")

        stage("Select4 under late binding")
        model.ClearSelection2(True)
        if not check(
            bool(lines[0].Select4(False, NULL_DISPATCH)),
            "ISketchSegment.Select4 marshals with the null VARIANT callout",
            "ISketchSegment.Select4 would not marshal - needs flag_methods",
        ):
            return 1
        model.ClearSelection2(True)

        stage("Are the polyline corners shared points?")
        points = list(value_of(sketch, "GetSketchPoints2") or [])
        print(f"    {len(points)} sketch points for {len(lines)} lines + 1 centreline")
        check(
            len(points) == len(lines) + 2,
            "corners are shared points - AddToDB does not stop a polyline "
            "closing on itself, so no merge pass is needed",
            f"expected {len(lines) + 2} points; the corners are not shared and "
            "the profile needs an sgMERGEPOINTS pass",
        )
        corner = lines[0].GetEndPoint2(), lines[1].GetStartPoint2()
        check(
            corner[0]._oleobj_ == corner[1]._oleobj_,
            "adjacent segments hand back the same point object",
            "adjacent segments hand back different point objects",
        )

        stage("Centreline apex endpoint")
        apex = _axis_apex(axis)
        check(
            math.hypot(apex.X, apex.Y) < 1e-9,
            f"apex endpoint is at the sketch origin ({apex.X:.3e}, {apex.Y:.3e})",
            f"apex endpoint is at ({apex.X:.6f}, {apex.Y:.6f}), not the origin",
        )

        stage("Angular dimension placement points (model mm)")
        for label, a, b in (
            ("face cone", outline[1], outline[2]),
            ("back cone", outline[2], outline[3]),
        ):
            x, _, z = _angle_position(a, b)
            print(f"    {label}: R = {x * 1000.0:7.3f}, z = {z * 1000.0:7.3f}")

        stage("Relations")
        try:
            _constrain_blank(model, axis, lines, outline)
        except Exception as exc:
            fail(f"{type(exc).__name__}: {exc}")
            return 1
        status = int(value_of(sketch, "GetConstrainedStatus"))
        print(f"    status after relations: {CONSTRAINED_STATUS_NAMES.get(status, status)}")
        check(
            status != SW_FULLY_CONSTRAINED,
            "still under defined, as it should be before dimensioning",
            "already fully defined before any dimension - relations over-constrain",
        )

        stage("Dimensions")
        try:
            dims = _dimension_blank(session.app, model, geo, MEMBER, axis, lines, outline)
        except Exception as exc:
            fail(f"{type(exc).__name__}: {exc}")
            traceback.print_exc()
            print(
                "\n    A value mismatch here means the entities or the angular\n"
                "    placement are wrong. A driving-state failure means the\n"
                "    degree-of-freedom count is wrong."
            )
            return 1
        ok("all dimensions created")

        stage("Every dimension is driving, and reads what we asked for")
        dim = model.Parameter("FaceConeAngle@Sketch1")
        if dim is None:
            fail("could not look the face cone angle back up by name")
        else:
            actual = math.degrees(float(dim.SystemValue))
            check(
                abs(actual - m.face_angle_deg) < 1e-6,
                f"face cone angle reads {actual:.6f} deg "
                f"(geometry: {m.face_angle_deg:.6f})",
                f"face cone angle reads {actual:.6f} deg, expected "
                f"{m.face_angle_deg:.6f} - wrong quadrant or wrong entities",
            )
            check(
                int(dim.DrivenState) == SW_DIM_DRIVING,
                "face cone angle is a driving dimension",
                f"face cone angle is driven (DrivenState {dim.DrivenState})",
            )

        stage("Sketch is fully defined")
        status = int(value_of(sketch, "GetConstrainedStatus"))
        check(
            status == SW_FULLY_CONSTRAINED,
            "fully defined",
            f"sketch is {CONSTRAINED_STATUS_NAMES.get(status, status)}",
        )

        model.SketchManager.InsertSketch(True)
        model.ClearSelection2(True)
        blank_sketch = model.FeatureByPositionReverse(0)

        stage("Global variables and the equations that drive the dimensions")
        try:
            _link_blank_equations(model, blank_sketch.Name, dims)
        except Exception as exc:
            fail(f"{type(exc).__name__}: {exc}")
            traceback.print_exc()
            return 1
        mgr = equation_manager(model)
        count = int(value_of(mgr, "GetCount"))
        check(
            count == 2 * len(dims),
            f"{count} equations for {len(dims)} dimensions",
            f"{count} equations, expected {2 * len(dims)}",
        )
        for i in range(count):
            print(f"    [{i}] {mgr.Equation(i)}")

        stage("Editing a variable moves the geometry")
        target = f'"CrownRadius" = {CROWN_TEST_MM}mm'
        for i in range(count):
            if mgr.Equation(i).startswith('"CrownRadius" ='):
                # SetEquation is a parameterized property put (dispid 8), which
                # late binding will not surface under its own name.
                mgr._oleobj_.Invoke(
                    8, 0, pythoncom.DISPATCH_PROPERTYPUT, 0, i, target
                )
        mgr.EvaluateAll()
        flag_methods(model, "EditRebuild3").EditRebuild3()
        moved = model.Parameter(f"CrownRadius@{blank_sketch.Name}")
        check(
            abs(float(moved.SystemValue) * 1000.0 - CROWN_TEST_MM) < 1e-6,
            f"crown radius followed the variable to {CROWN_TEST_MM} mm",
            f"crown radius is {float(moved.SystemValue) * 1000.0:.4f} mm, "
            f"expected {CROWN_TEST_MM}",
        )

        print("\n" + "=" * 62)
        if failures:
            print(f"{failures} STAGE(S) FAILED - see above.")
        else:
            print("ALL STAGES PASSED - the dimensioning approach holds up.")
        print("=" * 62)
        print("\nThe part is left open and unsaved; edit Sketch1 and look at it.")
        return 1 if failures else 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception:
        print("\n--- unhandled exception ---")
        traceback.print_exc()
        sys.exit(1)
