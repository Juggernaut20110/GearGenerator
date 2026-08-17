"""Milestone 0: prove we can drive SOLIDWORKS over COM from Python.

This exists to answer one question before any real work is invested: does this
Maker seat allow external automation, or is the API gated at runtime?

Run it with the venv interpreter:

    .venv\\Scripts\\python.exe tools\\smoke_com.py

Each stage prints PASS/FAIL so a partial failure tells us exactly how far the
license lets us go.
"""

import sys
import traceback

import pythoncom
import win32com.client
from win32com.client import VARIANT

# Type library identities, generated once via win32com.client.makepy against
# sldworks.tlb / swconst.tlb in the SOLIDWORKS install folder.
# Args are (typelib CLSID, lcid, major, minor).
SLDWORKS_TLB = ("{83A33D31-27C5-11CE-BFD4-00400513BB57}", 0, 34, 0)
SWCONST_TLB = ("{4687F359-55D0-4CD3-B6CF-2EB42C11F989}", 0, 34, 0)

# swUserPreferenceStringValue_e
SW_DEFAULT_TEMPLATE_PART = 8

# Late binding cannot marshal a bare Python None into a VARIANT of type
# IDispatch*, which is what SelectByID2's Callout parameter expects.
NULL_DISPATCH = VARIANT(pythoncom.VT_DISPATCH, None)

# Plane names vary by template locale; try each in turn.
FRONT_PLANE_NAMES = ("Front Plane", "Front", "Plane1")

stage_no = 0


def stage(label):
    global stage_no
    stage_no += 1
    print(f"\n[{stage_no}] {label}")


def ok(msg):
    print(f"    PASS  {msg}")


def fail(msg):
    print(f"    FAIL  {msg}")


def connect():
    """Early binding first, late binding as fallback.

    EnsureDispatch cannot introspect ISldWorks directly, so load the generated
    wrappers from the type library by GUID instead; Dispatch then hands back an
    early-bound object automatically.
    """
    try:
        win32com.client.gencache.EnsureModule(*SLDWORKS_TLB)
        win32com.client.gencache.EnsureModule(*SWCONST_TLB)
        app = win32com.client.Dispatch("SldWorks.Application")
        bound = type(app).__name__ != "CDispatch"
        ok(f"connected, early binding {'active' if bound else 'UNAVAILABLE'}")
        return app
    except Exception as exc:
        print(f"    note: early binding failed ({type(exc).__name__}: {exc})")

    app = win32com.client.Dispatch("SldWorks.Application")
    ok("connected via late binding (Dispatch)")
    return app


def select_front_plane(model):
    for name in FRONT_PLANE_NAMES:
        if model.Extension.SelectByID2(
            name, "PLANE", 0.0, 0.0, 0.0, False, 0, NULL_DISPATCH, 0
        ):
            return name
    return None


def main():
    pythoncom.CoInitialize()
    try:
        stage("Connect to SOLIDWORKS")
        sw = connect()

        stage("Query application")
        # RevisionNumber is a property, not a method, under late binding.
        print(f"    revision: {sw.RevisionNumber}")
        sw.Visible = True
        ok("visible, responding to property calls")

        stage("Resolve default part template")
        template = sw.GetUserPreferenceStringValue(SW_DEFAULT_TEMPLATE_PART)
        if not template:
            fail("no default part template configured")
            print("    Set one in Tools > Options > File Locations, then re-run.")
            return 1
        print(f"    template: {template}")
        ok("template resolved")

        stage("Create a new part document")
        model = sw.NewDocument(template, 0, 0, 0)
        if model is None:
            fail("NewDocument returned None - document creation is blocked")
            return 1
        ok("part document created")

        stage("Select the front plane")
        plane = select_front_plane(model)
        if plane is None:
            fail(f"none of {FRONT_PLANE_NAMES} could be selected")
            return 1
        ok(f"selected '{plane}'")

        stage("Sketch a circle (r = 25 mm)")
        # SOLIDWORKS API is metres and radians throughout.
        model.SketchManager.InsertSketch(True)
        circle = model.SketchManager.CreateCircleByRadius(0, 0, 0, 0.025)
        if circle is None:
            fail("CreateCircleByRadius returned None")
            return 1
        model.SketchManager.InsertSketch(True)
        ok("circle created, sketch closed")

        stage("Extrude the sketch (10 mm)")
        if not model.Extension.SelectByID2(
            "Sketch1", "SKETCH", 0.0, 0.0, 0.0, False, 0, NULL_DISPATCH, 0
        ):
            fail("could not select Sketch1")
            return 1
        feat = model.FeatureManager.FeatureExtrusion3(
            True,   # single ended
            False,  # flip side to cut
            False,  # reverse direction
            0, 0,   # end conditions (blind, blind)
            0.010, 0.01,  # depth dir1, depth dir2
            False, False,  # draft while extruding
            False, False,  # draft outward
            0.0, 0.0,      # draft angles
            False, False,  # offset reverse
            False, False,  # translate surface
            True,          # merge result
            True, True,    # use feature scope, auto select
            0, 0.0, False, # start condition, offset, flip start offset
        )
        if feat is None:
            fail("FeatureExtrusion3 returned None - feature creation is blocked")
            return 1
        ok(f"extrusion created: {feat.Name}")

        stage("Verify solid body exists")
        # GetBodies2 is an IPartDoc method, not a ModelDocExtension one.
        bodies = model.GetBodies2(0, True)  # swSolidBody, visible only
        count = len(bodies) if bodies else 0
        if count != 1:
            fail(f"expected 1 solid body, found {count}")
            return 1
        ok("1 solid body present")

        print("\n" + "=" * 62)
        print("ALL STAGES PASSED - external COM automation works on this seat.")
        print("=" * 62)
        print("\nThe test part is left open and unsaved; close it without saving.")
        return 0

    finally:
        pythoncom.CoUninitialize()


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception:
        print("\n--- unhandled exception ---")
        traceback.print_exc()
        print(
            "\nIf this is a COM error mentioning access denied or a missing class,\n"
            "the Maker license is likely gating external automation. See the M0\n"
            "fallback in the plan: generate a VBA macro and run it in-process."
        )
        sys.exit(1)
