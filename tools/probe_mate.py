"""Probe: what will AddMate5 actually accept as a component origin?

The first mate `sw.assembly` tries to add - the pinion's apex onto the assembly
origin - came back refused with error status 0, "unknown error", which tells us
nothing about which half of the call SOLIDWORKS objected to. This asks the
question in isolation, the way `probe_dimension.py` does for dimensions.

Two things are under test and they are independent:

* **the selection.** An origin can be reached by walking the feature tree to
  the OriginProfileFeature and selecting the feature, or by name through
  SelectByID2, or by digging the sketch point out of the feature. Only some of
  those put a *mateable point* in the selection list, and the selection manager
  will say which by reporting the selected object's type.
* **the call.** Mark value, alignment, and whether `CommandInProgress` - which
  the session sets to suppress dialogs - stops a mate being added at all.

Run it with SOLIDWORKS open and two parts already built:

    .venv\\Scripts\\python.exe tools\\probe_mate.py
    .venv\\Scripts\\python.exe tools\\probe_mate.py --part out\\pinion_m2_z17.SLDPRT
"""

from __future__ import annotations

import argparse
import sys
import traceback
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from gears.sw.session import (            # noqa: E402
    ADD_MATE_ERROR_NAMES,
    FEATURE_TYPE_ORIGIN,
    FEATURE_TYPE_REF_AXIS,
    NULL_DISPATCH,
    SW_ADD_COMPONENT_CURRENT_CONFIG,
    SW_ADD_MATE_NO_ERROR,
    SW_MATE_ALIGN_CLOSEST,
    SW_MATE_COINCIDENT,
    SwSession,
    flag_methods,
    int_byref,
    value_of,
)

# swSelectType_e, the handful that can come back from selecting an origin.
SEL_TYPE_NAMES = {
    0: "nothing",
    4: "DATUMPLANES",
    5: "DATUMAXES",
    6: "DATUMPOINTS",
    9: "SKETCHES",
    11: "SKETCHPOINTS",
    20: "COMPONENTS",
    24: "EXTSKETCHSEGS",
    25: "EXTSKETCHPOINTS",
    41: "POINTREFS",
    71: "SKETCHPOINTFEAT",
}


def describe_selection(model) -> str:
    """What SOLIDWORKS thinks is selected right now, by type."""
    try:
        mgr = model.SelectionManager
        count = int(mgr.GetSelectedObjectCount2(-1))
        kinds = []
        for i in range(1, count + 1):
            kind = int(mgr.GetSelectedObjectType3(i, -1))
            kinds.append(SEL_TYPE_NAMES.get(kind, str(kind)))
        return f"{count} selected: {', '.join(kinds) or '-'}"
    except Exception as exc:
        return f"could not read the selection: {exc}"


# --- ways of reaching an origin --------------------------------------------


def _feature_of_type(owner, type_name: str):
    feat = flag_methods(owner, "FirstFeature").FirstFeature()
    while feat is not None:
        if str(value_of(feat, "GetTypeName2")) == type_name:
            return feat
        feat = flag_methods(feat, "GetNextFeature").GetNextFeature()
    return None


def _select_by_id(model, name: str, type_: str, append: bool, mark: int) -> bool:
    return bool(
        model.Extension.SelectByID2(
            name, type_, 0.0, 0.0, 0.0, bool(append), int(mark), NULL_DISPATCH, 0
        )
    )


def _suffix(model, comp) -> str:
    """The "@component@assembly" tail SelectByID2 wants for a component entity."""
    title = str(model.GetTitle)
    for ext in (".SLDASM", ".sldasm"):
        if title.endswith(ext):
            title = title[: -len(ext)]
    return "" if comp is None else f"@{comp.Name2}@{title}"


def sel_feature(model, comp, append, mark):
    """Walk to the OriginProfileFeature and select the feature itself."""
    feat = _feature_of_type(comp if comp is not None else model, FEATURE_TYPE_ORIGIN)
    if feat is None:
        return False
    return bool(feat.Select2(bool(append), int(mark)))


def sel_named_point(model, comp, append, mark):
    """SelectByID2 on "Point1@Origin...", the name the UI shows in the tree."""
    return _select_by_id(
        model, f"Point1@Origin{_suffix(model, comp)}", "EXTSKETCHPOINT", append, mark
    )


def sel_named_origin(model, comp, append, mark):
    """SelectByID2 on the origin feature's own name."""
    return _select_by_id(
        model, f"Origin{_suffix(model, comp)}", "ORIGIN", append, mark
    )


def sel_sketch_point(model, comp, append, mark):
    """Dig the point out of the origin feature and select it as an entity."""
    feat = _feature_of_type(comp if comp is not None else model, FEATURE_TYPE_ORIGIN)
    if feat is None:
        return False
    sketch = flag_methods(feat, "GetSpecificFeature2").GetSpecificFeature2()
    if sketch is None:
        return False
    points = list(
        flag_methods(sketch, "GetSketchPoints2").GetSketchPoints2() or []
    )
    if not points:
        return False
    return bool(points[0].Select4(bool(append), NULL_DISPATCH))


ORIGIN_STRATEGIES = (
    ("feature walk, Select2", sel_feature),
    ("SelectByID2 Point1@Origin", sel_named_point),
    ("SelectByID2 Origin", sel_named_origin),
    ("origin sketch point, Select4", sel_sketch_point),
)


# --- the probe -------------------------------------------------------------


SW_DOC_PART = 1                # swDocumentTypes_e
SW_OPEN_DOC_SILENT = 1         # swOpenDocOptions_e


def open_part(app, path: Path):
    """Load the part before inserting it.

    `AddComponent5` inserts a document that is already in memory and returns
    None for one that is not - which is why a real build gets away without
    this: `build_gear` leaves both parts open.
    """
    errors, warnings = int_byref(), int_byref()
    doc = app.OpenDoc6(
        str(path), SW_DOC_PART, SW_OPEN_DOC_SILENT, "", errors, warnings
    )
    if doc is None:
        raise RuntimeError(
            f"could not open {path} (error {errors.value}, warning {warnings.value})"
        )
    return doc


def add_component(model, path: Path):
    comp = model.AddComponent5(
        str(path), SW_ADD_COMPONENT_CURRENT_CONFIG, "", False, "", 0.0, 0.0, 0.0
    )
    if comp is None:
        raise RuntimeError(f"could not insert {path}")
    return comp


def try_mate(model, mark: int, align: int):
    """AddMate5 on whatever is currently selected. Returns (mate, status)."""
    errors = int_byref()
    mate = model.AddMate5(
        SW_MATE_COINCIDENT, int(align), False,
        0.0, 0.0, 0.0,
        1.0, 1.0,
        0.0, 0.0, 0.0,
        False, False, 0,
        errors,
    )
    return mate, int(errors.value)


def report_mate(mate, status: int) -> str:
    if mate is not None and status == SW_ADD_MATE_NO_ERROR:
        return "MATE ADDED"
    reason = ADD_MATE_ERROR_NAMES.get(status, f"status {status}")
    return f"refused ({reason}, returned {'a mate' if mate else 'None'})"


def probe(session, part: Path, mark: int, align: int) -> None:
    app = session.app
    for name, strategy in ORIGIN_STRATEGIES:
        model = None
        try:
            template = app.GetUserPreferenceStringValue(9)
            model = app.NewDocument(template, 0, 0, 0)
            flag_methods(model, "EditRebuild3", "UnfixComponent", "FirstFeature")
            comp = add_component(model, part)
            model.EditRebuild3()

            # Float it, or a mate on it means nothing.
            model.ClearSelection2(True)
            comp.Select2(False, 0)
            model.UnfixComponent()
            model.ClearSelection2(True)

            ok_comp = strategy(model, comp, False, mark)
            comp_sel = describe_selection(model)
            ok_asm = strategy(model, None, True, mark)
            both_sel = describe_selection(model)

            line = f"  component: {'ok' if ok_comp else 'FAILED'} ({comp_sel})"
            print(f"{name}\n{line}")
            print(f"  assembly:  {'ok' if ok_asm else 'FAILED'} ({both_sel})")

            if ok_comp and ok_asm:
                mate, status = try_mate(model, mark, align)
                print(f"  mate:      {report_mate(mate, status)}")
            else:
                print("  mate:      not attempted, selection incomplete")
        except Exception as exc:
            print(f"{name}\n  EXCEPTION {type(exc).__name__}: {exc}")
        finally:
            if model is not None:
                try:
                    app.CloseDoc(model.GetTitle)
                except Exception:
                    pass
        print()


def probe_axis(session, part: Path) -> None:
    """Does a component reference axis mate to an assembly plane at all?

    If the origin turns out to be the only problem, this should still pass -
    which is worth knowing before redesigning the whole mate scheme.
    """
    app = session.app
    model = None
    try:
        model = app.NewDocument(app.GetUserPreferenceStringValue(9), 0, 0, 0)
        flag_methods(model, "EditRebuild3", "UnfixComponent")
        comp = add_component(model, part)
        model.EditRebuild3()
        model.ClearSelection2(True)
        comp.Select2(False, 0)
        model.UnfixComponent()
        model.ClearSelection2(True)

        axis = comp.FeatureByName("GearAxis") or _feature_of_type(
            comp, FEATURE_TYPE_REF_AXIS
        )
        if axis is None:
            print("axis to Top plane\n  no reference axis found in the part\n")
            return
        ok_axis = bool(axis.Select2(False, 1))
        print(f"axis to Top plane\n  axis:      {'ok' if ok_axis else 'FAILED'} "
              f"({describe_selection(model)})")
        ok_plane = _select_by_id(model, "Top Plane", "PLANE", True, 1)
        print(f"  plane:     {'ok' if ok_plane else 'FAILED'} "
              f"({describe_selection(model)})")
        if ok_axis and ok_plane:
            mate, status = try_mate(model, 1, SW_MATE_ALIGN_CLOSEST)
            print(f"  mate:      {report_mate(mate, status)}")
    except Exception as exc:
        print(f"axis to Top plane\n  EXCEPTION {type(exc).__name__}: {exc}")
    finally:
        if model is not None:
            try:
                app.CloseDoc(model.GetTitle)
            except Exception:
                pass
    print()


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--part", default=r"out\pinion_m2_z17.SLDPRT")
    ap.add_argument("--mark", type=int, default=1)
    ap.add_argument(
        "--align", type=int, default=SW_MATE_ALIGN_CLOSEST,
        help="swMateAlign_e: 0 aligned, 1 anti-aligned, 2 closest",
    )
    ap.add_argument(
        "--silent", action="store_true",
        help="set CommandInProgress, as a real build does",
    )
    args = ap.parse_args(argv)

    part = Path(args.part).resolve()
    if not part.exists():
        print(f"no such part: {part}")
        return 1

    print(f"part {part}")
    print(f"mark {args.mark}, align {args.align}, "
          f"CommandInProgress {'on' if args.silent else 'off'}\n")

    with SwSession(silent=args.silent) as session:
        open_part(session.app, part)
        probe(session, part, args.mark, args.align)
        probe_axis(session, part)
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception:
        traceback.print_exc()
        sys.exit(1)
