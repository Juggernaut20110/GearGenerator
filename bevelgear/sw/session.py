"""COM connection, unit conversion, and the checked-call discipline.

Three things learned the hard way in the M0 smoke test drive the design here:

* **Late binding only.** `EnsureDispatch` cannot introspect `ISldWorks`, and
  even after running `makepy` over `sldworks.tlb` the coclass is missing from
  the generated CLSID map, so `Dispatch` hands back a plain `CDispatch`. That
  means no argument coercion is done for us.
* **Everything must be the right VARIANT type.** Point arrays need an explicit
  `VT_ARRAY | VT_R8`, null object parameters need `VT_DISPATCH` holding None,
  and numeric arguments must be floats - an `int` where a `double` is expected
  raises "Type mismatch".
* **Failures are silent.** The API returns `None` or `False` instead of
  raising, so every call goes through `require()` and turns into a real Python
  exception naming the step that failed.

The SOLIDWORKS API is metres and radians throughout; `mm()` and the geometry
layer's radians are the only conversion points.
"""

from __future__ import annotations

import pythoncom
import win32com.client
from win32com.client import VARIANT

# --- enum values we actually use (from swconst.tlb) ------------------------

SW_DEFAULT_TEMPLATE_PART = 8      # swUserPreferenceStringValue_e
SW_DEFAULT_TEMPLATE_ASSEMBLY = 9
SW_ADD_COMPONENT_CURRENT_CONFIG = 0   # swAddComponentConfigOptions_e
SW_SOLID_BODY = 0                 # swBodyType_e
SW_END_COND_BLIND = 0             # swEndConditions_e
SW_THIN_ONE_DIRECTION = 0         # swThinWallType_e
SW_SAVE_AS_CURRENT_VERSION = 0    # swSaveAsVersion_e
SW_SAVE_AS_OPTIONS_SILENT = 1     # swSaveAsOptions_e

# Marks used by the feature calls. These are not arbitrary: the API looks for
# specific mark values in the selection list.
MARK_LOFT_PROFILE = 1
MARK_PATTERN_AXIS = 1
MARK_PATTERN_FEATURE = 4

# Late binding cannot marshal a bare None into a VARIANT of type IDispatch*.
NULL_DISPATCH = VARIANT(pythoncom.VT_DISPATCH, None)

# Plane names vary with the template's locale.
FRONT_PLANE_NAMES = ("Front Plane", "Front", "Plane1")
TOP_PLANE_NAMES = ("Top Plane", "Top", "Plane2")
RIGHT_PLANE_NAMES = ("Right Plane", "Right", "Plane3")


class SwError(RuntimeError):
    """A SOLIDWORKS API call failed. The message names the step."""


def mm(value: float) -> float:
    """Millimetres to metres, the unit the API actually wants."""
    return float(value) * 0.001


def doubles(values) -> VARIANT:
    """Pack a flat sequence of numbers into the VT_ARRAY|VT_R8 the API needs.

    Passing a plain Python list here is the single most common cause of a
    SOLIDWORKS call silently returning None.
    """
    return VARIANT(pythoncom.VT_ARRAY | pythoncom.VT_R8, [float(v) for v in values])


def points_to_doubles(points) -> VARIANT:
    """Flatten (x, y, z) triples in mm into the metre-based array the API wants."""
    flat = []
    for x, y, z in points:
        flat.extend((mm(x), mm(y), mm(z)))
    return doubles(flat)


def flag_methods(obj, *names):
    """Tell win32com's dynamic dispatch that these names are methods.

    A COM member whose only argument is a VARIANT is ambiguous to late
    binding, and win32com can resolve it as a property instead - in which case
    attribute access silently returns None rather than a callable.
    `SketchManager.CreateSpline(PointData)` is exactly that shape and hits it
    every time.
    """
    try:
        obj._FlagAsMethod(*names)
    except Exception:
        pass
    return obj


def require(value, what: str):
    """Raise unless the API call actually did something.

    `False` and `None` both mean failure in this API. Zero is a legitimate
    return in some places, so it is deliberately not treated as failure.
    """
    if value is None or value is False:
        raise SwError(f"SOLIDWORKS rejected: {what}")
    return value


def connect(visible: bool = True):
    """Attach to a running SOLIDWORKS, or start one."""
    app = win32com.client.Dispatch("SldWorks.Application")
    if app is None:
        raise SwError("could not create SldWorks.Application")
    app.Visible = visible
    return app


class SwSession:
    """Owns the COM lifetime and the handful of global flags we toggle.

    Use as a context manager so `CoUninitialize` and the sketch flags are
    always restored, even if a build raises partway through.
    """

    def __init__(self, visible: bool = True, silent: bool = True):
        self.visible = visible
        self.silent = silent
        self.app = None
        self._co_initialised = False

    def __enter__(self) -> "SwSession":
        pythoncom.CoInitialize()
        self._co_initialised = True
        self.app = connect(self.visible)
        if self.silent:
            # Suppress modal dialogs; a build driven over COM must never block
            # waiting for a click.
            self.app.CommandInProgress = True
        return self

    def __exit__(self, *exc):
        if self.app is not None and self.silent:
            try:
                self.app.CommandInProgress = False
            except Exception:
                pass
        if self._co_initialised:
            pythoncom.CoUninitialize()
            self._co_initialised = False
        return False

    # --- documents ---------------------------------------------------------

    def _new_document(self, preference: int, what: str):
        template = self.app.GetUserPreferenceStringValue(preference)
        if not template:
            raise SwError(
                f"no default {what} template is configured; set one under "
                "Tools > Options > File Locations"
            )
        return require(self.app.NewDocument(template, 0, 0, 0), f"new {what}")

    def new_part(self):
        return self._new_document(SW_DEFAULT_TEMPLATE_PART, "part")

    def new_assembly(self):
        return self._new_document(SW_DEFAULT_TEMPLATE_ASSEMBLY, "assembly")

    def math_utility(self):
        """IMathUtility, with the VARIANT-taking methods flagged."""
        util = require(
            flag_methods(self.app, "GetMathUtility").GetMathUtility(),
            "GetMathUtility",
        )
        return flag_methods(util, "CreateTransform", "CreateVector", "CreatePoint")

    def save(self, model, path: str) -> None:
        """Save under a new name, silently. Errors come back byref, not raised."""
        errors = VARIANT(pythoncom.VT_BYREF | pythoncom.VT_I4, 0)
        warnings = VARIANT(pythoncom.VT_BYREF | pythoncom.VT_I4, 0)
        # ExportData and AdvancedSaveAsOptions are IDispatch parameters; a bare
        # Python None will not marshal into them.
        ok = model.Extension.SaveAs3(
            str(path),
            SW_SAVE_AS_CURRENT_VERSION,
            SW_SAVE_AS_OPTIONS_SILENT,
            NULL_DISPATCH,
            NULL_DISPATCH,
            errors,
            warnings,
        )
        if not ok:
            raise SwError(
                f"SaveAs3 failed for {path} (error {errors.value}, "
                f"warning {warnings.value})"
            )

    def close(self, model, save_as: str | None = None) -> None:
        if save_as:
            self.save(model, save_as)
        # GetTitle is a property, not a method, under late binding.
        self.app.CloseDoc(model.GetTitle)


# --- selection -------------------------------------------------------------


def select(model, name: str, type_: str, append: bool = False, mark: int = 0) -> None:
    """SelectByID2 with the right VARIANT types, raising on failure."""
    ok = model.Extension.SelectByID2(
        name, type_, 0.0, 0.0, 0.0, bool(append), int(mark), NULL_DISPATCH, 0
    )
    if not ok:
        raise SwError(f"could not select {type_} named {name!r}")


def select_first(model, names, type_: str, append: bool = False, mark: int = 0) -> str:
    """Try several names for the same entity; return the one that worked."""
    for name in names:
        if model.Extension.SelectByID2(
            name, type_, 0.0, 0.0, 0.0, bool(append), int(mark), NULL_DISPATCH, 0
        ):
            return name
    raise SwError(f"none of {names} could be selected as {type_}")


class SketchFlags:
    """Turn off inference and display while writing generated geometry.

    `AddToDB` is the important one: with it off, SOLIDWORKS snaps new points to
    nearby existing geometry, which quietly destroys an involute.
    """

    def __init__(self, model):
        self.sm = model.SketchManager

    def __enter__(self):
        self.sm.AddToDB = True
        self.sm.DisplayWhenAdded = False
        return self.sm

    def __exit__(self, *exc):
        try:
            self.sm.AddToDB = False
            self.sm.DisplayWhenAdded = True
        except Exception:
            pass
        return False
