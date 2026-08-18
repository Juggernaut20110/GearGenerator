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

# Dimensions. Driving is 2 and driven is 1 - the opposite of the obvious guess,
# so these came out of swconst.tlb rather than being assumed.
SW_DIM_DRIVEN = 1                 # swDimensionDrivenState_e
SW_DIM_DRIVING = 2
SW_FULLY_CONSTRAINED = 3          # swConstrainedStatus_e

# swUserPreferenceToggle_e. The first one is the important one: left on, every
# AddDimension2 opens the Modify dialog and a COM-driven build hangs on it.
SW_PREF_INPUT_DIM_VAL_ON_CREATE = 10
SW_PREF_OVERDEF_DIMS_PROMPT = 100
SW_PREF_OVERDEF_DIMS_DRIVEN_BY_DEFAULT = 101

# Sketch relation ids, as SketchAddConstraints spells them.
REL_FIXED = "sgFIXED"
REL_HORIZONTAL = "sgHORIZONTAL"
REL_VERTICAL = "sgVERTICAL"

CONSTRAINED_STATUS_NAMES = {
    1: "unknown",
    2: "under defined",
    3: "fully defined",
    4: "over defined",
    5: "no solution",
    6: "invalid solution",
    7: "autosolve off",
}

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


def value_of(obj, name: str):
    """Read a no-argument COM member that late binding may have made a property.

    win32com decides on *first* access whether a member is a method or a
    property and caches that decision, and members that take no arguments are
    exactly the ambiguous shape. `sketch.GetConstrainedStatus` comes back as the
    status itself rather than something to call.

    This is only safe for members returning a plain value - an int, a tuple.
    A member returning IDispatch is itself callable, so the two cases become
    indistinguishable; flag those as methods before the first access instead.
    See `flag_methods`.
    """
    member = getattr(obj, name)
    return member() if callable(member) else member


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
    nearby existing geometry, which quietly destroys an involute. The cost is
    that nothing is inferred at all - not even the coincidences between the
    endpoints of a polyline drawn corner to corner - so anything that needs a
    relation has to say so explicitly afterwards. See `add_relation`.
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


class DimensionFlags:
    """Stop dimension creation from opening a dialog and blocking the build.

    `swInputDimValOnCreate` is the one that matters: left on - and it is on by
    default - every `AddDimension2` pops the Modify box and waits for a click,
    which a COM-driven build never supplies. The other two suppress the
    "this dimension over-defines the sketch, make it driven?" prompt and its
    silent-driven fallback; we would rather a dimension fail loudly than come
    back driven, because driven means the degree-of-freedom count is wrong.
    """

    _PREFS = (
        SW_PREF_INPUT_DIM_VAL_ON_CREATE,
        SW_PREF_OVERDEF_DIMS_PROMPT,
        SW_PREF_OVERDEF_DIMS_DRIVEN_BY_DEFAULT,
    )

    def __init__(self, app):
        self.app = app
        self.saved: dict[int, bool] = {}

    def __enter__(self):
        for pref in self._PREFS:
            try:
                self.saved[pref] = bool(self.app.GetUserPreferenceToggle(pref))
                self.app.SetUserPreferenceToggle(pref, False)
            except Exception:
                pass
        return self.app

    def __exit__(self, *exc):
        for pref, value in self.saved.items():
            try:
                self.app.SetUserPreferenceToggle(pref, value)
            except Exception:
                pass
        return False


# --- relations and dimensions ----------------------------------------------


def select_entities(model, entities, what: str) -> None:
    """Select sketch entities in order, replacing the current selection.

    `ISketchSegment.Select4` and `ISketchPoint.Select4` take (Append, Callout);
    the callout is an IDispatch parameter, so it needs the null VARIANT rather
    than a bare None. Order matters to the calls that consume the selection -
    an angular dimension takes its two lines in the order they were picked.
    """
    model.ClearSelection2(True)
    for i, entity in enumerate(entities):
        if not entity.Select4(i > 0, NULL_DISPATCH):
            raise SwError(f"could not select entity {i} for {what}")


def add_relation(model, entities, id_str: str, what: str) -> None:
    """Add a sketch relation to the given entities.

    `SketchAddConstraints` returns nothing at all, so there is no return value
    to check here; the sketch's constrained status at the end of the sketch is
    what actually proves the relations landed. See `require_fully_defined`.
    """
    select_entities(model, entities, what)
    model.SketchAddConstraints(id_str)
    model.ClearSelection2(True)


def add_dimension(model, entities, position, value: float, what: str, name: str = ""):
    """Add one *driving* dimension and return the IDimension.

    `position` is a model-space (x, y, z) in metres and `value` is in metres for
    a linear dimension or radians for an angular one - SI throughout, as the
    rest of the API is.

    The dimension is created against geometry that is already at `value`, so the
    value it comes back with is a check on everything else: pick the wrong
    entities, or let SOLIDWORKS choose the supplement of the angle you meant,
    and it will not match. That mismatch is raised rather than papered over by
    setting the value, because setting it would then move the geometry.
    """
    select_entities(model, entities, what)
    display = require(model.AddDimension2(*position), f"dimension {what}")
    model.ClearSelection2(True)

    # GetDimension takes no arguments and returns IDispatch, so late binding can
    # resolve it as a property; flag it before the first access. See flag_methods.
    dim = require(
        flag_methods(display, "GetDimension").GetDimension(),
        f"read back the dimension for {what}",
    )
    if dim.DrivenState != SW_DIM_DRIVING:
        dim.DrivenState = SW_DIM_DRIVING
        if dim.DrivenState != SW_DIM_DRIVING:
            raise SwError(
                f"{what} would not go driving - it over-defines the sketch, "
                "so the degree-of-freedom count is wrong"
            )

    actual = float(dim.SystemValue)
    if abs(actual - value) > 1e-6:
        raise SwError(
            f"{what} came back as {actual:.9g} but the geometry is at "
            f"{value:.9g} (SI units)"
        )
    dim.SystemValue = float(value)

    if name:
        try:
            dim.Name = name
        except Exception:
            pass
    return dim


def require_fully_defined(sketch, what: str) -> None:
    """Raise unless the sketch solved to fully defined.

    This is the check that makes the dimensioning worth anything: relations are
    added by a call with no return value and dimensions can silently fail to
    constrain what you thought, so the only honest test is to ask the solver.
    """
    status = int(value_of(sketch, "GetConstrainedStatus"))
    if status != SW_FULLY_CONSTRAINED:
        name = CONSTRAINED_STATUS_NAMES.get(status, str(status))
        raise SwError(f"{what} is {name}, not fully defined")
