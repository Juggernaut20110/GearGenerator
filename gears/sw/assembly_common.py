"""Assembly steps that do not care what kind of gear pair is being assembled.

Both pairs are built the same way, and the reason is worth stating once here
rather than twice below.

**Placement** is written straight into the component transforms. A pair needs
its members positioned, aimed *and* clocked, and the mate solver can satisfy the
first two while getting the third wrong. The placement arithmetic is exact - see
each type's `mesh` module - so writing it in leaves the solver nothing to get
wrong.

**Constraint** is then layered on top, so the saved assembly articulates rather
than being two frozen bodies. Each member has exactly five degrees of freedom
removed and keeps the sixth, the spin about its own axis, and a **gear mate**
ties those two remaining freedoms together in the ratio z1:z2. The assembly ends
up under defined by one degree of freedom, which is what an articulating gear set
*should* report - fully defined would mean the teeth could not turn.

Two things make the two mechanisms cooperate rather than fight:

* Every mate is added with `swMateAlignCLOSEST`. The components are already
  sitting exactly where they belong, so "closest" resolves each mate's
  alignment against the arrangement in front of it and nothing moves. Passing
  ALIGNED or ANTI_ALIGNED is a coin toss that flips a part half the time.
* No mate touches the spin, so the clocking written by the transform survives
  untouched. The gear mate couples the two spins wherever it finds them; it
  does not choose a phase.

That last point is also why placing by transform is safe: **the gear mate only
acts on an interactive drag**. Writing a component transform and rebuilding,
adding a driving angle mate, and `IDragOperator` were each measured to turn the
pinion exactly as asked and leave the mate sitting at 0.0000 degrees - see
`tools/probe_gear_sense.py`. The gear mate never gets the chance to shift the
phase that was written in.

Which way the coupling turns has to be **measured per gear type, not derived and
not inherited**. `placement.angular_velocity_ratio` proves the two members must
turn in opposite senses about their outward axes, but how SOLIDWORKS reads a
gear mate's Reverse flag against two reference axes is not something the API
will report.
"""

from __future__ import annotations


from .common import AXIS_FEATURE_NAME
from .session import (
    ADD_MATE_ERROR_NAMES,
    CONSTRAINED_STATUS_NAMES,
    FEATURE_TYPE_ORIGIN,
    FEATURE_TYPE_REF_AXIS,
    MARK_MATE_ENTITY,
    NULL_DISPATCH,
    SW_ADD_COMPONENT_CURRENT_CONFIG,
    SW_ADD_MATE_NO_ERROR,
    SW_MATE_ALIGN_CLOSEST,
    SwError,
    doubles,
    flag_methods,
    int_byref,
    mm,
    require,
    select_first,
    value_of,
)

# What each member should report once the mates are in: one spin left over.
EXPECTED_COMPONENT_STATUS = "under defined"


def part_filename(geo, member: str) -> str:
    """`pinion_m2_z17.sldprt`. The module's decimal point becomes a p."""
    m = geo.member(member)
    module = f"{geo.params.module:g}".replace(".", "p")
    return f"{member}_m{module}_z{m.z}.sldprt"


def add_component(model, path: str):
    comp = model.AddComponent5(
        str(path),
        SW_ADD_COMPONENT_CURRENT_CONFIG,
        "",       # new config name, unused
        False,    # use config for part references
        "",       # existing config name
        0.0, 0.0, 0.0,
    )
    if comp is None:
        raise SwError(f"could not insert {path} into the assembly")
    return comp


def place(session, comp, matrix, what: str, translation=(0.0, 0.0, 0.0)) -> None:
    """Write a component's placement transform directly.

    `translation` is in millimetres, like every other length upstream of this
    package - `mm()` converts it here, at the boundary, and nowhere else. A
    bevel pair shares an apex and leaves it at zero; a spur pair needs the
    centre distance.
    """
    from ..placement import to_array_data

    data = to_array_data(matrix, tuple(mm(v) for v in translation))
    xform = require(
        session.math_utility().CreateTransform(doubles(data)),
        f"build the transform for the {what}",
    )
    comp.Transform2 = xform


def measure_axis(comp) -> tuple[float, float, float]:
    """Read back where a component's own +Z axis actually points."""
    data = list(comp.Transform2.ArrayData)
    # Column-major: entries 6..8 are the image of the Z axis.
    return data[6], data[7], data[8]


def measure_origin(comp) -> tuple[float, float, float]:
    """Read back where a component's own origin actually sits, in mm.

    Entries 9..11 are the translation column, in metres. What the bevel pair
    never needs and the spur pair is checked by.
    """
    data = list(comp.Transform2.ArrayData)
    return data[9] * 1000.0, data[10] * 1000.0, data[11] * 1000.0


# --- finding the entities to mate against ----------------------------------


def walk_features(first):
    """Yield a feature and everything after it in the tree.

    `GetNextFeature` returns IDispatch and takes no arguments, so late binding
    can resolve it as a property - and it decides per object, on first access.
    Every feature that comes out of the walk therefore has to be flagged before
    its own first access, not just the one we started from.
    """
    feat = first
    while feat is not None:
        yield feat
        feat = flag_methods(feat, "GetNextFeature").GetNextFeature()


def feature_of_type(owner, type_name: str, what: str):
    """The first feature of a given type, in a component or in a document.

    By type rather than by name because type names survive localisation and
    feature names do not: the origin is "Ursprung" on a German seat and
    "Origine" on a French one, so a name lookup works until the day it does
    not. `GetTypeName2` returns an API constant, the same on every seat.
    """
    first = flag_methods(owner, "FirstFeature").FirstFeature()
    for feat in walk_features(first):
        if str(value_of(feat, "GetTypeName2")) == type_name:
            return feat
    raise SwError(f"could not find the {what}")


def origin_point(owner, what: str):
    """The point inside an origin - the only part of it that can be mated.

    Selecting the origin *feature* is what a user does in the tree, and it is
    the obvious thing to do over the API. It does not work. Measured with
    `tools/probe_mate.py`: `Feature.Select2` on an OriginProfileFeature lands
    in the selection list as swSelSKETCHES, and `AddMate5` refuses it by
    returning None with an error status of 0 - "unknown error", which is as
    unhelpful as it sounds. Reaching through the feature to its single sketch
    point instead reports swSelEXTSKETCHPOINTS and mates first time.

    `SelectByID2("Point1@Origin@part-1@assembly", "EXTSKETCHPOINT")` works too,
    and is what most examples use, but it needs the origin's localised name and
    a hand-built component path. This route needs neither.
    """
    feat = feature_of_type(owner, FEATURE_TYPE_ORIGIN, what)
    sketch = require(
        flag_methods(feat, "GetSpecificFeature2").GetSpecificFeature2(),
        f"read the sketch behind {what}",
    )
    points = list(
        flag_methods(sketch, "GetSketchPoints2").GetSketchPoints2() or []
    )
    if len(points) != 1:
        raise SwError(f"expected {what} to hold one point, found {len(points)}")
    return points[0]


def component_axis(comp, what: str):
    """The part's reference axis, by name where possible.

    Both part builders name it, so the name is the direct route. The type walk is
    the fallback for a part built before the rename, and it is unambiguous
    because these parts contain exactly one reference axis.
    """
    try:
        feat = comp.FeatureByName(AXIS_FEATURE_NAME)
    except Exception:
        feat = None
    if feat is not None:
        return feat
    return feature_of_type(comp, FEATURE_TYPE_REF_AXIS, f"{what}'s axis")


def feature_pick(feat, what: str):
    """A selector for a feature - a component's reference axis."""
    def pick(model, append: bool) -> None:
        if not feat.Select2(bool(append), MARK_MATE_ENTITY):
            raise SwError(f"could not select {what} to mate against")
    return pick


def point_pick(point, what: str):
    """A selector for a sketch point. `Select4` takes a callout, not a mark.

    Marks turn out not to matter to `AddMate5` - it mates whatever is in the
    selection list - but the callout is an IDispatch parameter, so it needs the
    null VARIANT rather than a bare None.
    """
    def pick(model, append: bool) -> None:
        if not point.Select4(bool(append), NULL_DISPATCH):
            raise SwError(f"could not select {what} to mate against")
    return pick


def plane_pick(names, what: str):
    """A selector for one of the assembly's own default planes.

    Unqualified names resolve against the assembly itself; a component's plane
    would need the "@component@assembly" suffix, which is exactly the kind of
    string this codebase avoids building.
    """
    def pick(model, append: bool) -> None:
        try:
            select_first(model, names, "PLANE", append=append, mark=MARK_MATE_ENTITY)
        except SwError as exc:
            raise SwError(f"could not select {what} to mate against: {exc}") from exc
    return pick


# --- mates -----------------------------------------------------------------


def add_mate(
    model,
    picks,
    mate_type: int,
    what: str,
    *,
    angle: float = 0.0,
    distance: float = 0.0,
    ratio: tuple[float, float] = (1.0, 1.0),
    flip: bool = False,
):
    """Select two entities and add one mate, checking the byref error status.

    `AddMate5` is the honest one of the family to call: it reports its refusal
    in the byref status rather than in the return value, which can come back as
    a mate object that is not actually in the assembly. Both are checked.

    Every numeric argument is passed as a float. The signature is all VT_R8
    apart from the type, alignment and width option, and an `int` where a
    `double` is expected raises "Type mismatch" under late binding.

    `distance` is in millimetres and converted here, like every other length
    crossing into the API.
    """
    model.ClearSelection2(True)
    for i, pick in enumerate(picks):
        pick(model, i > 0)

    errors = int_byref()
    mate = model.AddMate5(
        int(mate_type),
        SW_MATE_ALIGN_CLOSEST,
        bool(flip),
        mm(distance), 0.0, 0.0,   # distance, and its limits
        float(ratio[0]), float(ratio[1]),
        float(angle), 0.0, 0.0,   # angle, and its limits
        False,                    # for positioning only - we want it kept
        False,                    # lock rotation (a coincident-mate option)
        0,                        # width mate option, unused
        errors,
    )
    model.ClearSelection2(True)

    status = int(errors.value)
    if mate is None or status != SW_ADD_MATE_NO_ERROR:
        reason = ADD_MATE_ERROR_NAMES.get(status, f"status {status}")
        raise SwError(f"SOLIDWORKS would not add the {what} mate: {reason}")
    return mate


def unfix(model, comp, what: str) -> None:
    """Float a component so the mates can hold it instead.

    A component added through `AddComponent5` arrives fixed. Fixed outranks
    every mate on it, so the gear mate would be added, look right in the tree,
    and do nothing at all when the assembly is dragged.
    """
    model.ClearSelection2(True)
    if not comp.Select2(False, 0):
        raise SwError(f"could not select the {what} to float it")
    # Takes no arguments and returns nothing, which is the shape late binding
    # is free to read as a property. Flagged here rather than at the caller so
    # add_mates works on any assembly handed to it.
    flag_methods(model, "UnfixComponent").UnfixComponent()
    model.ClearSelection2(True)
    # IsFixed takes no arguments and returns a plain bool, so late binding can
    # hand back the value itself rather than something to call. See value_of.
    if value_of(comp, "IsFixed"):
        raise SwError(f"the {what} is still fixed, so no mate on it can act")


def component_status(comp) -> str:
    """How constrained a component ended up, in words."""
    try:
        return CONSTRAINED_STATUS_NAMES.get(
            int(value_of(comp, "GetConstrainedStatus")), "unknown"
        )
    except Exception:
        return "not checked"


def check_interference(model) -> tuple[int, float]:
    """Run SOLIDWORKS interference detection. Returns (count, total volume mm3).

    `TreatCoincidenceAsInterference` must be off: with zero backlash the tooth
    flanks touch exactly at the pitch point, and coincident faces are the
    correct answer there, not a fault.

    Some genuine interference is expected and worth reporting rather than
    hiding. Tredgold's back-cone construction is an approximation, so the
    flanks are not perfectly conjugate; the volume says how much that costs.
    Returns (-1, 0.0) if detection is unavailable.
    """
    try:
        idm = model.InterferenceDetectionManager
        if idm is None:
            return -1, 0.0
        idm.TreatCoincidenceAsInterference = False
        idm.TreatSubAssembliesAsComponents = False
        count = flag_methods(
            idm, "GetInterferenceCount", "GetInterferences", "Done"
        ).GetInterferenceCount()
        volume = 0.0
        if count:
            for itf in list(idm.GetInterferences() or []):
                try:
                    volume += float(itf.Volume) * 1e9  # m3 -> mm3
                except Exception:
                    pass
        try:
            idm.Done()
        except Exception:
            pass
        return int(count), volume
    except Exception:
        return -1, 0.0


