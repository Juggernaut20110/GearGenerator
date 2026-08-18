"""Build the full bevel set: both parts, then an assembly with them meshing.

Placing the pair and constraining it are two separate jobs here, done by two
different mechanisms on purpose.

**Placement** is written straight into the component transforms. A bevel pair
needs coincident apexes, axes at the shaft angle *and* clocked teeth, and the
mate solver can satisfy the first two while getting the third wrong. The
placement arithmetic is exact (see `bevelgear.mesh`), so writing it in leaves
the solver nothing to get wrong.

**Constraint** is then layered on top, so the saved assembly articulates rather
than being two frozen bodies. Each member has exactly five degrees of freedom
removed and keeps the sixth - the spin about its own axis:

    apex        component origin coincident with the assembly origin      (3)
    axis        component axis coincident with the assembly Top plane     (1)
    direction   pinion: axis also coincident with the Right plane         (1)
                gear:   angle mate to the pinion axis = the shaft angle   (1)

A **gear mate** between the two axes then ties those two remaining freedoms
together in the ratio z1:z2, so turning either member turns the other. The
assembly ends up under defined by one degree of freedom, which is what an
articulating gear set *should* report - fully defined would mean the teeth
could not turn.

Two things make the two mechanisms cooperate rather than fight:

* Every mate is added with `swMateAlignCLOSEST`. The components are already
  sitting exactly where they belong, so "closest" resolves each mate's
  alignment against the arrangement in front of it and nothing moves. Passing
  ALIGNED or ANTI_ALIGNED is a coin toss that flips a part half the time.
* No mate touches the spin, so the clocking written by the transform survives
  untouched. The gear mate couples the two spins wherever it finds them; it
  does not choose a phase.

The *sense* of that coupling had to be measured rather than derived.
`mesh.angular_velocity_ratio` proves the two members must turn in opposite
senses about their outward axes at any shaft angle, but how SOLIDWORKS reads a
gear mate's Reverse flag against two reference axes is not something the API
will report. Measured on the anchor set by dragging the pinion by hand: with
the pinion axis selected first and Reverse *off*, the pair turns the right way.
`reverse_gear_mate=True` is kept for the day that stops being true.

Measuring it took a hand because **the coupling only acts on a drag**. Writing
a component transform and rebuilding, adding a driving angle mate, and
`IDragOperator` were each measured to turn the pinion exactly as asked and
leave the gear sitting at 0.0000 degrees; none of the three runs the gear mate.
See `tools/probe_gear_sense.py`. That makes the sense awkward to test, but it
is also what lets this module get away with placing the pair by transform: the
gear mate never gets the chance to shift the phase that was written in.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path

from .. import mesh
from ..geometry import SetGeometry
from .part import AXIS_FEATURE_NAME, BuildResult, build_gear
from .session import (
    ADD_MATE_ERROR_NAMES,
    CONSTRAINED_STATUS_NAMES,
    FEATURE_TYPE_ORIGIN,
    FEATURE_TYPE_REF_AXIS,
    MARK_MATE_ENTITY,
    NULL_DISPATCH,
    RIGHT_PLANE_NAMES,
    SW_ADD_COMPONENT_CURRENT_CONFIG,
    SW_ADD_MATE_NO_ERROR,
    SW_MATE_ALIGN_CLOSEST,
    SW_MATE_ANGLE,
    SW_MATE_COINCIDENT,
    SW_MATE_GEAR,
    TOP_PLANE_NAMES,
    SwError,
    doubles,
    flag_methods,
    int_byref,
    require,
    select_first,
    value_of,
)

# What each member should report once the mates are in: one spin left over.
EXPECTED_COMPONENT_STATUS = "under defined"


@dataclass
class SetResult:
    """Both parts plus the assembly, and the checks made on the result."""

    pinion: BuildResult
    gear: BuildResult
    assembly_title: str
    assembly_path: str | None
    shaft_angle_deg: float
    clocking_deg: float
    measured_shaft_angle_deg: float
    interference_count: int = -1
    interference_volume_mm3: float = 0.0
    mates: tuple[str, ...] = ()
    gear_ratio: tuple[float, float] | None = None
    gear_mate_reversed: bool = False
    pinion_status: str = "not checked"
    gear_status: str = "not checked"
    model: object = field(default=None, repr=False)

    @property
    def shaft_angle_error_deg(self) -> float:
        return self.measured_shaft_angle_deg - self.shaft_angle_deg

    @property
    def articulates(self) -> bool:
        """Both members still free to spin, and a gear mate tying them together.

        A member that came back fully defined has been pinned by a mate that
        removed six degrees of freedom instead of five, and the set will not
        turn however good the gear mate is.
        """
        return bool(self.gear_ratio) and (
            self.pinion_status == self.gear_status == EXPECTED_COMPONENT_STATUS
        )


def part_filename(geo: SetGeometry, member: str) -> str:
    m = geo.member(member)
    module = f"{geo.params.module:g}".replace(".", "p")
    return f"{member}_m{module}_z{m.z}.sldprt"


def _add_component(model, path: str):
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


def _place(session, comp, matrix, what: str) -> None:
    """Write a component's placement transform directly."""
    xform = require(
        session.math_utility().CreateTransform(doubles(mesh.to_array_data(matrix))),
        f"build the transform for the {what}",
    )
    comp.Transform2 = xform


def _measure_axis(comp) -> tuple[float, float, float]:
    """Read back where a component's own +Z axis actually points."""
    data = list(comp.Transform2.ArrayData)
    # Column-major: entries 6..8 are the image of the Z axis.
    return data[6], data[7], data[8]


# --- finding the entities to mate against ----------------------------------


def _walk_features(first):
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


def _feature_of_type(owner, type_name: str, what: str):
    """The first feature of a given type, in a component or in a document.

    By type rather than by name because type names survive localisation and
    feature names do not: the origin is "Ursprung" on a German seat and
    "Origine" on a French one, so a name lookup works until the day it does
    not. `GetTypeName2` returns an API constant, the same on every seat.
    """
    first = flag_methods(owner, "FirstFeature").FirstFeature()
    for feat in _walk_features(first):
        if str(value_of(feat, "GetTypeName2")) == type_name:
            return feat
    raise SwError(f"could not find the {what}")


def _origin_point(owner, what: str):
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
    feat = _feature_of_type(owner, FEATURE_TYPE_ORIGIN, what)
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


def _component_axis(comp, what: str):
    """The part's reference axis, by name where possible.

    `build_gear` names it, so the name is the direct route. The type walk is
    the fallback for a part built before the rename, and it is unambiguous
    because these parts contain exactly one reference axis.
    """
    try:
        feat = comp.FeatureByName(AXIS_FEATURE_NAME)
    except Exception:
        feat = None
    if feat is not None:
        return feat
    return _feature_of_type(comp, FEATURE_TYPE_REF_AXIS, f"{what}'s axis")


def _feature_pick(feat, what: str):
    """A selector for a feature - a component's reference axis."""
    def pick(model, append: bool) -> None:
        if not feat.Select2(bool(append), MARK_MATE_ENTITY):
            raise SwError(f"could not select {what} to mate against")
    return pick


def _point_pick(point, what: str):
    """A selector for a sketch point. `Select4` takes a callout, not a mark.

    Marks turn out not to matter to `AddMate5` - it mates whatever is in the
    selection list - but the callout is an IDispatch parameter, so it needs the
    null VARIANT rather than a bare None.
    """
    def pick(model, append: bool) -> None:
        if not point.Select4(bool(append), NULL_DISPATCH):
            raise SwError(f"could not select {what} to mate against")
    return pick


def _plane_pick(names, what: str):
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


def _add_mate(
    model,
    picks,
    mate_type: int,
    what: str,
    *,
    angle: float = 0.0,
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
    """
    model.ClearSelection2(True)
    for i, pick in enumerate(picks):
        pick(model, i > 0)

    errors = int_byref()
    mate = model.AddMate5(
        int(mate_type),
        SW_MATE_ALIGN_CLOSEST,
        bool(flip),
        0.0, 0.0, 0.0,            # distance, and its limits
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


def _unfix(model, comp, what: str) -> None:
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


def _component_status(comp) -> str:
    """How constrained a component ended up, in words."""
    try:
        return CONSTRAINED_STATUS_NAMES.get(
            int(value_of(comp, "GetConstrainedStatus")), "unknown"
        )
    except Exception:
        return "not checked"


def add_mates(model, pinion_comp, gear_comp, geo: SetGeometry, reverse: bool = False):
    """Constrain the pair so it articulates, and couple it with a gear mate.

    Returns the names of the mates added, in the order they were added. See the
    module docstring for the degree-of-freedom bookkeeping - the short version
    is that each member keeps its spin and the gear mate joins the two.
    """
    _unfix(model, pinion_comp, "pinion")
    _unfix(model, gear_comp, "gear")

    origin = _point_pick(
        _origin_point(model, "the assembly origin"), "the assembly origin"
    )
    top = _plane_pick(TOP_PLANE_NAMES, "the assembly Top plane")
    right = _plane_pick(RIGHT_PLANE_NAMES, "the assembly Right plane")

    pick_pinion_axis = _feature_pick(
        _component_axis(pinion_comp, "pinion"), "the pinion axis"
    )
    pick_gear_axis = _feature_pick(
        _component_axis(gear_comp, "gear"), "the gear axis"
    )
    pick_pinion_apex = _point_pick(
        _origin_point(pinion_comp, "the pinion origin"), "the pinion apex"
    )
    pick_gear_apex = _point_pick(
        _origin_point(gear_comp, "the gear origin"), "the gear apex"
    )

    added: list[str] = []

    def mate(picks, mate_type: int, what: str, **kwargs) -> None:
        _add_mate(model, picks, mate_type, what, **kwargs)
        added.append(what)

    # Apex on the origin first: it takes all three translations away, so every
    # mate after it has only rotations left to remove and cannot over-define
    # the assembly by taking the same translation twice.
    mate((pick_pinion_apex, origin), SW_MATE_COINCIDENT,
         "pinion apex - assembly origin")
    mate((pick_pinion_axis, top), SW_MATE_COINCIDENT,
         "pinion axis - Top plane")
    mate((pick_pinion_axis, right), SW_MATE_COINCIDENT,
         "pinion axis - Right plane")
    mate((pick_gear_apex, origin), SW_MATE_COINCIDENT,
         "gear apex - assembly origin")
    mate((pick_gear_axis, top), SW_MATE_COINCIDENT,
         "gear axis - Top plane")
    # The shaft angle goes in as an angle mate rather than as a second plane,
    # because the gear axis is the one direction in this assembly that no
    # default plane is parallel to. It is also the number someone would want to
    # edit afterwards, and an angle mate is where they would look for it.
    mate((pick_gear_axis, pick_pinion_axis), SW_MATE_ANGLE,
         f"shaft angle {geo.params.shaft_angle:g} deg",
         angle=float(geo.params.sigma))

    ratio = mesh.gear_mate_ratio(geo.pinion.z, geo.gear.z)
    mate((pick_pinion_axis, pick_gear_axis), SW_MATE_GEAR,
         f"gear mate {geo.pinion.z}:{geo.gear.z}"
         + (" reversed" if reverse else ""),
         ratio=ratio, flip=bool(reverse))

    return tuple(added), ratio


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


def build_set(
    session,
    geo: SetGeometry,
    out_dir: str | Path,
    save_assembly: bool = True,
    mate: bool = True,
    reverse_gear_mate: bool = False,
) -> SetResult:
    """Build both members and an assembly with their teeth meshing.

    With `mate` on - the default - the pair is also constrained and coupled by
    a gear mate, so the saved assembly turns. Off, it is placed and left
    floating, which is the older behaviour and is worth having when a mate is
    the thing under suspicion.
    """
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    pinion_path = out_dir / part_filename(geo, "pinion")
    gear_path = out_dir / part_filename(geo, "gear")

    pinion = build_gear(session, geo, "pinion", str(pinion_path))
    gear = build_gear(session, geo, "gear", str(gear_path))

    model = session.new_assembly()
    flag_methods(model, "EditRebuild3", "ViewZoomtofit2")

    pinion_comp = _add_component(model, pinion_path)
    gear_comp = _add_component(model, gear_path)

    clocking = mesh.gear_clocking(geo.gear.z)
    pinion_matrix = mesh.pinion_placement()
    gear_matrix = mesh.gear_placement(geo.params.sigma, clocking)

    _place(session, pinion_comp, pinion_matrix, "pinion")
    _place(session, gear_comp, gear_matrix, "gear")

    mates: tuple[str, ...] = ()
    ratio = None
    if mate:
        mates, ratio = add_mates(
            model, pinion_comp, gear_comp, geo, reverse=reverse_gear_mate
        )

    model.EditRebuild3()
    try:
        model.ViewZoomtofit2()
    except Exception:
        pass

    # Measured after the rebuild, so it reports where the *mates* left the
    # components rather than where the transforms put them. A mate that solved
    # to the supplement of the shaft angle would show up here as 180 - sigma.
    measured = math.degrees(
        mesh.angle_between(_measure_axis(pinion_comp), _measure_axis(gear_comp))
    )
    interference_count, interference_volume = check_interference(model)

    path = None
    if save_assembly:
        module = f"{geo.params.module:g}".replace(".", "p")
        path = str(
            out_dir / f"set_m{module}_z{geo.pinion.z}x{geo.gear.z}.sldasm"
        )
        session.save(model, path)

    return SetResult(
        pinion=pinion,
        gear=gear,
        assembly_title=model.GetTitle,
        assembly_path=path,
        shaft_angle_deg=geo.params.shaft_angle,
        clocking_deg=math.degrees(clocking),
        measured_shaft_angle_deg=measured,
        interference_count=interference_count,
        interference_volume_mm3=interference_volume,
        mates=mates,
        gear_ratio=ratio,
        gear_mate_reversed=bool(reverse_gear_mate),
        pinion_status=_component_status(pinion_comp) if mate else "not mated",
        gear_status=_component_status(gear_comp) if mate else "not mated",
        model=model,
    )
