"""Build the full bevel set: both parts, then an assembly with them meshing.

Read `assembly_common` first - it carries the rationale both gear types share:
why the pair is *placed* by writing transforms and only *constrained*
afterwards, why every mate is swMateAlignCLOSEST, and why the gear mate cannot
disturb the clocking.

What is particular to a bevel pair is the arrangement. The two members share
their pitch apex and their axes stand at the shaft angle, so each is held by:

    apex        component origin coincident with the assembly origin      (3)
    axis        component axis coincident with the assembly Top plane     (1)
    direction   pinion: axis also coincident with the Right plane         (1)
                gear:   angle mate to the pinion axis = the shaft angle   (1)

The shaft angle goes in as an angle mate rather than as a second plane
coincidence, because the gear axis is the one direction in this assembly that no
default plane is parallel to.

The *sense* of the gear mate had to be measured rather than derived. Measured on
the anchor set by dragging the pinion by hand: with the pinion axis selected
first and Reverse *off*, the pair turns the right way. `reverse_gear_mate=True`
is kept for the day that stops being true. **This answer belongs to the bevel
pair and is not inherited by any other type** - see `tools/probe_gear_sense.py`
for why it took a hand to get, and `assembly_common` for why it has to be
re-measured rather than reasoned about.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path

from ..bevel import mesh
from ..bevel.geometry import SetGeometry
from .assembly_common import (
    EXPECTED_COMPONENT_STATUS,
    add_component,
    add_mate,
    check_interference,
    component_axis,
    component_status,
    feature_pick,
    measure_axis,
    origin_point,
    part_filename,
    place,
    plane_pick,
    point_pick,
    unfix,
)
from .bevel_part import BuildResult, build_gear
from .session import (
    RIGHT_PLANE_NAMES,
    SW_MATE_ANGLE,
    SW_MATE_COINCIDENT,
    SW_MATE_GEAR,
    TOP_PLANE_NAMES,
    flag_methods,
)


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
    def parts(self) -> tuple:
        """Every part this build produced, in the order it was built.

        A pair has two. A planetary train has three distinct parts, one of
        which is inserted several times, so the report formatter reads this
        rather than assuming a pinion and a gear.
        """
        return (self.pinion, self.gear)

    @property
    def gear_ratios(self) -> tuple[tuple[float, float], ...]:
        """Every gear mate's ratio. A pair has one mesh; a train has two."""
        return (self.gear_ratio,) if self.gear_ratio else ()

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


def add_mates(model, pinion_comp, gear_comp, geo: SetGeometry, reverse: bool = False):
    """Constrain the pair so it articulates, and couple it with a gear mate.

    Returns the names of the mates added, in the order they were added. See the
    module docstring for the degree-of-freedom bookkeeping - the short version
    is that each member keeps its spin and the gear mate joins the two.
    """
    unfix(model, pinion_comp, "pinion")
    unfix(model, gear_comp, "gear")

    origin = point_pick(
        origin_point(model, "the assembly origin"), "the assembly origin"
    )
    top = plane_pick(TOP_PLANE_NAMES, "the assembly Top plane")
    right = plane_pick(RIGHT_PLANE_NAMES, "the assembly Right plane")

    pick_pinion_axis = feature_pick(
        component_axis(pinion_comp, "pinion"), "the pinion axis"
    )
    pick_gear_axis = feature_pick(
        component_axis(gear_comp, "gear"), "the gear axis"
    )
    pick_pinion_apex = point_pick(
        origin_point(pinion_comp, "the pinion origin"), "the pinion apex"
    )
    pick_gear_apex = point_pick(
        origin_point(gear_comp, "the gear origin"), "the gear apex"
    )

    added: list[str] = []

    def mate(picks, mate_type: int, what: str, **kwargs) -> None:
        add_mate(model, picks, mate_type, what, **kwargs)
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
    # Absolute: SaveAs3 refuses a relative path with a bare "error 1" that says
    # nothing about what is wrong with it. `tools/build_set.py` already resolves
    # its own argument; this is the guard for every other caller.
    out_dir = Path(out_dir).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    pinion_path = out_dir / part_filename(geo, "pinion")
    gear_path = out_dir / part_filename(geo, "gear")

    pinion = build_gear(session, geo, "pinion", str(pinion_path))
    gear = build_gear(session, geo, "gear", str(gear_path))

    model = session.new_assembly()
    flag_methods(model, "EditRebuild3", "ViewZoomtofit2")

    pinion_comp = add_component(model, pinion_path)
    gear_comp = add_component(model, gear_path)

    clocking = mesh.gear_clocking(geo.gear.z)
    pinion_matrix = mesh.pinion_placement()
    gear_matrix = mesh.gear_placement(geo.params.sigma, clocking)

    place(session, pinion_comp, pinion_matrix, "pinion")
    place(session, gear_comp, gear_matrix, "gear")

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
        mesh.angle_between(measure_axis(pinion_comp), measure_axis(gear_comp))
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
        pinion_status=component_status(pinion_comp) if mate else "not mated",
        gear_status=component_status(gear_comp) if mate else "not mated",
        model=model,
    )
