"""Build the full spur set: both parts, then an assembly with them meshing.

Read `assembly_common` first - it carries the rationale both gear types share:
why the pair is *placed* by writing transforms and only *constrained*
afterwards, why every mate is swMateAlignCLOSEST, and why the gear mate cannot
disturb the clocking.

What is particular to a spur pair is the arrangement. The axes are parallel and
a **centre distance** apart, so the gear is translated rather than tilted:

    position    pinion: origin coincident with the assembly origin        (3)
                gear:   axis at distance a from the pinion axis           (1)
    axis        component axis coincident with the assembly Top plane     (1)
    direction   pinion: axis also coincident with the Right plane         (1)
                gear:   axis also coincident with the Front plane         (1)

Two differences from the bevel set are worth stating rather than leaving to be
inferred.

**The gear's position is a distance mate, not a coincident origin.** Both
members' origins land on the assembly origin in a bevel set because they share
an apex. Here they are `a` apart, and the honest place to put that number is a
distance mate between the two axes - which is also where someone would look for
it afterwards.

**The gear's direction is a second plane coincidence, not an angle mate.** The
bevel set uses an angle mate because no default plane is parallel to its gear
axis. Here every axis is parallel to +Z, so the Front plane serves, and an angle
mate at zero degrees is the kind of thing that solves to 180 as readily as to 0.

The *sense* of the gear mate has **not** been measured for a spur pair. An
external pair turns in opposite senses about parallel axes -
`placement.angular_velocity_ratio` proves it - but how SOLIDWORKS reads a gear
mate's Reverse flag against two reference axes is not something the API will
report, and the bevel answer does not transfer: its axes stand at 90 degrees to
each other and these are parallel. `--reverse-gear` is here from the first
build, and the measured answer belongs in this docstring once someone has
dragged the pinion and watched which way the gear went.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path

from ..spur import mesh
from ..spur.geometry import SpurSetGeometry
from .assembly_common import (
    EXPECTED_COMPONENT_STATUS,
    add_component,
    add_mate,
    check_interference,
    component_axis,
    component_status,
    feature_pick,
    measure_axis,
    measure_origin,
    origin_point,
    part_filename,
    place,
    plane_pick,
    point_pick,
    unfix,
)
from .common import BuildResult
from .session import (
    FRONT_PLANE_NAMES,
    RIGHT_PLANE_NAMES,
    SW_MATE_COINCIDENT,
    SW_MATE_DISTANCE,
    SW_MATE_GEAR,
    TOP_PLANE_NAMES,
    flag_methods,
)
from .spur_part import build_spur

__all__ = ["SpurSetResult", "build_spur_set"]


@dataclass
class SpurSetResult:
    """Both parts plus the assembly, and the checks made on the result."""

    pinion: BuildResult
    gear: BuildResult
    assembly_title: str
    assembly_path: str | None
    centre_distance_mm: float
    clocking_deg: float
    measured_centre_distance_mm: float
    measured_axis_angle_deg: float
    interference_count: int = -1
    interference_volume_mm3: float = 0.0
    mates: tuple[str, ...] = ()
    gear_ratio: tuple[float, float] | None = None
    gear_mate_reversed: bool = False
    pinion_status: str = "not checked"
    gear_status: str = "not checked"
    model: object = field(default=None, repr=False)

    @property
    def centre_distance_error_mm(self) -> float:
        return self.measured_centre_distance_mm - self.centre_distance_mm

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


def add_mates(
    model,
    pinion_comp,
    gear_comp,
    geo: SpurSetGeometry,
    reverse: bool = False,
):
    """Constrain the pair so it articulates, and couple it with a gear mate.

    Returns the names of the mates added, in the order they were added, and the
    gear ratio. See the module docstring for the degree-of-freedom bookkeeping -
    the short version is that each member keeps its spin and the gear mate joins
    the two.
    """
    unfix(model, pinion_comp, "pinion")
    unfix(model, gear_comp, "gear")

    origin = point_pick(
        origin_point(model, "the assembly origin"), "the assembly origin"
    )
    top = plane_pick(TOP_PLANE_NAMES, "the assembly Top plane")
    right = plane_pick(RIGHT_PLANE_NAMES, "the assembly Right plane")
    front = plane_pick(FRONT_PLANE_NAMES, "the assembly Front plane")

    pick_pinion_axis = feature_pick(
        component_axis(pinion_comp, "pinion"), "the pinion axis"
    )
    pick_gear_axis = feature_pick(
        component_axis(gear_comp, "gear"), "the gear axis"
    )
    pick_pinion_origin = point_pick(
        origin_point(pinion_comp, "the pinion origin"), "the pinion origin"
    )

    added: list[str] = []

    def mate(picks, mate_type: int, what: str, **kwargs) -> None:
        add_mate(model, picks, mate_type, what, **kwargs)
        added.append(what)

    # The pinion origin goes on the assembly origin first: it takes all three
    # translations away, so every mate after it has only rotations left to
    # remove and cannot over-define the assembly by taking a translation twice.
    mate((pick_pinion_origin, origin), SW_MATE_COINCIDENT,
         "pinion origin - assembly origin")
    mate((pick_pinion_axis, top), SW_MATE_COINCIDENT,
         "pinion axis - Top plane")
    mate((pick_pinion_axis, right), SW_MATE_COINCIDENT,
         "pinion axis - Right plane")
    mate((pick_gear_axis, top), SW_MATE_COINCIDENT,
         "gear axis - Top plane")
    mate((pick_gear_axis, front), SW_MATE_COINCIDENT,
         "gear axis - Front plane")
    mate((pick_gear_axis, pick_pinion_axis), SW_MATE_DISTANCE,
         f"centre distance {geo.centre_distance:.4f} mm",
         distance=geo.centre_distance)

    ratio = mesh.gear_mate_ratio(geo.pinion.z, geo.gear.z)
    mate((pick_pinion_axis, pick_gear_axis), SW_MATE_GEAR,
         f"gear mate {geo.pinion.z}:{geo.gear.z}"
         + (" reversed" if reverse else ""),
         ratio=ratio, flip=bool(reverse))

    return tuple(added), ratio


def build_spur_set(
    session,
    geo: SpurSetGeometry,
    out_dir: str | Path,
    save_assembly: bool = True,
    mate: bool = True,
    reverse_gear_mate: bool = False,
) -> SpurSetResult:
    """Build both members and an assembly with their teeth meshing.

    With `mate` on - the default - the pair is also constrained and coupled by
    a gear mate, so the saved assembly turns. Off, it is placed and left
    floating, which is worth having when a mate is the thing under suspicion.
    """
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    pinion_path = out_dir / part_filename(geo, "pinion")
    gear_path = out_dir / part_filename(geo, "gear")

    pinion = build_spur(session, geo, "pinion", str(pinion_path))
    gear = build_spur(session, geo, "gear", str(gear_path))

    model = session.new_assembly()
    flag_methods(model, "EditRebuild3", "ViewZoomtofit2")

    pinion_comp = add_component(model, pinion_path)
    gear_comp = add_component(model, gear_path)

    clocking = mesh.gear_clocking(geo.gear.z)
    place(session, pinion_comp, mesh.pinion_placement(), "pinion")
    place(
        session,
        gear_comp,
        mesh.gear_placement(clocking),
        "gear",
        translation=mesh.gear_translation(geo),
    )

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

    # Measured after the rebuild, so both report where the *mates* left the
    # components rather than where the transforms put them. A distance mate that
    # solved to the wrong side would show up here as a negative centre distance,
    # and any stray tilt as a non-zero axis angle.
    pinion_origin = measure_origin(pinion_comp)
    gear_origin = measure_origin(gear_comp)
    measured_centre_distance = math.dist(pinion_origin, gear_origin)
    measured_axis_angle = math.degrees(
        mesh.angle_between(measure_axis(pinion_comp), measure_axis(gear_comp))
    )
    interference_count, interference_volume = check_interference(model)

    path = None
    if save_assembly:
        module = f"{geo.params.module:g}".replace(".", "p")
        path = str(
            out_dir / f"spur_m{module}_z{geo.pinion.z}x{geo.gear.z}.sldasm"
        )
        session.save(model, path)

    return SpurSetResult(
        pinion=pinion,
        gear=gear,
        assembly_title=model.GetTitle,
        assembly_path=path,
        centre_distance_mm=geo.centre_distance,
        clocking_deg=math.degrees(clocking),
        measured_centre_distance_mm=measured_centre_distance,
        measured_axis_angle_deg=measured_axis_angle,
        interference_count=interference_count,
        interference_volume_mm3=interference_volume,
        mates=mates,
        gear_ratio=ratio,
        gear_mate_reversed=bool(reverse_gear_mate),
        pinion_status=component_status(pinion_comp) if mate else "not mated",
        gear_status=component_status(gear_comp) if mate else "not mated",
        model=model,
    )
