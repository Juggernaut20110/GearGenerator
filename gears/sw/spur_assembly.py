"""Build the full spur set: both parts, then an assembly with them meshing.

Read `assembly_common` first - it carries the rationale both gear types share:
why the pair is *placed* by writing transforms and only *constrained*
afterwards, why every mate is swMateAlignCLOSEST, and why the gear mate cannot
disturb the clocking.

What is particular to a spur pair is the arrangement. The axes are parallel and
a **centre distance** apart, so the gear is translated rather than tilted, and
the two members are held quite differently:

    pinion      origin coincident with the assembly origin                (3)
                axis coincident with the assembly Top plane               (1)
                axis coincident with the assembly Right plane             (1)

    gear        axis parallel to the pinion axis                          (2)
                axis coincident with the assembly Top plane               (1)
                origin coincident with the assembly Front plane           (1)
                axis at distance a from the pinion axis                   (1)

Three things about the gear's half are worth stating, because the obvious
arrangement is wrong in all three:

**The gear cannot be mated to the Right or Front plane the way the pinion is.**
Its axis runs along +Z, and a line along +Z cannot lie in the Front plane, which
is the XY plane. Only the Top and Right planes contain a +Z direction, and Right
would force the gear onto x = 0 - which is precisely where it must not be.

**Nothing above locates the gear along its own axis except the origin mate.**
The bevel set gets all three translations from putting both origins on the
assembly origin, because the members share an apex. Here they are `a` apart, so
the axial position has to be asked for separately: the gear's origin sits at its
front face, and mating that point to the Front plane puts both front faces on
z = 0.

**The direction is a parallel mate, not an angle mate at zero.** An angle mate at
zero degrees solves to 180 as readily as to 0.

The centre distance is a distance mate between the two axes, which is the honest
place to put that number and where someone would look for it afterwards.

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
    SW_MATE_PARALLEL,
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
    pick_gear_origin = point_pick(
        origin_point(gear_comp, "the gear origin"), "the gear origin"
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
    # Direction first, then position - the same order as the pinion's, so that
    # each mate after the first has fewer freedoms left to argue about.
    mate((pick_gear_axis, pick_pinion_axis), SW_MATE_PARALLEL,
         "gear axis parallel to the pinion axis")
    mate((pick_gear_axis, top), SW_MATE_COINCIDENT,
         "gear axis - Top plane")
    mate((pick_gear_origin, front), SW_MATE_COINCIDENT,
         "gear front face - Front plane")
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
    # Absolute: SaveAs3 refuses a relative path with a bare "error 1", which
    # says nothing about what is wrong with it.
    out_dir = Path(out_dir).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    # A ring gear of a given module and tooth count is a different part from an
    # external one, so it needs a different filename or the two overwrite each
    # other - the same trap `part_filename`'s `prefix` was added for.
    prefix = "internal_" if geo.params.internal else "spur_"
    pinion_path = out_dir / part_filename(geo, "pinion", prefix=prefix)
    gear_path = out_dir / part_filename(geo, "gear", prefix=prefix)

    pinion = build_spur(session, geo, "pinion", str(pinion_path))
    gear = build_spur(session, geo, "gear", str(gear_path))

    model = session.new_assembly()
    flag_methods(model, "EditRebuild3", "ViewZoomtofit2")

    pinion_comp = add_component(model, pinion_path)
    gear_comp = add_component(model, gear_path)

    # `clocking_for` picks the internal or external rule; they are different
    # answers, not the same one reached twice - see `spur.mesh`.
    clocking = mesh.clocking_for(geo)
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
            out_dir / f"{prefix}m{module}_z{geo.pinion.z}x{geo.gear.z}.sldasm"
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
