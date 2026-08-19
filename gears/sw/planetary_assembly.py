"""Build a full planetary set: three parts, then an assembly of N + 2 components.

Read `assembly_common` first - it carries the rationale every gear type shares:
why the members are *placed* by writing transforms and only *constrained*
afterwards, why every mate is swMateAlignCLOSEST, and why a gear mate cannot
disturb the clocking that the transforms wrote.

Three parts, more components
----------------------------
A pair builds two parts and inserts each once. A planetary train builds **three**
- a sun, one planet and a ring - and inserts the planet `N` times. That is the
only structural difference from `spur_assembly`, and it is the reason
`SpurSetResult.parts` grew into something a caller reads rather than a pair of
named attributes.

The arrangement
---------------
Every axis is parallel to +Z and every front face is at z = 0, so this is the
spur pair's arrangement with more members in it rather than a new one:

    sun         origin coincident with the assembly origin                (3)
                axis coincident with the assembly Top plane               (1)
                axis coincident with the assembly Right plane             (1)

    ring        axis coincident with the sun axis                         (2)
                origin coincident with the assembly Front plane           (1)
                one point locked against rotation? NO - see below         (0)

    planet k    axis parallel to the sun axis                             (2)
                axis coincident with the assembly Top plane   (k = 0 only) (1)
                origin coincident with the assembly Front plane           (1)
                axis at distance a from the sun axis                      (1)

**The ring is concentric with the sun, not offset from it.** A coincident mate
between the two reference axes takes both of its remaining translations at once,
where a spur gear needed a parallel mate and a distance mate to say the same
thing about a different arrangement. The ring keeps its spin, like everyone else.

**Only planet 0 can be mated to the Top plane.** The Top plane is the XZ plane,
so an axis lying in it is an axis at y = 0 - true of the planet on the +X station
and of nobody else. Every other planet is held by its distance from the sun axis
plus an angle mate to the Top plane, because those are the two numbers that
actually place it.

Two things this assembly does NOT do
------------------------------------
**There is no carrier.** The v1 assembly is the *carrier-stationary*
configuration: every member spins about an axis fixed in space, which is a real
and useful planetary arrangement (it is the one that gives the ring-to-sun ratio
of -z_r/z_s) and it needs no carrier solid to be correct. A carrier part with
orbiting planets is a genuinely different assembly - the planets' axes move, so
they cannot be mated to the assembly's own planes at all - and belongs in a
later pass.

**The gear mates' sense is not measured.** Two of them per planet, and neither
inherits from anything already known: the sun-planet mesh is an external pair
whose own answer is still open, and the planet-ring mesh is internal, which
turns the *same* way rather than the opposite way. `--reverse-gear` flips both.
Drag the sun and watch; the answer belongs in this docstring.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path

from ..planetary import mesh
from ..planetary.geometry import PlanetarySetGeometry
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
    place,
    plane_pick,
    point_pick,
    unfix,
)
from .common import BuildResult
from .session import (
    FRONT_PLANE_NAMES,
    RIGHT_PLANE_NAMES,
    SW_MATE_ANGLE,
    SW_MATE_COINCIDENT,
    SW_MATE_DISTANCE,
    SW_MATE_GEAR,
    SW_MATE_PARALLEL,
    TOP_PLANE_NAMES,
    flag_methods,
)
from .spur_part import build_spur

__all__ = ["PlanetarySetResult", "build_planetary_set"]


def part_filename(geo: PlanetarySetGeometry, member: str) -> str:
    """`planetary_sun_m2_z24.sldprt`. The module's decimal point becomes a p.

    Prefixed like the other types' parts, so a planetary sun of 24 teeth at
    module 2 does not claim the same filename as a spur pinion of 24 teeth at
    module 2 and quietly overwrite it.
    """
    m = geo.member(member)
    module = f"{geo.params.module:g}".replace(".", "p")
    return f"planetary_{member}_m{module}_z{m.z}.sldprt"


@dataclass
class PlanetarySetResult:
    """All three parts plus the assembly, and the checks made on the result."""

    sun: BuildResult
    planet: BuildResult
    ring: BuildResult
    assembly_title: str
    assembly_path: str | None
    centre_distance_mm: float
    planet_count: int
    clockings_deg: tuple[float, ...] = ()
    ring_clocking_deg: float = 0.0
    measured_positions_mm: tuple[tuple[float, float, float], ...] = ()
    measured_axis_angles_deg: tuple[float, ...] = ()
    interference_count: int = -1
    interference_volume_mm3: float = 0.0
    mates: tuple[str, ...] = ()
    gear_ratios: tuple[tuple[float, float], ...] = ()
    gear_mate_reversed: bool = False
    statuses: dict[str, str] = field(default_factory=dict)
    model: object = field(default=None, repr=False)

    @property
    def parts(self) -> tuple:
        """The three distinct parts. The planet is inserted `planet_count` times."""
        return (self.sun, self.planet, self.ring)

    @property
    def planets(self) -> tuple:
        """Every planet component's measured position. Length is `planet_count`."""
        return self.measured_positions_mm

    @property
    def worst_position_error_mm(self) -> float:
        """How far the worst-placed planet sits from its nominal orbit radius.

        Measured after the rebuild, so it reports where the *mates* left each
        component rather than where the transform put it. A distance mate that
        solved to the wrong side would show up here as a large error rather than
        as a picture nobody looked at.
        """
        if not self.measured_positions_mm:
            return 0.0
        return max(
            math.hypot(x, y) - self.centre_distance_mm
            for x, y, _ in self.measured_positions_mm
        )

    @property
    def articulates(self) -> bool:
        """Every member still free to spin, with gear mates tying them together.

        A member that came back fully defined has been pinned by a mate that
        removed six degrees of freedom instead of five, and the train will not
        turn however good the gear mates are.
        """
        return bool(self.gear_ratios) and bool(self.statuses) and all(
            status == EXPECTED_COMPONENT_STATUS for status in self.statuses.values()
        )


def add_mates(model, sun_comp, ring_comp, planet_comps, geo, reverse=False):
    """Constrain the train so it articulates, and couple it with gear mates.

    Returns the mate names in the order they were added, and every gear ratio.
    See the module docstring for the degree-of-freedom bookkeeping; the short
    version is that each member keeps its spin about its own axis, and the gear
    mates join those spins into one train.
    """
    unfix(model, sun_comp, "sun")
    unfix(model, ring_comp, "ring")
    for k, comp in enumerate(planet_comps):
        unfix(model, comp, f"planet {k}")

    origin = point_pick(
        origin_point(model, "the assembly origin"), "the assembly origin"
    )
    top = plane_pick(TOP_PLANE_NAMES, "the assembly Top plane")
    right = plane_pick(RIGHT_PLANE_NAMES, "the assembly Right plane")
    front = plane_pick(FRONT_PLANE_NAMES, "the assembly Front plane")

    pick_sun_axis = feature_pick(component_axis(sun_comp, "sun"), "the sun axis")
    pick_ring_axis = feature_pick(component_axis(ring_comp, "ring"), "the ring axis")
    pick_sun_origin = point_pick(
        origin_point(sun_comp, "the sun origin"), "the sun origin"
    )
    pick_ring_origin = point_pick(
        origin_point(ring_comp, "the ring origin"), "the ring origin"
    )

    added: list[str] = []

    def mate(picks, mate_type: int, what: str, **kwargs) -> None:
        add_mate(model, picks, mate_type, what, **kwargs)
        added.append(what)

    # The sun first, and its origin first of all: that takes all three
    # translations away, so every mate after it has only rotations left to
    # remove and cannot over-define the assembly by taking a translation twice.
    mate((pick_sun_origin, origin), SW_MATE_COINCIDENT, "sun origin - assembly origin")
    mate((pick_sun_axis, top), SW_MATE_COINCIDENT, "sun axis - Top plane")
    mate((pick_sun_axis, right), SW_MATE_COINCIDENT, "sun axis - Right plane")

    # The ring is concentric with the sun, so one coincident mate between the
    # two axes settles both of its translations and both of its tilts at once -
    # where a spur gear needed a parallel mate and a distance mate to say
    # something quite different. Its front face still has to be located
    # separately, exactly as a spur gear's does.
    mate((pick_ring_axis, pick_sun_axis), SW_MATE_COINCIDENT,
         "ring axis - sun axis")
    mate((pick_ring_origin, front), SW_MATE_COINCIDENT,
         "ring front face - Front plane")

    for k, comp in enumerate(planet_comps):
        pick_axis = feature_pick(
            component_axis(comp, f"planet {k}"), f"the planet {k} axis"
        )
        pick_origin = point_pick(
            origin_point(comp, f"the planet {k} origin"), f"the planet {k} origin"
        )
        # Direction first, then position - the same order as everywhere else, so
        # each mate after the first has fewer freedoms left to argue about.
        # Parallel rather than an angle mate at zero, because an angle mate at
        # zero degrees solves to 180 as readily as to 0.
        mate((pick_axis, pick_sun_axis), SW_MATE_PARALLEL,
             f"planet {k} axis parallel to the sun axis")
        mate((pick_origin, front), SW_MATE_COINCIDENT,
             f"planet {k} front face - Front plane")
        mate((pick_axis, pick_sun_axis), SW_MATE_DISTANCE,
             f"planet {k} orbit radius {geo.centre_distance:.4f} mm",
             distance=geo.centre_distance)

        # What is left is *where round the orbit*. Planet 0 sits at y = 0, which
        # is what the Top plane is, so it takes a coincident mate; the others
        # are at a real angle to it and take an angle mate carrying that number.
        # The angle is the honest place to put it and where someone would look.
        angle = mesh.carrier_angle(geo, k)
        if k == 0:
            mate((pick_axis, top), SW_MATE_COINCIDENT,
                 f"planet {k} axis - Top plane")
        else:
            mate((pick_axis, top), SW_MATE_ANGLE,
                 f"planet {k} carrier angle {math.degrees(angle):.4f} deg",
                 angle=angle)

    # --- the gear mates ----------------------------------------------------
    #
    # Two per planet: one to the sun and one to the ring. Every planet gets
    # both, which over-constrains the *train* in the sense that any one planet
    # would suffice to relate the sun to the ring - but each planet has a spin
    # of its own that has to be tied to something, and leaving the others
    # uncoupled would let them sit still while the train turned.
    ratios: list[tuple[float, float]] = []
    for k, comp in enumerate(planet_comps):
        pick_axis = feature_pick(
            component_axis(comp, f"planet {k}"), f"the planet {k} axis"
        )
        sun_ratio = mesh.sun_planet_ratio(geo)
        mate((pick_sun_axis, pick_axis), SW_MATE_GEAR,
             f"gear mate sun:planet {k} "
             f"{geo.params.z_sun}:{geo.params.z_planet}"
             + (" reversed" if reverse else ""),
             ratio=sun_ratio, flip=bool(reverse))
        ratios.append(sun_ratio)

        ring_ratio = mesh.planet_ring_ratio(geo)
        mate((pick_axis, pick_ring_axis), SW_MATE_GEAR,
             f"gear mate planet {k}:ring "
             f"{geo.params.z_planet}:{geo.params.z_ring}"
             + (" reversed" if reverse else ""),
             ratio=ring_ratio, flip=bool(reverse))
        ratios.append(ring_ratio)

    return tuple(added), tuple(ratios)


def build_planetary_set(
    session,
    geo: PlanetarySetGeometry,
    out_dir: str | Path,
    save_assembly: bool = True,
    mate: bool = True,
    reverse_gear_mate: bool = False,
) -> PlanetarySetResult:
    """Build all three parts and an assembly with the whole train meshing.

    With `mate` on - the default - the train is also constrained and coupled by
    gear mates, so the saved assembly turns. Off, it is placed and left
    floating, which is worth having when a mate is the thing under suspicion.
    """
    # Absolute: SaveAs3 refuses a relative path with a bare "error 1", which
    # says nothing about what is wrong with it.
    out_dir = Path(out_dir).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    p = geo.params
    paths = {
        name: out_dir / part_filename(geo, name)
        for name in ("sun", "planet", "ring")
    }

    # Each part is built out of the spur pair it belongs to, in the slot that
    # pair calls it - `mesh_for` and `role_of` do that lookup, so the part
    # builder never learns what a planetary set is.
    built = {
        name: build_spur(
            session, geo.mesh_for(name), geo.role_of(name), str(paths[name])
        )
        for name in ("sun", "planet", "ring")
    }

    model = session.new_assembly()
    flag_methods(model, "EditRebuild3", "ViewZoomtofit2")

    sun_comp = add_component(model, paths["sun"])
    ring_comp = add_component(model, paths["ring"])
    # The same part file, inserted N times. Each instance is placed by its own
    # transform, so nothing about the part has to know which station it is on.
    planet_comps = [
        add_component(model, paths["planet"]) for _ in range(p.n_planets)
    ]

    place(session, sun_comp, mesh.member_placement(mesh.sun_clocking()), "sun")
    place(
        session, ring_comp,
        mesh.member_placement(mesh.ring_clocking(geo, 0)), "ring",
    )
    clockings = []
    for k, comp in enumerate(planet_comps):
        clocking = mesh.planet_clocking(geo, k)
        clockings.append(clocking)
        place(
            session, comp, mesh.member_placement(clocking), f"planet {k}",
            translation=mesh.planet_translation(geo, k),
        )

    mates: tuple[str, ...] = ()
    ratios: tuple[tuple[float, float], ...] = ()
    if mate:
        mates, ratios = add_mates(
            model, sun_comp, ring_comp, planet_comps, geo,
            reverse=reverse_gear_mate,
        )

    model.EditRebuild3()
    try:
        model.ViewZoomtofit2()
    except Exception:
        pass

    # Measured after the rebuild, so these report where the mates left each
    # component rather than where the transforms put them.
    positions = tuple(measure_origin(comp) for comp in planet_comps)
    sun_axis = measure_axis(sun_comp)
    axis_angles = tuple(
        math.degrees(mesh.angle_between(sun_axis, measure_axis(comp)))
        for comp in (ring_comp, *planet_comps)
    )
    interference_count, interference_volume = check_interference(model)

    path = None
    if save_assembly:
        module = f"{p.module:g}".replace(".", "p")
        path = str(
            out_dir
            / f"planetary_m{module}_s{p.z_sun}p{p.z_planet}r{p.z_ring}"
              f"x{p.n_planets}.sldasm"
        )
        session.save(model, path)

    statuses = {}
    if mate:
        statuses["sun"] = component_status(sun_comp)
        statuses["ring"] = component_status(ring_comp)
        for k, comp in enumerate(planet_comps):
            statuses[f"planet {k}"] = component_status(comp)

    return PlanetarySetResult(
        sun=built["sun"],
        planet=built["planet"],
        ring=built["ring"],
        assembly_title=model.GetTitle,
        assembly_path=path,
        centre_distance_mm=geo.centre_distance,
        planet_count=p.n_planets,
        clockings_deg=tuple(math.degrees(c) for c in clockings),
        ring_clocking_deg=math.degrees(mesh.ring_clocking(geo, 0)),
        measured_positions_mm=positions,
        measured_axis_angles_deg=axis_angles,
        interference_count=interference_count,
        interference_volume_mm3=interference_volume,
        mates=mates,
        gear_ratios=ratios,
        gear_mate_reversed=bool(reverse_gear_mate),
        statuses=statuses,
        model=model,
    )
