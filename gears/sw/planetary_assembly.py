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
                origin coincident with the assembly Front plane           (1)
                axis at distance a from the sun axis                      (1)
                axis at distance a sin(theta_k) from the Top plane        (1)
                  - coincident with it instead when that distance is zero

**The ring is concentric with the sun, not offset from it.** A coincident mate
between the two reference axes takes both of its remaining translations at once,
where a spur gear needed a parallel mate and a distance mate to say the same
thing about a different arrangement. The ring keeps its spin, like everyone else.

**A planet's station round the orbit is a distance, not an angle.** The Top
plane is the XZ plane, so an axis lying *in* it is an axis at y = 0 - true of the
planet on the +X station and of nobody else, which is why only that one takes a
coincident mate. The obvious spelling for the rest is an angle mate carrying the
carrier angle, and it is impossible: an angle mate between a line and a plane
measures the angle *to the plane*, and every planet axis runs along +Z while the
Top plane contains +Z, so that angle is identically 0 wherever the planet sits.
Measured on the anchor set, planet 1 at 120 deg came back "unknown error" from
AddMate5 - a refusal to build an unsatisfiable mate.

So each planet is held by its distance from the sun axis and its own **y**,
`a sin(theta_k)`, as a distance from the Top plane. Those two numbers fix the
station up to the sign of x, and swMateAlignCLOSEST settles that sign against
the transform that already placed the component - which is exactly the division
of labour this builder is built on.

What this assembly does NOT do
------------------------------
**There is no carrier.** The v1 assembly is the *carrier-stationary*
configuration: every member spins about an axis fixed in space, which is a real
and useful planetary arrangement (it is the one that gives the ring-to-sun ratio
of -z_r/z_s) and it needs no carrier solid to be correct. A carrier part with
orbiting planets is a genuinely different assembly - the planets' axes move, so
they cannot be mated to the assembly's own planes at all - and belongs in a
later pass.

The gear mates' sense
---------------------
Two kinds of them, and neither inherits from anything already known: the
sun-planet mesh is an external pair, and the planet-ring mesh is internal, which
turns the *same* way rather than the opposite way. Measured by hand on the
anchor set, by dragging the sun and watching:

    sun:planet    Reverse off    correct
    planet:ring   Reverse off    backwards - the ring turned against the planet

So the two meshes carry **different** Reverse flags, which is the geometry
showing through the flag: an internal pair turning the same way as its pinion is
the one thing about this train that a spur pair cannot tell you. Each sense is a
named constant below, and `--reverse-gear` flips *both* away from their measured
values - it is the escape hatch for the day one of them stops holding, not a
setting with a right value of its own.
"""

from __future__ import annotations

import dataclasses
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
    SW_MATE_COINCIDENT,
    SW_MATE_DISTANCE,
    SW_MATE_GEAR,
    SW_MATE_PARALLEL,
    TOP_PLANE_NAMES,
    flag_methods,
)
from .spur_part import build_spur

__all__ = ["PlanetarySetResult", "build_planetary_set"]

# Below this, a planet's offset from the Top plane is treated as zero and the
# mate becomes coincident instead. A distance mate of zero cannot be created -
# the same limitation the spur builder's front face runs into - and the planets
# this catches are exactly the ones that really do sit at y = 0: the +X station,
# and the 180 deg station an even planet count produces.
ZERO_OFFSET_MM = 1e-9

# The measured Reverse flag for each of the train's two meshes, with the first
# axis of the pair selected first - sun:planet is added (sun, planet) and
# planet:ring is added (planet, ring). Measured by hand on the anchor set by
# dragging the sun, because a gear mate is applied by the interactive drag
# solver and by nothing else: the sun and the planets came out right with
# Reverse off and the ring came out backwards, so its flag goes on and theirs
# stays off. They are two constants rather than one because they are two
# independent measurements - the meshes are of different kinds, and an internal
# pair turns the same way as its pinion where an external pair opposes it.
#
# `reverse_gear_mate` flips both away from these, which is what makes it an
# escape hatch rather than a setting: with it off, the train turns the way it
# was measured to turn.
SUN_PLANET_GEAR_MATE_FLIP = False
PLANET_RING_GEAR_MATE_FLIP = True


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
    # Both meshes flipped *away* from their measured senses, not "the Reverse
    # flag is on" - the ring mate's flag is on by default. `mates` carries the
    # per-mate truth, since each name says "reversed" when its own flag did.
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
        """How far the worst-placed planet sits from **its own** station.

        Measured after the rebuild, so it reports where the *mates* left each
        component rather than where the transform put it.

        **Against the station, not against the radius.** Comparing
        `hypot(x, y)` with the orbit radius was the first version and it is
        blind to the failure that actually happened: two planets mated to the
        same distance from the Top plane both landed on the +y station, one
        inside the other, and every one of them was still exactly 42 mm from
        the axis. That check reported 0.00e+00 while the assembly held 18552
        mm3 of interference. A planet's station is two numbers and the check
        has to use both.
        """
        if not self.measured_positions_mm:
            return 0.0
        worst = 0.0
        for k, (x, y, _) in enumerate(self.measured_positions_mm):
            theta = 2.0 * math.pi * k / self.planet_count
            worst = max(
                worst,
                math.hypot(
                    x - self.centre_distance_mm * math.cos(theta),
                    y - self.centre_distance_mm * math.sin(theta),
                ),
            )
        return worst

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

        # What is left is *where round the orbit*, and it is a distance rather
        # than an angle. An angle mate carrying the carrier angle is the
        # obvious spelling and it is impossible: the angle between a line and a
        # plane is measured to the plane, and every planet axis runs along +Z
        # while the Top plane is the XZ plane, so that angle is identically 0
        # at every station. SOLIDWORKS refuses it - measured on the anchor set,
        # planet 1 at 120 deg came back "unknown error" from AddMate5 - and it
        # is refusing an unsatisfiable mate, not being awkward. It is the same
        # fact that makes planet 0's coincident mate work, one step further on:
        # an axis lying *in* the XZ plane is an axis at y = 0.
        #
        # So the station goes in as the planet's own y, `a sin(theta)`, as a
        # distance from that same plane.
        #
        # **The sign has to be carried by the flip flag, not by the alignment.**
        # A distance mate's value is a magnitude, and the obvious reading - that
        # swMateAlignCLOSEST would keep the component on the side the transform
        # already put it - is wrong here, which cost a build to find out. On the
        # anchor set planets 1 and 2 are at y = +36.3731 and -36.3731, the same
        # magnitude, and mating both to |y| landed them *both* at +36.3731:
        # stacked on the same station, 18552.6958 mm3 of one planet inside the
        # other. `flip` is what distinguishes them.
        angle = mesh.carrier_angle(geo, k)
        offset = geo.centre_distance * math.sin(angle)
        if abs(offset) < ZERO_OFFSET_MM:
            # y = 0: planet 0 on the +X station, and any planet at 180 deg -
            # which an even planet count really does produce. A zero-length
            # distance mate cannot be created, and coincident is what zero
            # distance means anyway.
            mate((pick_axis, top), SW_MATE_COINCIDENT,
                 f"planet {k} axis - Top plane")
        else:
            mate((pick_axis, top), SW_MATE_DISTANCE,
                 f"planet {k} station {offset:+.4f} mm from the Top plane "
                 f"(carrier angle {math.degrees(angle):.4f} deg)",
                 distance=abs(offset), flip=offset < 0.0)

    # --- the gear mates ----------------------------------------------------
    #
    # **N + 1 of them, not two per planet.** Every member keeps exactly one
    # freedom - its own spin - so the train has N + 2 of them, and a gear mate
    # removes one. N + 1 mates therefore leave exactly one, which is the train
    # turning; the N + 2nd is refused, and rightly.
    #
    # Two per planet is the obvious arrangement and it is one too many from the
    # third mate onward. Measured on the anchor set: sun:planet 0, planet 0:ring
    # and sun:planet 1 all went on, and "gear mate planet 1:ring" came back
    # "the mate would over-define the assembly" - 4 mates on 5 members is the
    # whole budget, and the loop was asking for 6.
    #
    # So the mates form a **spanning tree** over the members: every planet is
    # coupled to the sun, and the ring hangs off planet 0. That is the shape the
    # count demands, and it keeps the property the two-per-planet version was
    # reaching for - no planet is left uncoupled to sit still while the train
    # turns - because a tree connects every member by construction. Which planet
    # carries the ring is arbitrary; planet 0 is the one that is always there.
    #
    # **The two meshes carry different Reverse flags**, and that is measured
    # rather than reasoned about - see the module docstring. `reverse` flips
    # both away from their measured senses, so the name printed for each mate
    # is the flag that actually went on it, not the state of the argument.
    sun_flip = SUN_PLANET_GEAR_MATE_FLIP != bool(reverse)
    ring_flip = PLANET_RING_GEAR_MATE_FLIP != bool(reverse)

    ratios: list[tuple[float, float]] = []
    for k, comp in enumerate(planet_comps):
        pick_axis = feature_pick(
            component_axis(comp, f"planet {k}"), f"the planet {k} axis"
        )
        sun_ratio = mesh.sun_planet_ratio(geo)
        mate((pick_sun_axis, pick_axis), SW_MATE_GEAR,
             f"gear mate sun:planet {k} "
             f"{geo.params.z_sun}:{geo.params.z_planet}"
             + (" reversed" if sun_flip else ""),
             ratio=sun_ratio, flip=sun_flip)
        ratios.append(sun_ratio)

        if k == 0:
            ring_ratio = mesh.planet_ring_ratio(geo)
            mate((pick_axis, pick_ring_axis), SW_MATE_GEAR,
                 f"gear mate planet {k}:ring "
                 f"{geo.params.z_planet}:{geo.params.z_ring}"
                 + (" reversed" if ring_flip else ""),
                 ratio=ring_ratio, flip=ring_flip)
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
    #
    # The name is then put back. `role_of` hands the builder "pinion" or
    # "gear", which is what a spur pair calls its members and what the
    # BuildResult would otherwise carry out to the caller - so the window's
    # build report named the three members "pinion, gear, gear" and left the
    # reader to guess which gear was the ring.
    built = {
        name: dataclasses.replace(
            build_spur(
                session, geo.mesh_for(name), geo.role_of(name), str(paths[name])
            ),
            member=name,
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
