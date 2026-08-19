"""Planetary parameter validation. No COM, no GUI.

Errors block a build; warnings let it proceed but say what is unusual. Same
contract as the other two validators: never raises, reports everything it can
find in one pass, and early-returns only where carrying on would ask
`compute_set` to divide by zero.

Most of the work is delegated. Both meshes are spur meshes, so
`spur.validate.validate` already has plenty to say about each of them, and this
module runs it on both and relabels what comes back. What is genuinely new here
is the handful of conditions that only exist because there is a *train*: the
tooth-count relation, the assembly condition, and the planets not touching each
other.
"""

from __future__ import annotations

import math

from ..validate import (
    MAX_PRESSURE_ANGLE,
    MIN_PRESSURE_ANGLE,
    MIN_TEETH,
    Issue,
    ValidationResult,
)
from ..spur.validate import MAX_HELIX_ANGLE, validate as validate_spur
from .geometry import (
    centre_distance_from_ring,
    compute_set,
    planet_ring_params,
    sun_planet_params,
)
from .params import PlanetarySetParams

__all__ = ["Issue", "ValidationResult", "validate"]

# Fewest planets worth calling a planetary set. One is a legitimate machine -
# it is just an idler between a sun and a ring - and it needs no assembly
# condition, so it is allowed with a warning rather than refused.
MIN_PLANETS = 1

# Most that will fit before the neighbour check refuses them anyway. A rail
# rather than a design limit; the geometry says the real answer.
MAX_PLANETS = 12

# How much daylight to insist on between adjacent planets' tip circles, as a
# multiple of the module. They must not touch, and touching exactly is not a
# design - it is a bearing that has to be built to nothing.
MIN_PLANET_GAP_FACTOR = 0.5


def _check_basics(p: PlanetarySetParams, r: ValidationResult) -> None:
    """Checks that must pass before the geometry can even be computed."""
    if p.module <= 0:
        r.error("module", "must be greater than zero")
    if p.z_sun < MIN_TEETH:
        r.error("z_sun", f"the sun needs at least {MIN_TEETH} teeth")
    if p.z_planet < MIN_TEETH:
        r.error("z_planet", f"a planet needs at least {MIN_TEETH} teeth")
    if not (MIN_PRESSURE_ANGLE <= p.pressure_angle <= MAX_PRESSURE_ANGLE):
        r.error(
            "pressure_angle",
            f"must be between {MIN_PRESSURE_ANGLE} and {MAX_PRESSURE_ANGLE} degrees",
        )
    if not (-MAX_HELIX_ANGLE < p.helix_angle < MAX_HELIX_ANGLE):
        r.error(
            "helix_angle",
            f"must be between -{MAX_HELIX_ANGLE} and {MAX_HELIX_ANGLE} degrees "
            "(exclusive)",
        )
    if p.hand not in ("right", "left"):
        r.error("hand", "must be 'right' or 'left'")
    if not (MIN_PLANETS <= p.n_planets <= MAX_PLANETS):
        r.error(
            "n_planets",
            f"must be between {MIN_PLANETS} and {MAX_PLANETS}",
        )
    if p.face_width <= 0:
        r.error("face_width", "must be greater than zero")
    if p.bore < 0:
        r.error("bore", "cannot be negative")
    if p.hub_thickness < 0:
        r.error("hub_thickness", "cannot be negative")
    if p.rim_thickness < 0:
        r.error("rim_thickness", "cannot be negative")
    # Only the sign is checked here. Whether the backlash is *too much* for the
    # circular pitch is a spur mesh's question, and `_relay_mesh_issues` already
    # asks it on both meshes - asking it again here would report one warning
    # twice, because the relay's duplicate filter cannot see what this function
    # has already said.
    if p.backlash < 0:
        r.error("backlash", "cannot be negative")


def validate(p: PlanetarySetParams) -> ValidationResult:
    """Validate a parameter set. Never raises; report everything it can."""
    result = ValidationResult()
    _check_basics(p, result)
    if not result.ok:
        return result

    # --- the tooth-count relation ------------------------------------------
    #
    # Checked rather than assumed, even though `z_ring` is derived from exactly
    # this relation and so cannot fail. That is the point: it solves the centre
    # distance two ways that share no terms - m_t(z_s + z_p)/2 against
    # m_t(z_r - z_p)/2 - so if the derivation is ever changed to take a ring
    # count as an input, this is what notices.
    from_sun = p.centre_distance
    from_ring = centre_distance_from_ring(p)
    if abs(from_sun - from_ring) > 1e-9:
        result.error(
            "z_planet",
            f"the sun-planet centre distance ({from_sun:.4f} mm) and the "
            f"planet-ring centre distance ({from_ring:.4f} mm) disagree; a "
            f"planetary set needs z_ring = z_sun + 2 z_planet, which is "
            f"{p.z_sun} + 2 x {p.z_planet} = {p.z_ring}",
        )
        return result

    geo = compute_set(p)

    # --- the assembly condition --------------------------------------------
    #
    # Not a rule of thumb and not a manufacturing nicety: below it there is no
    # single ring clocking that meshes with every planet, so the last planet
    # physically cannot be fitted. See `mesh.ring_clocking` for where it comes
    # from.
    if p.n_planets > 1 and p.assembly_remainder != 0:
        total = p.z_sun + p.z_ring
        workable = [
            n for n in range(2, MAX_PLANETS + 1) if total % n == 0
        ]
        result.error(
            "n_planets",
            f"(z_sun + z_ring) = {total} is not divisible by {p.n_planets} "
            f"planets (remainder {p.assembly_remainder}), so there is no ring "
            f"clocking that meshes with all of them; "
            + (
                f"{', '.join(str(n) for n in workable)} planets would fit"
                if workable
                else "no planet count in range fits these teeth"
            ),
        )

    if p.n_planets == 1:
        result.warn(
            "n_planets",
            "one planet is an idler between the sun and the ring rather than a "
            "planetary set; it works, but nothing balances the radial load",
        )

    # --- do the planets clear each other? ----------------------------------
    #
    # The constraint that bites first when someone asks for more planets, and it
    # tightens fast: the spacing is 2 a sin(pi/N), which falls off while the
    # planets themselves stay the same size.
    if p.n_planets > 1:
        spacing = geo.neighbour_spacing
        tip_to_tip = spacing - 2.0 * geo.planet.tip_r
        needed = MIN_PLANET_GAP_FACTOR * p.module
        if tip_to_tip <= 0.0:
            result.error(
                "n_planets",
                f"{p.n_planets} planets of {2.0 * geo.planet.tip_r:.2f} mm "
                f"diameter do not fit on an orbit that spaces their axes "
                f"{spacing:.2f} mm apart; they overlap by "
                f"{-tip_to_tip:.2f} mm",
            )
        elif tip_to_tip < needed:
            result.warn(
                "n_planets",
                f"only {tip_to_tip:.2f} mm between adjacent planet tips "
                f"({tip_to_tip / p.module:.2f} * module); there is no room for a "
                "carrier plate between them",
            )

    # --- everything a spur mesh already knows how to say --------------------
    _relay_mesh_issues(p, result)

    # --- the sun's own blank ------------------------------------------------
    min_wall = max(p.module, 1.0)
    wall = geo.sun.root_r - p.bore / 2.0
    if wall <= 0.0:
        result.error(
            "bore",
            f"the bore is wider than the sun's root circle "
            f"({2.0 * geo.sun.root_r:.2f} mm); there is no material under its teeth",
        )
    elif wall < min_wall:
        result.warn(
            "bore",
            f"only {wall:.2f} mm of wall under the sun's teeth; "
            f"{min_wall:.2f} mm is the usual minimum",
        )

    return result


def _relay_mesh_issues(p: PlanetarySetParams, result: ValidationResult) -> None:
    """Run the spur validator on both meshes and relabel what it says.

    Relabelled rather than passed through, because the spur validator talks
    about a "pinion" and a "gear" and this set has a sun, planets and a ring.
    An issue reading `z2: the ring's tip radius is inside its base circle` is
    useful; one reading `z2: gear ...` on a set with no field called z2 is not.

    Duplicates are dropped. The planet appears in both meshes, so anything said
    about the planet's own tooth form - its undercut limit, its top land - would
    otherwise be reported twice for one gear.
    """
    seen: set[tuple[str, str]] = set()

    for pair_params, names in (
        (sun_planet_params(p), {"z1": "z_sun", "z2": "z_planet",
                                "pinion": "sun", "gear": "planet"}),
        (planet_ring_params(p), {"z1": "z_planet", "z2": "z_ring",
                                 "pinion": "planet", "gear": "ring"}),
    ):
        pair_result = validate_spur(pair_params)
        for issues, add in (
            (pair_result.errors, result.error),
            (pair_result.warnings, result.warn),
        ):
            for issue in issues:
                field = names.get(issue.field, issue.field)
                message = issue.message
                for old, new in names.items():
                    if old in ("z1", "z2"):
                        continue
                    message = message.replace(old, new)
                if (field, message) in seen:
                    continue
                seen.add((field, message))
                add(field, message)
