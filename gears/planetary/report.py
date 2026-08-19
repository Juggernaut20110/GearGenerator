"""The terminal report for a planetary set. No SOLIDWORKS, no GUI.

The same three-column shape as the other two reports, widened by one: a
planetary train has three members rather than two, so the MEMBERS block carries
a SUN, PLANET and RING column instead of a PINION and a GEAR. Everything above
it lines up with the bevel and spur reports as before.
"""

from __future__ import annotations

import math

from ..report_format import emit_issues, row as _row
from ..spur.geometry import blank_outline
from . import mesh
from .geometry import compute_set
from .params import PlanetarySetParams
from .validate import validate

# Everything this type accepts, which is what the dispatcher's foreign-flag
# guard reads. It is deliberately a superset of what `add_arguments` below
# creates: `--beta`, `--rim` and `--backlash` mean exactly the same thing to a
# planetary set as to a spur one, so the spur module declares them and both
# modules claim them. A flag claimed by two types is refused only by the third.
FLAGS = (
    "alpha", "beta", "z_sun", "z_planet", "planets", "face_width", "bore",
    "hub", "rim", "backlash", "member",
)

MEMBERS = ("sun", "planet", "ring")


def add_arguments(ap) -> None:
    """Add only the flags no other type defines.

    `--beta`, `--rim` and `--backlash` are the spur module's to create - every
    `add_arguments` runs against the same parser, so creating them twice is an
    argparse conflict rather than a merge.
    """
    ap.add_argument("--z-sun", type=int, help="sun tooth count (or use --z1)")
    ap.add_argument("--z-planet", type=int, help="planet tooth count (or use --z2)")
    ap.add_argument("--planets", type=int, default=3, help="how many planets")


def params_from_args(args) -> PlanetarySetParams:
    """Read the flags into a parameter set.

    `--z1` and `--z2` are the shared required flags, so a planetary set accepts
    them as the sun and the planet - and `--z-sun` / `--z-planet` override them
    when someone would rather say what they mean. The ring is derived either way
    and there is deliberately no flag for it.
    """
    z_sun = args.z_sun if args.z_sun is not None else args.z1
    z_planet = args.z_planet if args.z_planet is not None else args.z2

    overrides = {
        "pressure_angle": args.alpha,
        "helix_angle": args.beta,
        "hand": args.hand,
        "backlash": args.backlash,
        "n_planets": args.planets,
    }
    if args.face_width is not None:
        overrides["face_width"] = args.face_width
    if args.bore is not None:
        overrides["bore"] = args.bore
    if args.hub is not None:
        overrides["hub_thickness"] = args.hub
    if args.rim is not None:
        overrides["rim_thickness"] = args.rim
    return PlanetarySetParams.with_defaults(args.module, z_sun, z_planet, **overrides)


def _member_row(label, geo, get, unit=""):
    values = [get(geo.member(name)) for name in MEMBERS]
    return f"  {label:<28}" + "".join(f"{v:>14}" for v in values) + f"  {unit}"


def print_report(geo) -> None:
    p = geo.params
    f = lambda v: f"{v:.4f}"  # noqa: E731

    print("INPUT")
    print(_row("normal module", f(p.module), "", "mm"))
    print(_row("sun teeth", p.z_sun))
    print(_row("planet teeth", p.z_planet))
    print(_row("ring teeth (derived)", p.z_ring))
    print(_row("planets", p.n_planets))
    print(_row("normal pressure angle", f(p.pressure_angle), "", "deg"))
    print(_row("helix angle", f(p.helix_angle), "", "deg"))
    print(_row("hand (sun)", p.hand))
    print(_row("face width", f(p.face_width), "", "mm"))
    print(_row("sun bore", f(p.bore), "", "mm"))
    print(_row("hub thickness", f(p.hub_thickness), "", "mm"))
    print(_row("ring rim thickness", f(p.rim_thickness), "", "mm"))
    print(_row("backlash", f(p.backlash), "", "mm"))

    print("\nTRAIN")
    print(_row("centre distance a", f(geo.centre_distance), "", "mm"))
    print(_row("transverse module m_t", f(geo.transverse_module), "", "mm"))
    print(
        _row(
            "transverse pressure angle",
            f(math.degrees(geo.transverse_pressure_angle)),
            "",
            "deg",
        )
    )
    print(_row("circular pitch", f(geo.circular_pitch), "", "mm"))
    print(_row("whole depth", f(geo.whole_depth), "", "mm"))
    print(_row("assembly condition", f"{p.z_sun} + {p.z_ring} = {p.z_sun + p.z_ring}"))
    print(
        _row(
            "  divisible by planets",
            "yes" if p.assembly_remainder == 0 else
            f"NO (remainder {p.assembly_remainder})",
        )
    )
    print(_row("planet orbit radius", f(geo.centre_distance), "", "mm"))
    print(_row("neighbour spacing", f(geo.neighbour_spacing), "", "mm"))
    print(
        _row(
            "  gap between planet tips",
            f(geo.neighbour_spacing - 2.0 * geo.planet.tip_r),
            "",
            "mm",
        )
    )
    print(_row("contact ratio, sun-planet", f(geo.sun_planet_contact_ratio)))
    print(_row("contact ratio, planet-ring", f(geo.planet_ring_contact_ratio)))

    print("\nRATIOS")
    print(_row("ring held: sun / carrier", f(p.ratio_carrier_to_sun)))
    print(_row("carrier held: ring / sun", f(p.ratio_ring_to_sun)))

    print(f"\nMEMBERS{'':<21}{'SUN':>14}{'PLANET':>14}{'RING':>14}")
    print(_member_row("teeth", geo, lambda m: m.z))
    print(_member_row("hand", geo, lambda m: m.hand))
    print(_member_row("arrangement", geo,
                      lambda m: "internal" if m.internal else "external"))
    print(_member_row("pitch diameter", geo, lambda m: f(2.0 * m.pitch_r), "mm"))
    print(_member_row("base radius", geo, lambda m: f(m.base_r), "mm"))
    print(_member_row("tip radius", geo, lambda m: f(m.tip_r), "mm"))
    print(_member_row("root radius", geo, lambda m: f(m.root_r), "mm"))
    print(_member_row("twist over the face", geo, lambda m: f(m.twist_deg), "deg"))
    # Only the ring has one, so the sun and planet columns stay blank rather
    # than carrying a zero that would read as a real measurement.
    print(_member_row(
        "rim radius", geo,
        lambda m: f(geo.ring_rim_radius) if m.internal else "", "mm",
    ))

    print("\nPLACEMENT")
    for k in range(p.n_planets):
        x, y, _ = mesh.planet_translation(geo, k)
        print(
            f"  planet {k}: carrier {math.degrees(mesh.carrier_angle(geo, k)):8.3f} deg"
            f"   at ({x:9.4f}, {y:9.4f})"
            f"   clocked {math.degrees(mesh.planet_clocking(geo, k)) % 360.0:9.4f} deg"
        )
    print(
        f"  ring    : clocked "
        f"{math.degrees(mesh.ring_clocking(geo, 0)) % 360.0:9.4f} deg"
    )
    print("  sun     : keeps the frame it was built in")


def report(args) -> tuple[object, object, list]:
    """Print the whole report; return (geo, section, loop_3d) for the CSV dump."""
    p = params_from_args(args)
    if not emit_issues(validate(p)):
        return None, None, []

    geo = compute_set(p)
    print_report(geo)

    # `--member` defaults to "pinion", which is a name this train has no
    # member by. A planetary set maps the pair-shaped defaults onto its own
    # rather than erroring, so `--type planetary` with no --member works.
    member = {"pinion": "sun", "gear": "planet"}.get(args.member, args.member)
    pair = geo.mesh_for(member)
    role = geo.role_of(member)

    from ..spur.geometry import tooth_space_section

    section = tooth_space_section(pair, role, z=0.0)
    loop3d = section.loop_3d()
    f = lambda v: f"{v:.4f}"  # noqa: E731
    print(f"\nTOOTH SPACE SECTION  ({member}, transverse, z = 0)")
    print(_row("boundary points", len(section.loop_2d)))
    print(_row("root fillet fitted", "yes" if section.filleted else "no (sharp)"))
    print(_row("root radius", f(section.r_root), "", "mm"))
    print(_row("tip radius", f(section.r_tip), "", "mm"))
    print(_row("cut cap radius", f(section.r_cap), "", "mm"))

    print(f"\nBLANK OUTLINE  ({member}, meridian R/z, front face at z = 0)")
    for R, z in blank_outline(pair, role):
        print(f"    R={R:9.4f}   z={z:9.4f}")

    return geo, section, loop3d
