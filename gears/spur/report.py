"""The terminal report for a spur set. No SOLIDWORKS, no GUI.

Same three-column shape as the bevel report, so the two line up side by side.
The rows differ because the quantities do: no cone angles or cone distances, and
in their place the transverse conversions, the centre distance and the two
contact ratios.
"""

from __future__ import annotations

import math

from ..report_format import emit_issues, row as _row
from .geometry import (
    blank_outline,
    compute_set,
    end_overshoot,
    rim_radius,
    tooth_space_section,
)
from .params import SpurSetParams
from .validate import validate

FLAGS = (
    "alpha", "beta", "face_width", "bore", "hub", "member", "backlash",
    "internal", "rim",
)


def add_arguments(ap) -> None:
    ap.add_argument(
        "--beta", type=float, default=0.0, help="helix angle, deg (0 = straight)"
    )
    ap.add_argument("--backlash", type=float, default=0.0, help="circular backlash, mm")
    ap.add_argument(
        "--internal",
        action="store_true",
        help="z2 is an internal ring gear rather than an external one",
    )
    ap.add_argument(
        "--rim", type=float, help="ring rim thickness outside its root circle, mm"
    )


def params_from_args(args) -> SpurSetParams:
    overrides = {
        "pressure_angle": args.alpha,
        "helix_angle": args.beta,
        "hand": args.hand,
        "backlash": args.backlash,
        "internal": args.internal,
    }
    if args.face_width is not None:
        overrides["face_width"] = args.face_width
    if args.bore is not None:
        overrides["bore"] = args.bore
    if args.hub is not None:
        overrides["hub_thickness"] = args.hub
    if args.rim is not None:
        overrides["rim_thickness"] = args.rim
    return SpurSetParams.with_defaults(args.module, args.z1, args.z2, **overrides)


def print_report(geo) -> None:
    p = geo.params
    f = lambda v: f"{v:.4f}"  # noqa: E731

    print("INPUT")
    print(_row("arrangement", "internal" if p.internal else "external"))
    print(_row("normal module", f(p.module), "", "mm"))
    print(_row("teeth", p.z1, p.z2))
    print(_row("normal pressure angle", f(p.pressure_angle), "", "deg"))
    print(_row("helix angle", f(p.helix_angle), "", "deg"))
    print(_row("hand (pinion)", p.hand))
    print(_row("face width", f(p.face_width), "", "mm"))
    print(_row("bore", f(p.bore), "", "mm"))
    print(_row("hub thickness", f(p.hub_thickness), "", "mm"))
    if p.internal:
        print(_row("ring rim thickness", f(p.rim_thickness), "", "mm"))
    print(_row("backlash", f(p.backlash), "", "mm"))

    print("\nSET")
    print(_row("ratio", f(p.ratio)))
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
    print(_row("circular pitch (transverse)", f(geo.circular_pitch), "", "mm"))
    axial = "inf" if math.isinf(geo.axial_pitch) else f(geo.axial_pitch)
    print(_row("axial pitch", axial, "", "mm"))
    print(_row("whole depth", f(geo.whole_depth), "", "mm"))
    print(_row("contact ratio, transverse", f(geo.transverse_contact_ratio)))
    print(_row("contact ratio, axial", f(geo.axial_contact_ratio)))
    print(_row("contact ratio, total", f(geo.total_contact_ratio)))

    second = "RING" if p.internal else "GEAR"
    print(f"\nMEMBERS{'':<21}{'PINION':>14}{second:>14}")
    a, b = geo.pinion, geo.gear
    print(_row("teeth", a.z, b.z))
    print(_row("hand", a.hand, b.hand))
    print(_row("pitch diameter", f(2.0 * a.pitch_r), f(2.0 * b.pitch_r), "mm"))
    print(_row("outside diameter", f(a.outside_dia), f(b.outside_dia), "mm"))
    print(_row("base radius", f(a.base_r), f(b.base_r), "mm"))
    # For the ring these two are the other way round - tip innermost, root
    # outermost - which is why the report names them rather than ordering them.
    print(_row("tip radius", f(a.tip_r), f(b.tip_r), "mm"))
    print(_row("root radius", f(a.root_r), f(b.root_r), "mm"))
    if p.internal:
        print(_row("rim radius", "", f(rim_radius(geo, "gear")), "mm"))
    print(_row("addendum", f(a.addendum), f(b.addendum), "mm"))
    print(_row("dedendum", f(a.dedendum), f(b.dedendum), "mm"))
    print(_row("virtual teeth z_v", f(a.virtual_teeth), f(b.virtual_teeth)))
    print(_row("twist over the face", f(a.twist_deg), f(b.twist_deg), "deg"))


def report(args) -> tuple[object, object, list]:
    """Print the whole report; return (geo, section, loop_3d) for the CSV dump."""
    p = params_from_args(args)
    if not emit_issues(validate(p)):
        return None, None, []

    geo = compute_set(p)
    print_report(geo)

    f = lambda v: f"{v:.4f}"  # noqa: E731
    section = tooth_space_section(geo, args.member, z=0.0)
    loop3d = section.loop_3d()
    print(f"\nTOOTH SPACE SECTION  ({args.member}, transverse, z = 0)")
    print(_row("boundary points", len(section.loop_2d)))
    print(_row("root fillet fitted", "yes" if section.filleted else "no (sharp)"))
    print(_row("root radius", f(section.r_root), "", "mm"))
    print(_row("tip radius", f(section.r_tip), "", "mm"))
    print(_row("cut cap radius", f(section.r_cap), "", "mm"))
    ov = end_overshoot(geo)
    print(_row("cut z span", f"{-ov:.4f} .. {geo.params.face_width + ov:.4f}", "", "mm"))

    print(f"\nBLANK OUTLINE  ({args.member}, meridian R/z, front face at z = 0)")
    for R, z in blank_outline(geo, args.member):
        print(f"    R={R:9.4f}   z={z:9.4f}")

    return geo, section, loop3d
