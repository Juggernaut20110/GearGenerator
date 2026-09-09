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
    "internal", "rim", "x1", "x2",
)


def add_arguments(ap) -> None:
    ap.add_argument(
        "--beta", type=float, default=0.0, help="helix angle, deg (0 = straight)"
    )
    ap.add_argument(
        "--backlash", type=float, default=0.0,
        help="backlash allowance, mm (hypoid: outer transverse at wheel outer cone)",
    )
    ap.add_argument(
        "--x1", type=float, default=0.0,
        help="pinion profile-shift coefficient x1 (normal-module units)",
    )
    ap.add_argument(
        "--x2", type=float, default=0.0,
        help="gear/ring profile-shift coefficient x2 (normal-module units)",
    )
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
        "profile_shift_1": args.x1,
        "profile_shift_2": args.x2,
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
    print(_row("normal module m_n", f(p.module), "", "mm"))
    print(_row("teeth", p.z1, p.z2))
    print(_row("normal pressure angle alpha_n", f(p.pressure_angle), "", "deg"))
    print(_row("profile shift x1", f(p.profile_shift_1)))
    print(_row("profile shift x2", f(p.profile_shift_2)))
    print(_row("helix angle beta", f(p.helix_angle), "", "deg"))
    print(_row("hand (pinion)", p.hand))
    print(_row("face width", f(p.face_width), "", "mm"))
    print(_row("bore", f(p.bore), "", "mm"))
    print(_row("hub thickness", f(p.hub_thickness), "", "mm"))
    if p.internal:
        print(_row("ring rim thickness", f(p.rim_thickness), "", "mm"))
    print(_row("backlash", f(p.backlash), "", "mm"))
    print(_row("root geometry", p.root_geometry))

    print("\nREFERENCE GEOMETRY (SET)")
    print(_row("ratio", f(p.ratio)))
    print(_row("transverse module m_t", f(geo.transverse_module), "", "mm"))
    print(
        _row(
            "transverse pressure angle alpha_t",
            f(math.degrees(geo.transverse_pressure_angle)),
            "",
            "deg",
        )
    )
    print(_row("reference centre distance a", f(geo.reference_centre_distance), "", "mm"))
    print(_row("circular pitch (transverse)", f(geo.circular_pitch), "", "mm"))
    print(_row("rack addendum coefficient h_aP*", f(p.h_aP_star)))
    print(_row("rack clearance coefficient c_P*", f(p.c_P_star)))
    print(_row("rack root-radius coefficient rho_fP*", f(p.rho_fP_star)))

    print("\nWORKING / OPERATING GEOMETRY")
    print(_row("working centre distance a_w", f(geo.working_centre_distance), "", "mm"))
    print(_row("centre-distance modification y", f(geo.centre_distance_modification), "", "m_n"))
    print(
        _row(
            "working pressure angle alpha_wt",
            f(math.degrees(geo.working_pressure_angle)),
            "",
            "deg",
        )
    )
    print(_row("epsilon_alpha (transverse)", f(geo.transverse_contact_ratio)))
    print(_row("epsilon_beta (overlap)", f(geo.overlap_ratio)))
    print(_row("epsilon_gamma (total)", f(geo.total_contact_ratio)))
    axial = "inf" if math.isinf(geo.axial_pitch) else f(geo.axial_pitch)
    print(_row("axial pitch", axial, "", "mm"))
    print(_row("whole depth", f(geo.whole_depth), "", "mm"))

    second = "RING" if p.internal else "GEAR"
    print(f"\nMEMBER GEOMETRY (MEMBERS){'':<12}{'PINION':>14}{second:>14}")
    a, b = geo.pinion, geo.gear
    print(_row("teeth", a.z, b.z))
    print(_row("hand", a.hand, b.hand))
    print(_row("profile shift x", f(a.profile_shift), f(b.profile_shift)))
    print(_row("reference diameter d", f(a.reference_d), f(b.reference_d), "mm"))
    print(_row("base diameter d_b", f(a.base_d), f(b.base_d), "mm"))
    print(_row("working pitch diameter d_w", f(a.working_d), f(b.working_d), "mm"))
    print(_row("tip diameter d_a", f(a.tip_d), f(b.tip_d), "mm"))
    print(_row("root diameter d_f", f(a.root_d), f(b.root_d), "mm"))
    print(
        _row(
            "reference tooth thickness s_t",
            f(a.reference_tooth_thickness),
            f(b.reference_tooth_thickness),
            "mm",
        )
    )
    print(
        _row(
            "normal tooth thickness s_n",
            f(a.normal_tooth_thickness),
            f(b.normal_tooth_thickness),
            "mm",
        )
    )
    print(_row("outside diameter", f(a.outside_dia), f(b.outside_dia), "mm"))
    # For the ring these two are the other way round - tip innermost, root
    # outermost - which is why the report names them rather than ordering them.
    print(_row("tip radius", f(a.tip_r), f(b.tip_r), "mm"))
    print(_row("root radius", f(a.root_r), f(b.root_r), "mm"))
    print(
        _row(
            "generated root diameter d_fE",
            _optional(a.generated_root_d),
            _optional(b.generated_root_d),
            "mm",
        )
    )
    if p.internal:
        print(_row("rim radius", "", f(rim_radius(geo, "gear")), "mm"))
    print(_row("addendum", f(a.addendum), f(b.addendum), "mm"))
    print(_row("dedendum", f(a.dedendum), f(b.dedendum), "mm"))
    print(_row("virtual teeth z_v", f(a.virtual_teeth), f(b.virtual_teeth)))
    print(_row("twist over the face", f(a.twist_deg), f(b.twist_deg), "deg"))


def _optional(value):
    """Format an optional derived quantity without inventing a zero value."""
    return "not available" if value is None else f"{value:.4f}"


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
    root_form = (
        "rack-generated envelope" if section.rack_generated
        else "legacy circular fillet" if section.filleted
        else "sharp root"
    )
    print(_row("root form", root_form))
    print(_row("root radius", f(section.r_root), "", "mm"))
    print(_row("tip radius", f(section.r_tip), "", "mm"))
    print(_row("cut cap radius", f(section.r_cap), "", "mm"))
    ov = end_overshoot(geo)
    print(_row("cut z span", f"{-ov:.4f} .. {geo.params.face_width + ov:.4f}", "", "mm"))

    print(f"\nBLANK OUTLINE  ({args.member}, meridian R/z, front face at z = 0)")
    for R, z in blank_outline(geo, args.member):
        print(f"    R={R:9.4f}   z={z:9.4f}")

    return geo, section, loop3d
