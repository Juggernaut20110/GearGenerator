"""Build one bevel gear, straight or spiral, as a SOLIDWORKS part.

Sequence, matching the plan:

1. reference axis along Z (intersection of the Top and Right planes)
2. blank: meridian outline sketched on the Top Plane, fully dimensioned with
   driving dimensions, revolved 360 degrees
3. 3D sketches of the tooth-space section, two for a straight gear and as many
   as the trace needs for a spiral one
4. loft cut through them, with a guide curve when the teeth are spiral
5. circular pattern of that cut, z instances about the axis

Coordinates come out of `geometry` in millimetres with the pitch apex at the
origin and the gear axis along +Z; `mm()` converts at every API boundary.

**The blank does not change between the two.** A spiral changes where along the
face each section sits about the axis, not the meridian outline it is cut out
of, so the whole dimension plan below - every driving dimension, every global
variable, the derived `BackFaceToApex` - is untouched by this feature and the
straight gear it was written for still comes out byte for byte the same.
"""

from __future__ import annotations

import math

from ..bevel.geometry import (
    SetGeometry,
    blank_outline,
    guide_spiral,
    section_cone_distances,
    to_cone_3d,
    tooth_space_section,
)
from .common import (
    AXIS_FEATURE_NAME,
    BlankDimension,
    BlankVariable,
    BuildResult,
    axial_position,
    axis_datum,
    close_and_revolve_blank,
    constrain_blank,
    create_axis,
    draw_curves_3d,
    drop_offcut,
    loft_cut,
    measure_part,
    pattern_teeth,
    radial_position,
    sketch_blank_outline,
)
from .session import MARK_LOFT_GUIDE, DimensionFlags, add_dimension, mm

__all__ = ["AXIS_FEATURE_NAME", "BuildResult", "build_gear"]


def _section_curves(section) -> list[tuple[str, list]]:
    """Break a tooth-space section into 3D curves to draw, in loop order.

    The fillet is merged into its flank so a single spline covers both: they
    are tangent by construction, and splitting them would drop a spline
    endpoint in the middle of a smooth run. The corners that are genuinely
    sharp - flank to riser, riser to cap - stay separate so they stay sharp.

    The cap arrives already split in two when the section was built with
    `split_cap`, and the two halves stay separate here so the vertex between
    them survives into the sketch. That vertex is what the guide curve passes
    through on a spiral gear, and a guide that only passes *near* a spline is
    the classic way a guided loft fails.
    """
    seg = section.segments

    def join(first: list, second: list) -> list:
        if not first:
            return second
        if not second:
            return first
        return first + second[1:]  # drop the shared junction point

    cap = (
        [("cap_neg", seg["cap_neg"]), ("cap_pos", seg["cap_pos"])]
        if "cap_neg" in seg
        else [("cap", seg["cap"])]
    )
    ordered = [
        ("flank_neg", join(seg["fillet_neg"], seg["flank_neg"])),
        ("riser_neg", seg["riser_neg"]),
        *cap,
        ("riser_pos", seg["riser_pos"]),
        ("flank_pos", join(seg["flank_pos"], seg["fillet_pos"])),
        ("root", seg["root"]),
    ]
    return [
        (
            name,
            [
                to_cone_3d(
                    x, y, section.pitch_angle, section.cone_apex_z, section.phase
                )
                for x, y in pts
            ],
        )
        for name, pts in ordered
    ]


def _draw_section(model, geo: SetGeometry, member: str, cone_dist: float, split_cap: bool):
    """Draw one tooth-space section as a 3D sketch; return the sketch feature.

    `cone_dist` is already pushed past the end of the face width at the two ends
    by `end_overshoot`, so the loft cut starts and finishes clear of the blank
    rather than tangent to one of its faces.
    """
    section = tooth_space_section(
        geo, member, cone_dist=cone_dist, split_cap=split_cap
    )
    return draw_curves_3d(
        model, _section_curves(section), f"section at A={cone_dist:.4f}"
    )


def _draw_guide(model, geo: SetGeometry, member: str, a_hi: float, a_lo: float):
    """Draw the curve the loft is guided along, as a single 3D spline.

    Sampled rather than built from any feature SOLIDWORKS offers, for the same
    reason the spur builder samples its helix: this builder already knows how to
    write a 3D sketch through points, and a sampled curve needs no agreement
    with SOLIDWORKS about pitch, start angle or hand.

    It is a conical spiral rather than a helix - the cap radius shrinks toward
    the toe as the section scales - so there is no feature that would have built
    it anyway.
    """
    points = guide_spiral(geo, member, a_hi, a_lo)
    return draw_curves_3d(model, [("guide spiral", points)], "loft guide")


def _angle_position(a, b) -> tuple[float, float, float]:
    """Where to place the angular dimension between a cone segment and the axis.

    Placement is not cosmetic for an angular dimension: it is what picks which
    of the four angles around the intersection SOLIDWORKS creates. Extend the
    cone to where it crosses the axis, then sit the text on the bisector of the
    axis and the cone, on the side the gear is actually on. `add_dimension`
    re-reads the value, so a placement that lands in the wrong quadrant comes
    back as the supplement and fails loudly rather than quietly.
    """
    (r1, z1), (r2, z2) = a, b
    z0 = z1 - r1 * (z2 - z1) / (r2 - r1)      # cone apex, on the axis
    d1 = math.hypot(r1, z1 - z0)
    d2 = math.hypot(r2, z2 - z0)

    far = a if d1 >= d2 else b
    ur, uz = far[0] / max(d1, d2), (far[1] - z0) / max(d1, d2)
    # Both endpoints lie on the same ray, so the axis leg runs the same way in z.
    br, bz = ur, uz + (1.0 if uz >= 0.0 else -1.0)
    norm = math.hypot(br, bz) or 1.0
    reach = 0.5 * min(d1, d2)
    return (mm(reach * br / norm), 0.0, mm(z0 + reach * bz / norm))


def _dimension_blank(app, model, geo: SetGeometry, member: str, axis, lines, outline):
    """Drive the blank profile the way a bevel gear drawing dimensions it.

    The two cones carry angles and everything else carries a linear dimension.
    That is not just presentation: P1's radius is left undimensioned because the
    face cone angle pins it, and the crown's axial position is left
    undimensioned because the back cone angle pins it, which is exactly how the
    two cones define the blank.

    The counts have to be exact or the dimensions cannot all be driving. Without
    a hub or a root rim the sketch has seven points - five profile vertices and
    the two ends of the centreline - so fourteen degrees of freedom; fixing both
    centreline ends removes four, and three relations (flat front, flat back,
    bore) remove three, leaving the seven that the seven dimensions below take
    up. A hub adds two points, two relations and two dimensions; a root rim adds
    one point, one relation (the rim is a cylinder, so vertical) and one
    dimension. All four combinations balance, which is what
    `require_fully_defined` then confirms against the solver.

    Values come from `outline` rather than being recomputed, so a dimension
    cannot disagree with the geometry it is measuring - except the two angles,
    which are the nominal cone angles and are checked against the drawn
    segments by `add_dimension`.
    """
    p = geo.params
    m = geo.member(member)
    apex = axis_datum(axis)

    # Line i runs from outline[i] to outline[i+1], so the root rim - when there
    # is one - displaces the flat back and everything behind it by one.
    has_rim = p.min_root_thickness > 0.0
    i_back = 4 if has_rim else 3

    r_bore, z_front = outline[0]
    tip_r, z_crown = outline[2]
    root_r, z_root = outline[3]
    z_back = outline[i_back][1]

    front_face, face_cone, back_cone = lines[0], lines[1], lines[2]
    flat_back = lines[i_back]
    bore = lines[-1]

    plan = [
        ((axis, bore), radial_position(r_bore, 0.5 * (z_front + z_back)),
         r_bore, "mm", "bore radius", "BoreRadius", None),
        ((apex, front_face), axial_position(tip_r, 0.5 * z_front),
         z_front, "mm", "front face to apex", "FrontFaceToApex", None),
    ]

    if has_rim:
        # Measured from the root point to the flat back rather than along the
        # rim line itself: point-to-line is the same pattern as every other
        # axial dimension here, and a line-length dimension is not.
        plan.append(
            ((back_cone.GetEndPoint2(), flat_back),
             # Further out than the other axial dimensions so the two that share
             # this end of the blank do not sit on top of each other.
             axial_position(1.5 * tip_r, 0.5 * (z_root + z_back)),
             z_back - z_root, "mm", "minimum root thickness", "MinRootThickness",
             None)
        )

    plan.append(
        ((apex, flat_back), axial_position(tip_r, 0.5 * z_back),
         z_back, "mm", "back face to apex", "BackFaceToApex",
         '"OuterRootToApex" + "MinRootThickness"' if has_rim else None)
    )
    plan.append(
        ((axis, back_cone.GetStartPoint2()), radial_position(tip_r, z_crown),
         tip_r, "mm", "crown radius", "CrownRadius", None)
    )
    plan.append(
        ((axis, back_cone.GetEndPoint2()), radial_position(root_r, z_back),
         root_r, "mm", "outer root radius", "OuterRootRadius", None)
    )

    if p.hub_thickness > 0.0:
        hub_r = outline[i_back + 1][0]
        z_hub_back = outline[i_back + 2][1]
        plan.append(
            ((axis, lines[i_back + 1]),
             radial_position(hub_r, 0.5 * (z_back + z_hub_back)),
             hub_r, "mm", "hub radius", "HubRadius", None)
        )
        plan.append(
            ((apex, lines[i_back + 2]), axial_position(tip_r, 0.5 * z_hub_back),
             z_hub_back, "mm", "hub back to apex", "HubBackToApex", None)
        )

    # The back cone is perpendicular to the pitch cone, so its angle to the axis
    # is 90 degrees minus the pitch angle - not the root angle, which belongs to
    # a different cone and is 20 degrees away from this one.
    plan.append(
        ((face_cone, axis), _angle_position(outline[1], outline[2]),
         math.degrees(m.face_angle), "deg", "face cone angle", "FaceConeAngle", None)
    )
    plan.append(
        ((back_cone, axis), _angle_position(outline[2], outline[3]),
         math.degrees(math.pi / 2.0 - m.pitch_angle), "deg",
         "back cone angle", "BackConeAngle", None)
    )

    created = []
    with DimensionFlags(app):
        for entities, position, value, unit, what, name, defn in plan:
            si = mm(value) if unit == "mm" else math.radians(value)
            add_dimension(model, entities, position, si, what, name)
            created.append(BlankDimension(name, value, unit, si, what, defn))

    variables = []
    if has_rim:
        variables.append(
            BlankVariable(
                "OuterRootToApex", f"{z_root:.9g}mm", "outer root point to apex"
            )
        )
    return created, variables


def build_blank(app, model, geo: SetGeometry, member: str):
    """Sketch the meridian outline on the Top Plane, dimension it, and revolve it."""
    outline = blank_outline(geo, member)
    mgr, axis, lines = sketch_blank_outline(model, outline)

    # Relations and dimensions go on with AddToDB back off and the sketch still
    # open; both are edits to the active sketch, not to a closed one.
    constrain_blank(model, axis, lines, outline)
    dims, variables = _dimension_blank(app, model, geo, member, axis, lines, outline)
    return close_and_revolve_blank(model, mgr, dims, variables)


def build_gear(
    session,
    geo: SetGeometry,
    member: str,
    save_path: str | None = None,
) -> BuildResult:
    """Build one member of the set as a new part document."""
    m = geo.member(member)
    model = session.new_part()

    axis_feat = create_axis(model)
    build_blank(session.app, model, geo, member)

    # A straight bevel gear takes two sections, one at each end of the face
    # width: both are pure conical surfaces through the pitch apex, so the ruled
    # surface the loft puts between them is the tooth flank exactly, and no
    # guide curve is needed or wanted.
    #
    # A spiral one takes as many as `section_cone_distances` asks for - 11 per
    # member on the anchor set - because consecutive sections are rotated
    # relative to each other and a loft chords straight between them while the
    # tooth trace is an arc. The guide pins the cap exactly and leaves the rest
    # interpolated, which is why it is not a substitute for the extra sections;
    # the spur builder paid for that lesson and the note beside
    # MAX_SECTION_SAGITTA_MM records what it cost.
    spiral = geo.trace is not None
    distances = section_cone_distances(geo, member)

    sections = [
        _draw_section(model, geo, member, A, split_cap=spiral) for A in distances
    ]
    guides = (
        (_draw_guide(model, geo, member, distances[0], distances[-1]),)
        if spiral
        else ()
    )

    cut = loft_cut(model, sections, guides, guide_mark=MARK_LOFT_GUIDE)
    drop_offcut(model)
    pattern_teeth(model, cut, axis_feat, m.z)

    return measure_part(
        session, model, member, m.z, m.outside_dia / 2.0, save_path
    )
