"""Build one involute spur gear, straight or helical, as a SOLIDWORKS part.

Sequence, matching the bevel builder's:

1. reference axis along Z (intersection of the Top and Right planes)
2. blank: meridian outline sketched on the Top Plane, fully dimensioned with
   driving dimensions, revolved 360 degrees
3. 3D sketches of the tooth-space section, two for straight teeth and as many
   as the twist needs for helical ones
4. loft cut through them, with a helix guide curve when the teeth are helical
5. circular pattern of that cut, z instances about the axis

Coordinates come out of `geometry` in millimetres with the front face at the
origin and the gear axis along +Z; `mm()` converts at every API boundary.

Where this differs from the bevel builder
-----------------------------------------
The blank is a stepped cylinder rather than a pair of cones, so every dimension
is linear and none is angular - which means this builder needs neither
`_angle_position` nor the quadrant argument behind it.

Both sections are the same shape. On a bevel gear they are not: each is a
different size, scaled about the pitch apex. Here only the phase changes.

**A straight gear takes two sections; a helical one takes as many as it
needs.** Two identical sections loft to a prism, which is the tooth. Two
*rotated* sections loft to a surface that meets the helicoid only at the ends
and falls inside it in between, because a loft carries each profile point along
a straight chord and a helix is an arc.

A guide curve alone does not fix that, which is worth stating because it is what
this builder was written to do first. Measured on the anchor helical pinion,
built from two sections plus a guide: the mid-face profile stood 0.198 mm proud
of the end profile at the same relative angle, and the cut came out 0.9 percent
short of the volume a true helicoid removes. The guide pins the one point it
runs through - the tip agreed exactly - and leaves the rest interpolated.

So the section count comes from `section_heights`, which solves the chord's
sagitta against a stated tolerance. The guide stays as well: it costs one sketch
and it pins the cap exactly.
"""

from __future__ import annotations

from ..spur.geometry import (
    SpurSetGeometry,
    blank_outline,
    guide_helix,
    rim_radius,
    section_heights,
    to_axial_3d,
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
from .session import (
    MARK_LOFT_GUIDE,
    REL_COINCIDENT,
    DimensionFlags,
    add_dimension,
    add_relation,
    mm,
)

__all__ = ["AXIS_FEATURE_NAME", "BuildResult", "build_spur"]

# Below this the teeth are treated as straight: no twist to force, so no guide.
# A nanoradian over a face width is a few picometres of drift at the tip.
STRAIGHT_TWIST_TOL = 1e-9


def _section_curves(section) -> list[tuple[str, list]]:
    """Break a tooth-space section into 3D curves to draw, in loop order.

    The fillet is merged into its flank so a single spline covers both: they
    are tangent by construction, and splitting them would drop a spline
    endpoint in the middle of a smooth run. The corners that are genuinely
    sharp - flank to riser, riser to cap - stay separate so they stay sharp.

    The cap arrives already split in two when the section was built with
    `split_cap`, and the two halves stay separate here so the vertex between
    them survives into the sketch. That vertex is what the guide curve passes
    through, and a guide that only passes *near* a spline is the classic way a
    guided loft fails.
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
        (name, [to_axial_3d(x, y, section.phase, section.z) for x, y in pts])
        for name, pts in ordered
    ]


def _draw_section(model, geo: SpurSetGeometry, member: str, z: float, split_cap: bool):
    """Draw one tooth-space section as a 3D sketch; return the sketch feature.

    `z` is already pushed past the end face by `end_overshoot`, so the loft cut
    starts and finishes clear of the blank rather than tangent to one of its
    faces - the same zero-thickness rejection the bevel builder works around.
    """
    section = tooth_space_section(geo, member, z=z, split_cap=split_cap)
    return draw_curves_3d(model, _section_curves(section), f"section at z={z:.4f}")


def _draw_guide(model, geo: SpurSetGeometry, member: str, z_lo: float, z_hi: float):
    """Draw the helix the loft is guided along, as a single 3D spline.

    Sampled rather than handed to `InsertHelix`, for the same reason the sections
    are sampled: this builder already knows how to write a 3D sketch through
    points, and a sampled curve needs no agreement with SOLIDWORKS about pitch,
    start angle or hand - three chances to be off by a sign on a part where a
    sign error is a gear that will not mesh.
    """
    points = guide_helix(geo, member, z_lo, z_hi)
    return draw_curves_3d(model, [("guide helix", points)], "loft guide")


def _dimension_blank(app, model, geo: SpurSetGeometry, member: str, axis, lines, outline):
    """Drive the blank profile the way a spur gear drawing dimensions it.

    Every dimension is linear. A spur blank is a stepped cylinder, so there is no
    cone angle to carry and none of the angular-placement trouble that comes with
    one.

    The counts have to be exact or the dimensions cannot all be driving. Without
    a hub the sketch has six points - four profile vertices and the two ends of
    the centreline - so twelve degrees of freedom. Fixing both centreline ends
    removes four; the four alternating horizontal/vertical relations remove four
    more; and the relation tying the front face to the axis datum removes one.
    That leaves three, which the three dimensions below take up. A hub adds two
    points, two relations and two dimensions, so it balances too.
    `require_fully_defined` then confirms the arithmetic against the solver.

    **The front face is held by a relation, not a dimension**, because its
    dimension would measure zero and a zero-length dimension cannot be created.
    That is not a workaround: "the front face sits at z = 0" is a statement about
    the coordinate system rather than a number anyone would want to edit, so a
    relation is what it should have been either way.

    It has to be coincident between the datum **point and the front-face line** -
    point on line. Two other spellings look right and are not, and both fail
    quietly enough to cost an afternoon: a horizontal relation between the datum
    and the line's endpoint does nothing at all, and a coincident relation
    between those same two points *merges* them, dragging the blank's corner onto
    the axis. See the note beside `REL_COINCIDENT` for the measurements.

    Values come from `outline` rather than being recomputed, so a dimension
    cannot disagree with the geometry it is measuring.
    """
    p = geo.params
    m = geo.member(member)
    datum = axis_datum(axis)

    has_hub = len(outline) > 4
    r_inner, _ = outline[0]
    r_outer, _ = outline[1]
    z_back = outline[2][1]

    front_face, outside, back_face = lines[0], lines[1], lines[2]
    inner = lines[-1]

    add_relation(
        model,
        (datum, front_face),
        REL_COINCIDENT,
        "front face through the axis datum",
    )

    # A ring gear's blank is the same four-line profile as a hubless external
    # one - a rectangle in the meridian plane - so the degree-of-freedom count
    # below and every relation holds unchanged. What differs is only what the
    # two radii *are*, and so what they should be called in the equations: the
    # inner one is the ring's tip circle rather than a bore, and the outer one
    # is the rim rather than the tips. Naming them the external way would leave
    # someone editing "BoreRadius" to move a tooth.
    if m.internal:
        inner_what, inner_name = "tip radius", "TipRadius"
        outer_what, outer_name = "rim radius", "RimRadius"
    else:
        inner_what, inner_name = "bore radius", "BoreRadius"
        outer_what, outer_name = "tip radius", "TipRadius"

    plan = [
        ((axis, inner), radial_position(r_inner, 0.5 * z_back),
         r_inner, "mm", inner_what, inner_name, None),
        ((axis, outside), radial_position(r_outer, 0.5 * z_back),
         r_outer, "mm", outer_what, outer_name, None),
        ((datum, back_face), axial_position(r_outer, 0.5 * z_back),
         z_back, "mm", "face width", "FaceWidth", None),
    ]

    variables: list[BlankVariable] = []
    if has_hub:
        # Only an external member ever gets here: a ring gear's outline is four
        # points exactly. It has no hub, and no bore for one to stand around.
        r_hub, _ = outline[3]
        z_hub_back = outline[4][1]
        hub_outside, hub_back = lines[3], lines[4]

        # Same trick as the bevel builder's root rim, for the same reason. The
        # number a person wants to edit is the hub's thickness, but the sketch
        # has to be dimensioned to the hub's back face. So the thickness is a
        # variable of its own and the back face is derived from it, which makes
        # editing HubThickness grow the boss *backward* instead of walking the
        # gear's own back face forward into the teeth.
        variables.append(
            BlankVariable("HubThickness", f"{p.hub_thickness:.9g}mm", "hub thickness")
        )
        plan.append(
            ((axis, hub_outside), radial_position(r_hub, 0.5 * (z_back + z_hub_back)),
             r_hub, "mm", "hub radius", "HubRadius", None)
        )
        plan.append(
            # Further out than the other axial dimensions so the two that share
            # this end of the blank do not sit on top of each other.
            # `r_outer` is the tip radius here and only here: this branch runs
            # for an external member alone, which is what makes the two names
            # the same number. Kept as `r_outer` so the rename that generalised
            # the pair for ring gears stays complete.
            ((datum, hub_back), axial_position(1.5 * r_outer, 0.5 * z_hub_back),
             z_hub_back, "mm", "hub back face", "HubBackFace",
             '"FaceWidth" + "HubThickness"')
        )

    created = []
    with DimensionFlags(app):
        for entities, position, value, unit, what, name, defn in plan:
            si = mm(value)
            add_dimension(model, entities, position, si, what, name)
            created.append(BlankDimension(name, value, unit, si, what, defn))
    return created, variables


def build_blank(app, model, geo: SpurSetGeometry, member: str):
    """Sketch the meridian outline on the Top Plane, dimension it, and revolve it."""
    outline = blank_outline(geo, member)
    mgr, axis, lines = sketch_blank_outline(model, outline)

    # Relations and dimensions go on with AddToDB back off and the sketch still
    # open; both are edits to the active sketch, not to a closed one.
    constrain_blank(model, axis, lines, outline)
    dims, variables = _dimension_blank(app, model, geo, member, axis, lines, outline)
    return close_and_revolve_blank(model, mgr, dims, variables)


def build_spur(
    session,
    geo: SpurSetGeometry,
    member: str,
    save_path: str | None = None,
) -> BuildResult:
    """Build one member of the set as a new part document."""
    m = geo.member(member)
    model = session.new_part()

    axis_feat = create_axis(model)
    build_blank(session.app, model, geo, member)

    helical = abs(m.twist) > STRAIGHT_TWIST_TOL
    heights = section_heights(geo, member)

    sections = [
        _draw_section(model, geo, member, z, split_cap=helical) for z in heights
    ]
    guides = (
        (_draw_guide(model, geo, member, heights[0], heights[-1]),) if helical else ()
    )

    cut = loft_cut(model, sections, guides, guide_mark=MARK_LOFT_GUIDE)
    drop_offcut(model)
    pattern_teeth(model, cut, axis_feat, m.z)

    # A ring's outermost surface is its rim, not its teeth, so that is what the
    # bounding box will measure. `m.tip_r` is its *innermost* radius and would
    # come back as a 16 percent error on the anchor ring.
    expected_radius = rim_radius(geo, member) if m.internal else m.tip_r
    return measure_part(session, model, member, m.z, expected_radius, save_path)
