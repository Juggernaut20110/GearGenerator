"""Build one bevel gear as a SOLIDWORKS part.

Sequence, matching the plan:

1. reference axis along Z (intersection of the Top and Right planes)
2. blank: meridian outline sketched on the Top Plane, fully dimensioned with
   driving dimensions, revolved 360 degrees
3. two 3D sketches - the tooth-space section at each end of the face width
4. loft cut between them
5. circular pattern of that cut, z instances about the axis

Coordinates come out of `geometry` in millimetres with the pitch apex at the
origin and the gear axis along +Z; `mm()` converts at every API boundary.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from ..geometry import (
    SetGeometry,
    blank_outline,
    end_overshoot,
    to_cone_3d,
    tooth_space_section,
)
from .session import (
    MARK_LOFT_PROFILE,
    MARK_PATTERN_AXIS,
    MARK_PATTERN_FEATURE,
    REL_FIXED,
    REL_HORIZONTAL,
    REL_VERTICAL,
    RIGHT_PLANE_NAMES,
    SW_SOLID_BODY,
    TOP_PLANE_NAMES,
    DimensionFlags,
    SketchFlags,
    SwError,
    add_dimension,
    add_equation,
    add_relation,
    equation_manager,
    flag_methods,
    mm,
    points_to_doubles,
    require,
    require_angle_precision,
    require_fully_defined,
    select,
    select_first,
)


# The reference axis is named so `sw.assembly` can mate against it. Kept here
# next to the feature that creates it rather than in session.py, because it is
# a fact about this part, not about the API.
AXIS_FEATURE_NAME = "GearAxis"


@dataclass
class BuildResult:
    """What the build produced, and enough measurements to check it."""

    member: str
    title: str
    teeth: int
    body_count: int
    face_count: int
    box_mm: tuple[float, ...]
    max_radius_mm: float
    expected_radius_mm: float
    path: str | None = None
    model: object = field(default=None, repr=False)

    @property
    def radius_error_pct(self) -> float:
        return (
            100.0
            * (self.max_radius_mm - self.expected_radius_mm)
            / self.expected_radius_mm
        )


def _model_to_sketch(sketch):
    """Return a function mapping model-space metres into sketch-space metres.

    `IMathTransform.ArrayData` is 16 doubles: 9 rotation, 3 translation, then a
    scale factor. The rotation is stored **column-major** - determined
    empirically by placing a circle at sketch (0, +50 mm) on the Top Plane and
    measuring the resulting solid at model z = -50 mm. Reading it row-major
    silently mirrors the part, so this is not a detail to guess at.
    """
    a = list(sketch.ModelToSketchTransform.ArrayData)
    scale = a[12] or 1.0

    def to_sketch(x: float, y: float, z: float) -> tuple[float, float]:
        sx = (a[0] * x + a[3] * y + a[6] * z) * scale + a[9]
        sy = (a[1] * x + a[4] * y + a[7] * z) * scale + a[10]
        return sx, sy

    return to_sketch


def _last_feature(model):
    """The IFeature just created.

    Holding the object beats holding a name: `IFeature.Select2` selects it
    directly, with no name string and no selection-type string to get wrong.
    Names are localised and the type string for a 3D sketch is not the one you
    would guess.
    """
    feat = model.FeatureByPositionReverse(0)
    if feat is None:
        raise SwError("could not read back the feature just created")
    return feat


def _select_feature(feat, what: str, append: bool = False, mark: int = 0) -> None:
    if not feat.Select2(bool(append), int(mark)):
        raise SwError(f"could not select {what} ({feat.Name})")


def _section_curves(section) -> list[tuple[str, list]]:
    """Break a tooth-space section into 3D curves to draw, in loop order.

    The fillet is merged into its flank so a single spline covers both: they
    are tangent by construction, and splitting them would drop a spline
    endpoint in the middle of a smooth run. The corners that are genuinely
    sharp - flank to riser, riser to cap - stay separate so they stay sharp.
    """
    seg = section.segments

    def join(first: list, second: list) -> list:
        if not first:
            return second
        if not second:
            return first
        return first + second[1:]  # drop the shared junction point

    ordered = [
        ("flank_neg", join(seg["fillet_neg"], seg["flank_neg"])),
        ("riser_neg", seg["riser_neg"]),
        ("cap", seg["cap"]),
        ("riser_pos", seg["riser_pos"]),
        ("flank_pos", join(seg["flank_pos"], seg["fillet_pos"])),
        ("root", seg["root"]),
    ]
    return [
        (
            name,
            [to_cone_3d(x, y, section.pitch_angle, section.cone_apex_z) for x, y in pts],
        )
        for name, pts in ordered
    ]


def _draw_section(model, geo: SetGeometry, member: str, end: str):
    """Draw one tooth-space section as a 3D sketch; return the sketch feature.

    The section is pushed past the end of the face width by `end_overshoot`, so
    the loft cut starts and finishes clear of the blank rather than tangent to
    one of its faces.
    """
    mgr = flag_methods(model.SketchManager, "CreateSpline", "CreateSpline2")
    curves = _section_curves(
        tooth_space_section(geo, member, end, overshoot=end_overshoot(geo, member))
    )

    mgr.Insert3DSketch(True)
    with SketchFlags(model):
        for name, pts in curves:
            if len(pts) < 2:
                continue
            if len(pts) == 2:
                (x1, y1, z1), (x2, y2, z2) = pts
                require(
                    mgr.CreateLine(mm(x1), mm(y1), mm(z1), mm(x2), mm(y2), mm(z2)),
                    f"{end} section line {name}",
                )
            else:
                require(
                    mgr.CreateSpline(points_to_doubles(pts)),
                    f"{end} section spline {name} ({len(pts)} points)",
                )
    mgr.Insert3DSketch(True)
    model.ClearSelection2(True)
    return _last_feature(model)


def create_axis(model):
    """Reference axis along Z: the intersection of the Top and Right planes.

    Top Plane is XZ and Right Plane is YZ, so they meet along Z - which is the
    gear axis in the geometry module's coordinate system.

    The axis is renamed because the assembly mates against it by name, and
    "Axis1" is both unrecognisable in the tree and not guaranteed to be the
    number a part picks up. Renaming is safe here: everything in this module
    selects the feature *object*, never the name.
    """
    model.ClearSelection2(True)
    select_first(model, TOP_PLANE_NAMES, "PLANE")
    select_first(model, RIGHT_PLANE_NAMES, "PLANE", append=True)
    # InsertAxis2 lives on IModelDoc2, not IFeatureManager.
    require(model.InsertAxis2(True), "InsertAxis2")
    axis = _last_feature(model)
    try:
        axis.Name = AXIS_FEATURE_NAME
    except Exception:
        pass
    return axis


_COORD_TOL = 1e-9    # mm; outline coordinates that "share" a value share it exactly


def _endpoint_methods(line):
    """Flag a fresh sketch line's endpoint accessors as methods, before use.

    `GetStartPoint2` and `GetEndPoint2` take no arguments and return IDispatch,
    which late binding can resolve either way - and it decides on *first*
    access and caches that decision for the object. Resolved as a property, the
    attribute is already the point, so the `()` in `line.GetStartPoint2()` lands
    on the point instead and fails with "Member not found". Flagging has to
    happen here, at creation, because by the time the first read fails it is
    already too late.
    """
    return flag_methods(line, "GetStartPoint2", "GetEndPoint2")


def _axis_apex(axis):
    """The centreline endpoint sitting at the pitch apex.

    The apex is the sketch origin, so it is whichever endpoint is at (0, 0) in
    sketch space. Taking the nearer of the two rather than assuming
    `CreateCenterLine` kept its arguments in the order they were given.
    """
    a, b = axis.GetStartPoint2(), axis.GetEndPoint2()
    return a if math.hypot(a.X, a.Y) <= math.hypot(b.X, b.Y) else b


def _radial_position(r: float, z: float) -> tuple[float, float, float]:
    """Put a radial dimension halfway between the axis and what it measures."""
    return (mm(0.5 * r), 0.0, mm(z))


def _axial_position(reach: float, z: float) -> tuple[float, float, float]:
    """Put an axial dimension to the left of the axis, where nothing is drawn."""
    return (mm(-0.3 * reach), 0.0, mm(z))


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


@dataclass(frozen=True)
class BlankDimension:
    """One driving dimension on the blank sketch, and its global variable.

    `value` is in the unit the equation is written in - millimetres or degrees,
    whichever a person editing it would expect to type - while `si` is the
    metres or radians the API works in.

    `defn` overrides the right-hand side of the variable's defining equation. It
    is how a dimension can be *derived* from other variables rather than carrying
    a number of its own - see `BlankVariable`.
    """

    name: str
    value: float
    unit: str
    si: float
    what: str
    defn: str | None = None


@dataclass(frozen=True)
class BlankVariable:
    """A global variable with no dimension of its own to drive.

    Only one so far: `OuterRootToApex`. The sketch has to be dimensioned to the
    *back face* - the alternative is an apex-to-point distance, which is a
    point-to-point dimension whose axial-versus-diagonal reading depends on where
    the text is placed, and that is not a thing to gamble a driving dimension on.
    But the number a person wants to edit is the rim, not the back face. So the
    root point's axial position is held here, and the back face is derived from
    it:

        "OuterRootToApex" = 19.9314mm
        "MinRootThickness" = 0.5mm
        "BackFaceToApex"  = "OuterRootToApex" + "MinRootThickness"

    which makes editing MinRootThickness grow the blank *backward*, leaving the
    root point - and so the whole tooth - exactly where the geometry put it.
    Dimensioning both to literals instead would move the root point forward and
    eat into the teeth.
    """

    name: str
    text: str
    what: str


def _constrain_blank(model, axis, lines, outline) -> None:
    """Relations first - the dimensions need something to hold on to.

    No merge pass is needed at the corners. `AddToDB` being on suppresses
    inference against *existing* geometry, but a line started exactly where the
    previous one ended still shares its point: measured on this sketch, five
    lines and a centreline give seven sketch points, not twelve, and
    `a.GetEndPoint2()` and `b.GetStartPoint2()` come back as the same underlying
    object. So the profile arrives already closed, with one vertex per outline
    point, which is what the degree-of-freedom count in `_dimension_blank`
    assumes.

    The centreline is fixed rather than tied to the sketch origin. It is both
    the revolve axis and the datum every axial dimension measures from, and
    pinning it outright avoids having to select the origin point over COM,
    which is the one selection in this API with no good handle to hold.

    It is the two *endpoints* that get fixed, not the line. Fixing a line pins
    where it lies and which way it points but leaves its length free, and that
    one leftover degree of freedom is enough to keep the whole sketch under
    defined however well the profile itself is dimensioned.
    """
    add_relation(
        model,
        (axis.GetStartPoint2(), axis.GetEndPoint2()),
        REL_FIXED,
        "fix the gear axis",
    )

    n = len(lines)
    for i, line in enumerate(lines):
        (r1, z1), (r2, z2) = outline[i], outline[(i + 1) % n]
        if abs(z1 - z2) < _COORD_TOL:
            add_relation(model, (line,), REL_HORIZONTAL, f"blank segment {i} flat")
        elif abs(r1 - r2) < _COORD_TOL:
            add_relation(model, (line,), REL_VERTICAL, f"blank segment {i} cylindrical")


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
    apex = _axis_apex(axis)

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
        ((axis, bore), _radial_position(r_bore, 0.5 * (z_front + z_back)),
         r_bore, "mm", "bore radius", "BoreRadius", None),
        ((apex, front_face), _axial_position(tip_r, 0.5 * z_front),
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
             _axial_position(1.5 * tip_r, 0.5 * (z_root + z_back)),
             z_back - z_root, "mm", "minimum root thickness", "MinRootThickness",
             None)
        )

    plan.append(
        ((apex, flat_back), _axial_position(tip_r, 0.5 * z_back),
         z_back, "mm", "back face to apex", "BackFaceToApex",
         '"OuterRootToApex" + "MinRootThickness"' if has_rim else None)
    )
    plan.append(
        ((axis, back_cone.GetStartPoint2()), _radial_position(tip_r, z_crown),
         tip_r, "mm", "crown radius", "CrownRadius", None)
    )
    plan.append(
        ((axis, back_cone.GetEndPoint2()), _radial_position(root_r, z_back),
         root_r, "mm", "outer root radius", "OuterRootRadius", None)
    )

    if p.hub_thickness > 0.0:
        hub_r = outline[i_back + 1][0]
        z_hub_back = outline[i_back + 2][1]
        plan.append(
            ((axis, lines[i_back + 1]),
             _radial_position(hub_r, 0.5 * (z_back + z_hub_back)),
             hub_r, "mm", "hub radius", "HubRadius", None)
        )
        plan.append(
            ((apex, lines[i_back + 2]), _axial_position(tip_r, 0.5 * z_hub_back),
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


def _link_blank_equations(model, sketch_name: str, dims, variables=()) -> None:
    """Give every blank dimension a named global variable and drive it from that.

    Two equations per dimension: `"CrownRadius" = 19.922mm` defines the
    variable, and `"CrownRadius@Sketch1" = "CrownRadius"` hands the dimension
    over to it. All the variables go in first, because an equation cannot refer
    to a global variable that does not exist yet - which is also why `variables`
    is written ahead of them, and why a dimension whose `defn` names another
    variable has to be planned after the one it names.

    A variable and a dimension may share a name - the dimension is only ever
    referred to with its `@sketch` suffix, so the two never collide.

    Values are written to nine significant figures. These dimensions *drive* the
    profile, so precision here is not cosmetic - a rounded equation moves the
    blank rather than just labelling it differently, which is also why the
    document's angular precision has to be raised first (see
    `require_angle_precision`). Nine figures leaves about a nanometre on a 25 mm
    dimension, far below anything the solver or the check in `add_dimension`
    can see, and it keeps `repr`'s float artefacts - 13.625339999999998 for a
    number that is meant to read 13.62534 - out of a dialog people edit by hand.

    `IDimension.DrivenState` reads back as driven once a dimension is bound to
    an equation. That is the "driven by equation" sense, not a reference
    dimension: the sketch stays fully defined, and editing the variable moves
    the geometry. Measured on the anchor pinion, setting CrownRadius to 22 mm
    walks the crown from (19.9220, 22.5585) to (22.0000, 21.0000) - along the
    back cone, exactly where the 53.13 degree angle puts it.
    """
    require_angle_precision(model)
    mgr = equation_manager(model)
    for v in variables:
        add_equation(mgr, f'"{v.name}" = {v.text}', f"{v.what} variable")
    for d in dims:
        text = d.defn if d.defn else f"{d.value:.9g}{d.unit}"
        add_equation(mgr, f'"{d.name}" = {text}', f"{d.what} variable")
    for d in dims:
        add_equation(
            mgr, f'"{d.name}@{sketch_name}" = "{d.name}"', f"{d.what} link"
        )
    mgr.EvaluateAll()

    for d in dims:
        full = f"{d.name}@{sketch_name}"
        dim = model.Parameter(full)
        if dim is None:
            raise SwError(f"could not read {full} back after linking it")
        actual = float(dim.SystemValue)
        if abs(actual - d.si) > 1e-6:
            raise SwError(
                f"{full} reads {actual:.9g} after its equation was applied but "
                f"the geometry is at {d.si:.9g} (SI units)"
            )


def build_blank(app, model, geo: SetGeometry, member: str):
    """Sketch the meridian outline on the Top Plane, dimension it, and revolve it."""
    fm = model.FeatureManager
    mgr = model.SketchManager
    outline = blank_outline(geo, member)
    z_hi = max(z for _, z in outline)

    model.ClearSelection2(True)
    select_first(model, TOP_PLANE_NAMES, "PLANE")
    mgr.InsertSketch(True)
    to_sketch = _model_to_sketch(mgr.ActiveSketch)

    with SketchFlags(model):
        # A single centreline in the sketch is what FeatureRevolve2 picks up as
        # its axis, which saves selecting one explicitly - so this stays the only
        # construction line here. It starts at the pitch apex rather than short
        # of the front face because that endpoint is also the datum every axial
        # dimension measures from.
        c1 = to_sketch(0.0, 0.0, 0.0)
        c2 = to_sketch(0.0, 0.0, mm(z_hi + 5.0))
        axis = _endpoint_methods(
            require(
                mgr.CreateCenterLine(c1[0], c1[1], 0.0, c2[0], c2[1], 0.0),
                "blank centreline",
            )
        )
        lines = []
        closed = outline + [outline[0]]
        for i, ((r1, za), (r2, zb)) in enumerate(zip(closed, closed[1:])):
            a = to_sketch(mm(r1), 0.0, mm(za))
            b = to_sketch(mm(r2), 0.0, mm(zb))
            lines.append(
                _endpoint_methods(
                    require(
                        mgr.CreateLine(a[0], a[1], 0.0, b[0], b[1], 0.0),
                        f"blank outline segment {i}",
                    )
                )
            )

    # Relations and dimensions go on with AddToDB back off and the sketch still
    # open; both are edits to the active sketch, not to a closed one.
    _constrain_blank(model, axis, lines, outline)
    dims, variables = _dimension_blank(app, model, geo, member, axis, lines, outline)
    require_fully_defined(mgr.ActiveSketch, "blank profile sketch")

    mgr.InsertSketch(True)
    model.ClearSelection2(True)
    blank_sketch = _last_feature(model)

    # Only now is the sketch's own name settled, and an equation needs it to
    # name the dimension it drives.
    _link_blank_equations(model, blank_sketch.Name, dims, variables)

    _select_feature(blank_sketch, "blank profile sketch")
    require(
        fm.FeatureRevolve2(
            True,    # single direction
            True,    # solid
            False,   # thin
            False,   # cut
            False,   # reverse
            False,   # both directions up to same entity
            0, 0,    # blind, blind
            2.0 * math.pi, 0.0,
            False, False,
            0.0, 0.0,
            0,       # thin type
            0.0, 0.0,
            True,    # merge
            True, True,
        ),
        "revolve the blank",
    )
    return _last_feature(model)


def _drop_offcut(model) -> None:
    """Discard the chunk `InsertCutBlend` leaves behind.

    A loft cut here does not delete the material it encloses - it *splits* the
    blank in two and keeps both halves. Measured on the anchor pinion: the
    blank is 11738 mm3, and after the cut the two bodies are 11562 and 176 mm3,
    summing back to exactly 11738. The large body is already the gear with one
    tooth space correctly removed; the small one is the tooth space itself.

    Leaving it in place also blocks the circular pattern, which fails outright
    while the extra body exists. Dropping it here makes the pattern succeed and
    removes every remaining tooth space in one body.
    """
    bodies = list(model.GetBodies2(SW_SOLID_BODY, True) or [])
    if len(bodies) <= 1:
        return

    keep = max(bodies, key=lambda b: b.GetMassProperties(1.0)[3])
    model.ClearSelection2(True)
    # IBody2.Select2 wants a real dispatch or a plain None for Data - the
    # VT_DISPATCH VARIANT used elsewhere is rejected here.
    if not keep.Select2(False, None):
        raise SwError("could not select the gear body to keep")
    require(
        flag_methods(model.FeatureManager, "InsertDeleteBody2").InsertDeleteBody2(
            True  # keep the selected bodies, delete the rest
        ),
        f"discard {len(bodies) - 1} off-cut body/bodies",
    )


def build_gear(
    session,
    geo: SetGeometry,
    member: str,
    save_path: str | None = None,
) -> BuildResult:
    """Build one member of the set as a new part document."""
    m = geo.member(member)
    model = session.new_part()
    fm = model.FeatureManager

    axis_feat = create_axis(model)
    build_blank(session.app, model, geo, member)

    # --- 3. tooth space sections -----------------------------------------
    outer_sketch = _draw_section(model, geo, member, "outer")
    inner_sketch = _draw_section(model, geo, member, "inner")

    # --- 4. loft cut ------------------------------------------------------
    model.ClearSelection2(True)
    _select_feature(outer_sketch, "outer section", mark=MARK_LOFT_PROFILE)
    _select_feature(inner_sketch, "inner section", append=True, mark=MARK_LOFT_PROFILE)
    cut = require(
        fm.InsertCutBlend(
            False,   # closed loft
            False,   # keep tangency
            False,   # force non-rational
            1.0,     # tessellation tolerance factor
            0, 0,    # start / end matching: none
            False,   # thin body
            0.0, 0.0,
            0,       # thin type
            True,    # use feature scope
            True,    # auto select bodies
        ),
        "loft cut the tooth space",
    )

    _drop_offcut(model)

    # --- 5. circular pattern ---------------------------------------------
    model.ClearSelection2(True)
    _select_feature(cut, "loft cut", mark=MARK_PATTERN_FEATURE)
    _select_feature(axis_feat, "gear axis", append=True, mark=MARK_PATTERN_AXIS)
    require(
        fm.FeatureCircularPattern5(
            int(m.z),                 # instances, including the original
            # With EqualSpacing on, Spacing is the TOTAL angle swept, not the
            # per-instance pitch. Passing the pitch here packs all z instances
            # into one tooth's worth of arc and the feature fails.
            2.0 * math.pi,
            False,                    # flip direction
            "NULL",
            # Geometry pattern, and it has to be on. Off, SOLIDWORKS repeats the
            # *feature* at each instance: it re-solves the loft cut, its end
            # conditions and its feature scope against the body at every rotated
            # position, and a cut driven by two 3D sketch profiles will not
            # re-solve that way - the pattern is rejected outright. On, it copies
            # the resulting faces about the axis instead, which is also what the
            # part actually is: every tooth is an exact rotational copy of the
            # first by construction. Carl found this by ticking the box by hand.
            True,                     # geometry pattern
            True,                     # equal spacing
            False,                    # vary instance
            False,                    # sync sub-assemblies
            False, False,             # direction 2, symmetric
            1, 0.0, "NULL", False,
        ),
        f"circular pattern of {m.z} teeth",
    )

    # Both resolve as properties under late binding unless flagged.
    flag_methods(model, "EditRebuild3", "ViewZoomtofit2")
    model.EditRebuild3()
    try:
        model.ViewZoomtofit2()
    except Exception:
        pass

    # --- 6. measure what we actually got ----------------------------------
    bodies = model.GetBodies2(SW_SOLID_BODY, True)
    bodies = list(bodies) if bodies else []
    if len(bodies) != 1:
        raise SwError(f"expected exactly 1 solid body, got {len(bodies)}")

    box = tuple(v * 1000.0 for v in bodies[0].GetBodyBox())
    faces = bodies[0].GetFaces()
    max_radius = max(abs(box[0]), abs(box[1]), abs(box[3]), abs(box[4]))

    if save_path:
        session.save(model, save_path)

    return BuildResult(
        member=member,
        title=model.GetTitle,
        teeth=m.z,
        body_count=len(bodies),
        face_count=len(faces) if faces else 0,
        box_mm=box,
        max_radius_mm=max_radius,
        expected_radius_mm=m.outside_dia / 2.0,
        path=save_path,
        model=model,
    )
