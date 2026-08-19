"""Part-building steps that do not care what kind of gear is being built.

Every gear this package makes is the same five moves: an axis, a revolved blank
driven by named global variables, one or more 3D sketch sections, a loft cut,
and a circular pattern. Only two things differ between types - the meridian
outline the blank is revolved from, and the shape and placement of the sections -
so everything else lives here and is called by both builders.

Extracted from the bevel builder rather than written fresh, so the comments
below are its hard-won facts and the measurements in them were taken on the
anchor bevel pinion. They are no less true of a spur gear.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

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
    SketchFlags,
    SwError,
    add_equation,
    add_relation,
    equation_manager,
    flag_methods,
    mm,
    points_to_doubles,
    require,
    require_angle_precision,
    require_fully_defined,
    select_first,
)

# The reference axis is named so the assembly builders can mate against it.
AXIS_FEATURE_NAME = "GearAxis"

# mm; outline coordinates that "share" a value share it exactly.
COORD_TOL = 1e-9


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


# ---------------------------------------------------------------------------
# Holding on to things
# ---------------------------------------------------------------------------


def model_to_sketch(sketch):
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


def last_feature(model):
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


def select_feature(feat, what: str, append: bool = False, mark: int = 0) -> None:
    if not feat.Select2(bool(append), int(mark)):
        raise SwError(f"could not select {what} ({feat.Name})")


def endpoint_methods(line):
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


# ---------------------------------------------------------------------------
# The axis
# ---------------------------------------------------------------------------


def create_axis(model):
    """Reference axis along Z: the intersection of the Top and Right planes.

    Top Plane is XZ and Right Plane is YZ, so they meet along Z - which is the
    gear axis in every geometry module's coordinate system.

    The axis is renamed because the assembly mates against it by name, and
    "Axis1" is both unrecognisable in the tree and not guaranteed to be the
    number a part picks up. Renaming is safe here: everything selects the
    feature *object*, never the name.
    """
    model.ClearSelection2(True)
    select_first(model, TOP_PLANE_NAMES, "PLANE")
    select_first(model, RIGHT_PLANE_NAMES, "PLANE", append=True)
    # InsertAxis2 lives on IModelDoc2, not IFeatureManager.
    require(model.InsertAxis2(True), "InsertAxis2")
    axis = last_feature(model)
    try:
        axis.Name = AXIS_FEATURE_NAME
    except Exception:
        pass
    return axis


# ---------------------------------------------------------------------------
# The blank
# ---------------------------------------------------------------------------


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

    The bevel blank has one, `OuterRootToApex`. The sketch has to be dimensioned
    to the *back face* - the alternative is an apex-to-point distance, which is a
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


def axis_datum(axis):
    """The centreline endpoint sitting at the sketch origin.

    Every axial dimension measures from it - the pitch apex on a bevel blank,
    the front face on a spur one - so it is whichever endpoint is at (0, 0) in
    sketch space. Taking the nearer of the two rather than assuming
    `CreateCenterLine` kept its arguments in the order they were given.
    """
    a, b = axis.GetStartPoint2(), axis.GetEndPoint2()
    return a if math.hypot(a.X, a.Y) <= math.hypot(b.X, b.Y) else b


def radial_position(r: float, z: float) -> tuple[float, float, float]:
    """Put a radial dimension halfway between the axis and what it measures."""
    return (mm(0.5 * r), 0.0, mm(z))


def axial_position(reach: float, z: float) -> tuple[float, float, float]:
    """Put an axial dimension to the left of the axis, where nothing is drawn."""
    return (mm(-0.3 * reach), 0.0, mm(z))


def sketch_blank_outline(model, outline, axis_overshoot: float = 5.0):
    """Open a sketch on the Top Plane and draw the meridian outline in it.

    Returns `(sketch_manager, centreline, lines)` with the sketch still **open**,
    because relations and dimensions are edits to the active sketch.

    A single centreline in the sketch is what `FeatureRevolve2` picks up as its
    axis, which saves selecting one explicitly - so this stays the only
    construction line. It starts at the origin because that endpoint is also the
    datum every axial dimension measures from.
    """
    mgr = model.SketchManager
    z_hi = max(z for _, z in outline)

    model.ClearSelection2(True)
    select_first(model, TOP_PLANE_NAMES, "PLANE")
    mgr.InsertSketch(True)
    to_sketch = model_to_sketch(mgr.ActiveSketch)

    with SketchFlags(model):
        c1 = to_sketch(0.0, 0.0, 0.0)
        c2 = to_sketch(0.0, 0.0, mm(z_hi + axis_overshoot))
        axis = endpoint_methods(
            require(
                mgr.CreateCenterLine(c1[0], c1[1], 0.0, c2[0], c2[1], 0.0),
                "blank centreline",
            )
        )
        lines = []
        closed = list(outline) + [outline[0]]
        for i, ((r1, za), (r2, zb)) in enumerate(zip(closed, closed[1:])):
            a = to_sketch(mm(r1), 0.0, mm(za))
            b = to_sketch(mm(r2), 0.0, mm(zb))
            lines.append(
                endpoint_methods(
                    require(
                        mgr.CreateLine(a[0], a[1], 0.0, b[0], b[1], 0.0),
                        f"blank outline segment {i}",
                    )
                )
            )
    return mgr, axis, lines


def constrain_blank(model, axis, lines, outline) -> None:
    """Relations first - the dimensions need something to hold on to.

    No merge pass is needed at the corners. `AddToDB` being on suppresses
    inference against *existing* geometry, but a line started exactly where the
    previous one ended still shares its point: measured on the bevel sketch, five
    lines and a centreline give seven sketch points, not twelve, and
    `a.GetEndPoint2()` and `b.GetStartPoint2()` come back as the same underlying
    object. So the profile arrives already closed, with one vertex per outline
    point, which is what each builder's degree-of-freedom count assumes.

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
        if abs(z1 - z2) < COORD_TOL:
            add_relation(model, (line,), REL_HORIZONTAL, f"blank segment {i} flat")
        elif abs(r1 - r2) < COORD_TOL:
            add_relation(model, (line,), REL_VERTICAL, f"blank segment {i} cylindrical")


def link_blank_equations(model, sketch_name: str, dims, variables=()) -> None:
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
    the geometry. Measured on the anchor bevel pinion, setting CrownRadius to
    22 mm walks the crown from (19.9220, 22.5585) to (22.0000, 21.0000) - along
    the back cone, exactly where the 53.13 degree angle puts it.
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


def close_and_revolve_blank(model, mgr, dims, variables=()):
    """Close the dimensioned sketch, bind its equations, and revolve it 360."""
    require_fully_defined(mgr.ActiveSketch, "blank profile sketch")

    mgr.InsertSketch(True)
    model.ClearSelection2(True)
    blank_sketch = last_feature(model)

    # Only now is the sketch's own name settled, and an equation needs it to
    # name the dimension it drives.
    link_blank_equations(model, blank_sketch.Name, dims, variables)

    select_feature(blank_sketch, "blank profile sketch")
    require(
        model.FeatureManager.FeatureRevolve2(
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
    return last_feature(model)


# ---------------------------------------------------------------------------
# The tooth
# ---------------------------------------------------------------------------


def draw_curves_3d(model, curves, what: str):
    """Draw a list of (name, 3D points) as one 3D sketch; return the feature.

    Two points become a line and more become a spline, so genuinely sharp
    corners stay sharp instead of being smoothed over by a single spline forced
    through the whole loop.
    """
    mgr = flag_methods(model.SketchManager, "CreateSpline", "CreateSpline2")

    mgr.Insert3DSketch(True)
    with SketchFlags(model):
        for name, pts in curves:
            if len(pts) < 2:
                continue
            if len(pts) == 2:
                (x1, y1, z1), (x2, y2, z2) = pts
                require(
                    mgr.CreateLine(mm(x1), mm(y1), mm(z1), mm(x2), mm(y2), mm(z2)),
                    f"{what} line {name}",
                )
            else:
                require(
                    mgr.CreateSpline(points_to_doubles(pts)),
                    f"{what} spline {name} ({len(pts)} points)",
                )
    mgr.Insert3DSketch(True)
    model.ClearSelection2(True)
    return last_feature(model)


def loft_cut(model, profiles, guides=(), guide_mark: int = 0):
    """Cut the tooth space by lofting between the section sketches.

    `guides` are selected under `guide_mark` if given. A straight-toothed gear
    needs none: the two sections are the same shape and the ruled surface between
    them is the tooth flank. A helical one does, because the flank between two
    rotated sections is *not* the helicoid unless something forces the twist to
    be uniform along the way.
    """
    model.ClearSelection2(True)
    for i, sketch in enumerate(profiles):
        select_feature(
            sketch, f"loft profile {i}", append=i > 0, mark=MARK_LOFT_PROFILE
        )
    for i, guide in enumerate(guides):
        select_feature(guide, f"loft guide {i}", append=True, mark=guide_mark)

    return require(
        model.FeatureManager.InsertCutBlend(
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


def drop_offcut(model) -> None:
    """Discard the chunk `InsertCutBlend` leaves behind.

    A loft cut here does not delete the material it encloses - it *splits* the
    blank in two and keeps both halves. Measured on the anchor bevel pinion: the
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


def pattern_teeth(model, cut, axis_feat, z: int) -> None:
    """Circular-pattern the tooth-space cut into a full set of teeth."""
    model.ClearSelection2(True)
    select_feature(cut, "loft cut", mark=MARK_PATTERN_FEATURE)
    select_feature(axis_feat, "gear axis", append=True, mark=MARK_PATTERN_AXIS)
    require(
        model.FeatureManager.FeatureCircularPattern5(
            int(z),                   # instances, including the original
            # With EqualSpacing on, Spacing is the TOTAL angle swept, not the
            # per-instance pitch. Passing the pitch here packs all z instances
            # into one tooth's worth of arc and the feature fails.
            2.0 * math.pi,
            False,                    # flip direction
            "NULL",
            # Geometry pattern, and it has to be on. Off, SOLIDWORKS repeats the
            # *feature* at each instance: it re-solves the loft cut, its end
            # conditions and its feature scope against the body at every rotated
            # position, and a cut driven by 3D sketch profiles will not re-solve
            # that way - the pattern is rejected outright. On, it copies the
            # resulting faces about the axis instead, which is also what the part
            # actually is: every tooth is an exact rotational copy of the first by
            # construction. Carl found this by ticking the box by hand.
            True,                     # geometry pattern
            True,                     # equal spacing
            False,                    # vary instance
            False,                    # sync sub-assemblies
            False, False,             # direction 2, symmetric
            1, 0.0, "NULL", False,
        ),
        f"circular pattern of {z} teeth",
    )


# ---------------------------------------------------------------------------
# Measuring what came out
# ---------------------------------------------------------------------------


def measure_part(
    session,
    model,
    member: str,
    teeth: int,
    expected_radius_mm: float,
    save_path: str | None = None,
) -> BuildResult:
    """Rebuild, measure, optionally save. The build's own sanity check.

    `GetBodyBox` is tessellation-based and only guaranteed to *contain* the
    body, so `max_radius_mm` is an upper bound on the outside radius rather than
    a measurement of it. Callers treat a sub-percent discrepancy as agreement.
    """
    # Both resolve as properties under late binding unless flagged.
    flag_methods(model, "EditRebuild3", "ViewZoomtofit2")
    model.EditRebuild3()
    try:
        model.ViewZoomtofit2()
    except Exception:
        pass

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
        teeth=teeth,
        body_count=len(bodies),
        face_count=len(faces) if faces else 0,
        box_mm=box,
        max_radius_mm=max_radius,
        expected_radius_mm=expected_radius_mm,
        path=save_path,
        model=model,
    )
