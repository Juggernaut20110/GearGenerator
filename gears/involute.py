"""Planar involute tooth profiles. Pure math - no COM, no GUI, no file I/O.

Millimetres and radians throughout, like everything upstream of `gears.sw`.

The frame
---------
One flat plane. The gear is centred on the origin and a tooth **SPACE** is
centred on angle 0, so the profile this module returns is the shape a cutter
removes, not the shape of a tooth. That choice is what lets the SOLIDWORKS layer
cut one space and circular-pattern it `z` times.

Both gear types share this module, and they mean different planes by it:

* a **spur** gear uses it directly - the plane is the transverse plane, and the
  profile it returns is the real tooth form
* a **bevel** gear uses it through Tredgold's approximation - the plane is the
  developed back cone, the profile is a "virtual" spur gear with `z / cos(delta)`
  teeth, and `gears.bevel.geometry.to_cone_3d` bends the result onto the cone

Nothing here knows which caller it has. Every function takes plain radii and
angles, so the only inputs that reach the flank shape are the base radius, the
root and tip radii, `psi0` and the angular half-pitch.

Internal gears
--------------
A ring gear's teeth point **inward**, so its tip is its smallest radius and its
root its largest - the reverse of everything above. The involute is the same
curve of the same base circle, and the one identity that makes it all reusable
is this:

    an internal gear's tooth SPACE has the shape of an external gear's TOOTH

Both narrow as the radius grows, following `psi0 - inv(alpha_r)`, where the
external gear's *space* widens instead. That sign is why `internal_flank_points`
is a second function rather than `flank_points` called cleverly: no substitution
of `psi0` or `half_pitch` turns one form into the other.

`tooth_space_loop` takes an `internal` flag and returns the same named segments
either way. The names describe the tooth's own features rather than radii - the
root is where the fillet sits, the cap is where the cut clears the blank - so
they stay true read from the material inward or outward.

Why `psi0` and `half_pitch` rather than module and tooth count
--------------------------------------------------------------
Because the angular constants are **scale-invariant**: `acos(k*r_b / k*r)` is
`acos(r_b / r)` for any k. A bevel section at any cone distance is a uniform
scaling of the outer one, so one `psi0` and one `half_pitch` serve every section
along the face width. Passing the two angles instead of the quantities they were
derived from is what makes that reuse free.
"""

from __future__ import annotations

import math

Point2 = tuple[float, float]

# Default sampling density along one involute flank.
FLANK_POINTS = 40

# Smallest top land the tip is allowed to keep, as a multiple of the module.
#
# Past a point the two flanks of a tooth cross: the top land goes negative and
# neighbouring tooth spaces overlap. Callers clamp the tip radius so that never
# happens.
#
# It is not a hypothetical. A Gleason long-addendum bevel pinion with few teeth
# is already close to pointed at its nominal tip - 17 teeth at 2.53:1 leaves only
# 0.36 mm of top land - and pushing the loft section outward along the cone grows
# the tip radius further. The extension region lies outside the blank anyway, so
# clamping it costs nothing.
MIN_TOP_LAND_FACTOR = 0.05

# How far past the tip the tooth-space profile is carried, so that a cut fully
# clears the blank radially. A multiple of the module.
CUT_OVERSHOOT_FACTOR = 0.5


def inv(angle: float) -> float:
    """Involute function: inv(a) = tan(a) - a."""
    return math.tan(angle) - angle


def polar(r: float, phi: float) -> Point2:
    return r * math.cos(phi), r * math.sin(phi)


def top_land(r: float, r_base: float, psi0: float) -> float:
    """Tooth thickness at radius r, along the arc. Negative means pointed."""
    alpha_r = math.acos(min(1.0, r_base / r))
    return 2.0 * (psi0 - inv(alpha_r)) * r


def max_tip_radius(r_base: float, psi0: float, min_land: float) -> float:
    """Largest tip radius still leaving `min_land` of top land.

    `top_land` decreases monotonically once past the base circle, so a plain
    bisection is safe.
    """
    if top_land(r_base, r_base, psi0) <= min_land:
        return r_base

    lo = hi = r_base
    for _ in range(200):
        hi *= 1.05
        if top_land(hi, r_base, psi0) <= min_land:
            break
    for _ in range(80):
        mid = 0.5 * (lo + hi)
        if top_land(mid, r_base, psi0) > min_land:
            lo = mid
        else:
            hi = mid
    return lo


def flank_points(
    r_base: float,
    r_root: float,
    r_tip: float,
    psi0: float,
    half_pitch: float,
    n: int,
) -> list[Point2]:
    """One side of an external gear's tooth space, root to tip, at positive angle.

    The tooth centred on angle 0 has angular half-thickness
        theta_t(r) = psi0 - inv(acos(r_base / r))
    so the space centred on angle 0 has half-width half_pitch - theta_t(r).

    Below the base circle the involute is undefined; a radial line is used
    instead, which is the standard simplification (the true form there is a
    trochoid that depends on the cutter).
    """

    def space_angle(r: float) -> float:
        alpha_r = math.acos(min(1.0, r_base / r))
        return half_pitch - psi0 + inv(alpha_r)

    pts: list[Point2] = []

    # Sample uniformly in the involute roll parameter rather than in radius:
    # it distributes points evenly along the curve instead of bunching them at
    # the tip.
    r_lo = max(r_root, r_base)
    if r_root < r_base:
        pts.append(polar(r_root, space_angle(r_base)))

    t_lo = math.sqrt(max(0.0, (r_lo / r_base) ** 2 - 1.0))
    t_hi = math.sqrt(max(0.0, (r_tip / r_base) ** 2 - 1.0))
    for i in range(n):
        t = t_lo + (t_hi - t_lo) * i / (n - 1)
        r = r_base * math.sqrt(1.0 + t * t)
        pts.append(polar(r, space_angle(r)))

    return pts


def internal_flank_points(
    r_base: float,
    r_root: float,
    r_tip: float,
    psi0: float,
    n: int,
) -> list[Point2]:
    """One side of an *internal* gear's tooth space, tip to root, positive angle.

    An internal gear's tooth space has the shape of an external gear's **tooth**.
    That is the whole of the difference, and it is worth stating as an identity
    rather than a rule of thumb, because it is what lets one involute serve
    both. Compare the two half-widths:

        external tooth   theta_t(r) = psi0 - inv(alpha_r)      narrows outward
        external space   theta_s(r) = half_pitch - theta_t(r)  widens outward
        internal space   theta_s(r) = psi0 - inv(alpha_r)      narrows outward

    So the sign on `inv` flips, and that is why this cannot be `flank_points`
    called with its arguments rearranged - no substitution of psi0 or half_pitch
    turns one into the other. `psi0` here is the **space** half-width
    extrapolated to the base circle, `e / (2 * r_p) + inv(alpha_t)`, where the
    external gear's psi0 is built from the tooth thickness instead.

    The radial run also reverses. An internal gear's tip is its *innermost*
    radius - the teeth point inward, toward the axis - and its root is furthest
    out, so the space runs from `r_tip` up to `r_root` with r_tip < r_root.

    There is no below-the-base-circle case to handle. The internal root is the
    outermost radius of the whole tooth, so if the tip clears the base circle
    then so does everything above it; a tip that does *not* is a gear with no
    involute flank at all, and `validate` refuses it rather than drawing a
    radial line and pretending.
    """

    def space_angle(r: float) -> float:
        alpha_r = math.acos(min(1.0, r_base / r))
        return psi0 - inv(alpha_r)

    # Sampled uniformly in the roll parameter, same as the external case, and
    # ordered inner to outer so the caller gets tip-to-root - the direction that
    # matches "root to tip" once you remember which way an internal tooth points.
    t_lo = math.sqrt(max(0.0, (max(r_tip, r_base) / r_base) ** 2 - 1.0))
    t_hi = math.sqrt(max(0.0, (r_root / r_base) ** 2 - 1.0))

    pts: list[Point2] = []
    for i in range(n):
        t = t_lo + (t_hi - t_lo) * i / (n - 1)
        r = r_base * math.sqrt(1.0 + t * t)
        pts.append(polar(r, space_angle(r)))
    return pts


def internal_space_width(r: float, r_base: float, psi0: float) -> float:
    """Width of an internal gear's tooth space at radius r, along the arc.

    The mirror of `top_land`: that measures an external gear's tooth, this
    measures an internal gear's space, and they are the same formula because
    they are the same shape. Negative means the space has closed up.
    """
    return top_land(r, r_base, psi0)


def internal_tooth_width(
    r: float, r_base: float, psi0: float, half_pitch: float
) -> float:
    """Width of an internal gear's *tooth* at radius r, along the arc.

    What is left of the pitch after the space is taken out. An internal tooth
    is narrowest at its **tip**, which is its innermost radius - the opposite
    end from an external tooth - so this is what decides whether a ring gear's
    teeth come to a point, and it is checked at the tip radius.
    """
    return 2.0 * (half_pitch - (psi0 - inv(math.acos(min(1.0, r_base / r))))) * r


def min_internal_tip_radius(
    r_base: float, psi0: float, half_pitch: float, min_land: float
) -> float:
    """Smallest tip radius an internal gear can have and still keep a top land.

    The counterpart of `max_tip_radius`, and it bisects the other way round:
    an internal tooth widens as the radius grows, so the constraint is a floor
    rather than a ceiling.
    """
    if internal_tooth_width(r_base, r_base, psi0, half_pitch) > min_land:
        return r_base

    lo, hi = r_base, r_base
    for _ in range(200):
        hi *= 1.05
        if internal_tooth_width(hi, r_base, psi0, half_pitch) > min_land:
            break
    for _ in range(80):
        mid = 0.5 * (lo + hi)
        if internal_tooth_width(mid, r_base, psi0, half_pitch) > min_land:
            hi = mid
        else:
            lo = mid
    return hi


def point_segment_distance(p: Point2, a: Point2, b: Point2) -> tuple[float, Point2]:
    ax, ay = a
    bx, by = b
    px, py = p
    dx, dy = bx - ax, by - ay
    denom = dx * dx + dy * dy
    if denom < 1e-18:
        return math.hypot(px - ax, py - ay), a
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / denom))
    q = (ax + t * dx, ay + t * dy)
    return math.hypot(px - q[0], py - q[1]), q


def distance_to_polyline(p: Point2, poly: list[Point2]) -> tuple[float, Point2, int]:
    best = (float("inf"), poly[0], 0)
    for i in range(len(poly) - 1):
        d, q = point_segment_distance(p, poly[i], poly[i + 1])
        if d < best[0]:
            best = (d, q, i)
    return best


def root_fillet(
    flank: list[Point2],
    r_root: float,
    rho: float,
    arc_points: int = 9,
    internal: bool = False,
) -> tuple[list[Point2], list[Point2]] | None:
    """Fit a circular fillet of radius `rho` tangent to the flank and the root.

    Returns (trimmed_flank, arc_points_root_to_flank), or None if no fillet of
    that size fits inside the space - in which case the caller keeps a sharp
    corner.

    The fillet centre must sit one fillet radius **inside the material** from
    the root circle, so only its angle is unknown. For an external gear the root
    is the innermost radius and the centre goes at `r_root + rho`; for an
    internal gear the root is the outermost and it goes at `r_root - rho`. That
    one sign is the whole difference - distance-to-flank still decreases
    monotonically as the angle sweeps from the space centreline toward the
    flank, so the same bisection serves both.

    `flank` must start at the root end either way. The internal flank generator
    returns tip-to-root, so its caller reverses it before handing it over.
    """
    if rho <= 0.0:
        return None

    phi_flank = math.atan2(flank[0][1], flank[0][0])
    if phi_flank <= 0.0:
        return None

    r_centre = r_root - rho if internal else r_root + rho
    if r_centre <= 0.0:
        return None

    def gap(phi: float) -> float:
        c = polar(r_centre, phi)
        return distance_to_polyline(c, flank)[0] - rho

    lo, hi = 0.0, phi_flank
    if gap(lo) <= 0.0:
        return None  # space too narrow for this fillet radius

    for _ in range(60):
        mid = 0.5 * (lo + hi)
        if gap(mid) > 0.0:
            lo = mid
        else:
            hi = mid
    phi_c = 0.5 * (lo + hi)

    centre = polar(r_centre, phi_c)
    _, touch, seg = distance_to_polyline(centre, flank)

    # Tangent point on the root circle is radially outward from the centre for
    # an internal gear and inward for an external one - which is just to say it
    # is on the root circle at the centre's own angle, either way.
    root_touch = polar(r_root, phi_c)

    a0 = math.atan2(root_touch[1] - centre[1], root_touch[0] - centre[0])
    a1 = math.atan2(touch[1] - centre[1], touch[0] - centre[0])
    # Keep the short way round.
    while a1 - a0 > math.pi:
        a1 -= 2.0 * math.pi
    while a0 - a1 > math.pi:
        a1 += 2.0 * math.pi

    arc = [
        (
            centre[0] + rho * math.cos(a0 + (a1 - a0) * i / (arc_points - 1)),
            centre[1] + rho * math.sin(a0 + (a1 - a0) * i / (arc_points - 1)),
        )
        for i in range(arc_points)
    ]
    return [touch] + flank[seg + 1:], arc


def tooth_space_loop(
    r_base: float,
    r_root: float,
    r_tip: float,
    r_cap: float,
    psi0: float,
    half_pitch: float,
    fillet_rho: float,
    n_flank: int = FLANK_POINTS,
    split_cap: bool = False,
    internal: bool = False,
) -> tuple[dict[str, list[Point2]], list[Point2], bool]:
    """One closed tooth-space boundary: named segments, flat loop, fillet flag.

    The loop runs counter-clockwise: up the negative flank, out past the tip,
    across the cap, back down the positive flank, then round the root. It is
    returned both as named pieces and as one flattened polyline, because the two
    consumers want different things - the SOLIDWORKS layer builds splines and
    arcs separately rather than forcing one spline through a sharp corner, while
    the preview and DXF export want a single closed polyline.

    `split_cap` breaks the cap into `cap_neg` + `cap_pos` meeting at a real
    vertex on the space centreline. A guided loft needs its guide curve to touch
    a point that exists in every profile, and a point that merely lies *near* a
    spline is the classic way such a loft fails - so the helical spur builder
    asks for the vertex to be there by construction.

    `internal` builds a ring gear's space instead. **Every segment keeps its
    name and its job**, and the caller sees the same dictionary; what changes is
    which way "out past the tip" points. An internal gear's teeth point inward,
    so its tip is its smallest radius and its root its largest, and the caller
    passes `r_tip < r_root` with `r_cap` smaller still. The names stay honest
    because they name the tooth's own features, not radii: the root is where the
    fillet is, the cap is where the cut clears the blank, and both are true of
    a ring gear read from the material outward.
    """
    if internal:
        # Tip to root, so reverse it: `root_fillet` and the loop assembly below
        # both want the flank to start at the root end.
        flank = internal_flank_points(r_base, r_root, r_tip, psi0, n_flank)[::-1]
    else:
        flank = flank_points(r_base, r_root, r_tip, psi0, half_pitch, n_flank)

    fillet = root_fillet(flank, r_root, fillet_rho, internal=internal)
    if fillet is not None:
        flank, arc = fillet
    else:
        arc = []

    # Mirror across the x axis for the other side of the space.
    def mirror(pts: list[Point2]) -> list[Point2]:
        return [(x, -y) for x, y in pts]

    phi_root = math.atan2(arc[0][1], arc[0][0]) if arc else math.atan2(
        flank[0][1], flank[0][0]
    )
    phi_tip = math.atan2(flank[-1][1], flank[-1][0])

    root_arc = [polar(r_root, -phi_root + 2.0 * phi_root * i / 8.0) for i in range(9)]
    cap = [polar(r_cap, -phi_tip + 2.0 * phi_tip * i / 4.0) for i in range(5)]

    segments = {
        # Root to flank, matching the direction of travel round the loop; the
        # positive side runs the other way and so is reversed instead.
        "fillet_neg": mirror(arc) if arc else [],
        "flank_neg": mirror(flank),
        "riser_neg": [mirror(flank)[-1], cap[0]],
        "cap": cap,
        "riser_pos": [cap[-1], flank[-1]],
        "flank_pos": flank[::-1],
        "fillet_pos": arc[::-1] if arc else [],
        "root": root_arc[::-1],
    }
    order = [
        "fillet_neg", "flank_neg", "riser_neg", "cap",
        "riser_pos", "flank_pos", "fillet_pos", "root",
    ]

    if split_cap:
        # cap has 5 points, so index 2 is exactly on the centreline.
        mid = len(cap) // 2
        segments["cap_neg"] = cap[: mid + 1]
        segments["cap_pos"] = cap[mid:]
        del segments["cap"]
        order[order.index("cap")] = "cap_neg"
        order.insert(order.index("cap_neg") + 1, "cap_pos")

    loop: list[Point2] = []
    for name in order:
        for pt in segments[name]:
            if not loop or math.dist(loop[-1], pt) > 1e-9:
                loop.append(pt)
    if loop and math.dist(loop[0], loop[-1]) < 1e-9:
        loop.pop()

    return segments, loop, bool(arc)
