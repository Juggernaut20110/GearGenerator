"""Closed-form checks on the geometry engine. SOLIDWORKS is not involved."""

from __future__ import annotations

import math

import pytest

from bevelgear.geometry import (
    CLEARANCE_FACTOR,
    MIN_TOP_LAND_FACTOR,
    WHOLE_DEPTH_FACTOR,
    WORKING_DEPTH_FACTOR,
    max_tip_radius,
    top_land,
    beyond_back_cone,
    blank_outline,
    blank_reach_past_back_cone,
    compute_set,
    end_overshoot,
    front_face_z,
    inv,
    tip_radius_at_cone_distance,
    to_cone_3d,
    tooth_space_section,
)
from bevelgear.params import BevelSetParams

# The anchor case used throughout: m=2, 17 x 43 teeth, 20 deg, 90 deg shafts.
ANCHOR = BevelSetParams.with_defaults(2.0, 17, 43)


@pytest.fixture
def geo():
    return compute_set(ANCHOR)


# --- cone angles -----------------------------------------------------------


def test_anchor_pitch_angles(geo):
    assert geo.pinion.pitch_angle_deg == pytest.approx(
        math.degrees(math.atan2(17, 43)), abs=1e-9
    )
    assert geo.pinion.pitch_angle_deg == pytest.approx(21.5713, abs=1e-4)
    assert geo.gear.pitch_angle_deg == pytest.approx(68.4287, abs=1e-4)


def test_anchor_outer_cone_distance(geo):
    assert geo.outer_cone_dist == pytest.approx(46.2385, abs=1e-4)


@pytest.mark.parametrize("sigma", [45.0, 60.0, 90.0, 120.0, 150.0])
@pytest.mark.parametrize("z1,z2", [(17, 43), (20, 20), (12, 60)])
def test_cone_distance_agrees_between_members(sigma, z1, z2):
    """Ao computed from either member must match - the core consistency check."""
    p = BevelSetParams.with_defaults(2.0, z1, z2, shaft_angle=sigma)
    g = compute_set(p)
    from_pinion = g.pinion.pitch_dia / (2.0 * math.sin(g.pinion.pitch_angle))
    from_gear = g.gear.pitch_dia / (2.0 * math.sin(g.gear.pitch_angle))
    assert from_pinion == pytest.approx(from_gear, rel=1e-12)
    assert g.outer_cone_dist == pytest.approx(from_gear, rel=1e-12)


@pytest.mark.parametrize("sigma", [30.0, 45.0, 90.0, 135.0, 170.0])
def test_pitch_angles_sum_to_shaft_angle(sigma):
    g = compute_set(BevelSetParams.with_defaults(2.0, 17, 43, shaft_angle=sigma))
    total = g.pinion.pitch_angle + g.gear.pitch_angle
    assert math.degrees(total) == pytest.approx(sigma, abs=1e-9)


def test_miter_pair_splits_the_shaft_angle_evenly():
    g = compute_set(BevelSetParams.with_defaults(2.0, 20, 20))
    assert g.pinion.pitch_angle_deg == pytest.approx(45.0, abs=1e-9)
    assert g.gear.pitch_angle_deg == pytest.approx(45.0, abs=1e-9)


# --- Gleason proportions ---------------------------------------------------


def test_addendums_sum_to_working_depth(geo):
    assert geo.pinion.addendum + geo.gear.addendum == pytest.approx(
        WORKING_DEPTH_FACTOR * ANCHOR.module, rel=1e-12
    )


def test_pinion_gets_the_long_addendum(geo):
    assert geo.pinion.addendum > geo.gear.addendum


def test_depths_are_consistent(geo):
    for m in (geo.pinion, geo.gear):
        assert m.addendum + m.dedendum == pytest.approx(
            WHOLE_DEPTH_FACTOR * ANCHOR.module, rel=1e-12
        )
    assert geo.whole_depth - geo.working_depth == pytest.approx(
        CLEARANCE_FACTOR * ANCHOR.module, rel=1e-12
    )


def test_addendum_angle_of_each_member_is_the_mates_dedendum_angle(geo):
    """This is what makes the clearance constant along the face width."""
    assert geo.pinion.addendum_angle == pytest.approx(geo.gear.dedendum_angle)
    assert geo.gear.addendum_angle == pytest.approx(geo.pinion.dedendum_angle)


def test_root_cone_passes_through_the_pitch_apex(geo):
    """Dedendum angle = atan(b_f/Ao) puts the root cone apex at the origin.

    Verified by checking that the outer root point lies on the ray from the
    origin at the root angle - the property the section scaling depends on.
    """
    for m in (geo.pinion, geo.gear):
        cos_d, sin_d = math.cos(m.pitch_angle), math.sin(m.pitch_angle)
        root_r = geo.outer_cone_dist * sin_d - m.dedendum * cos_d
        root_z = geo.outer_cone_dist * cos_d + m.dedendum * sin_d
        assert math.atan2(root_r, root_z) == pytest.approx(m.root_angle, abs=1e-12)


def test_outside_diameter_matches_the_formula(geo):
    for m in (geo.pinion, geo.gear):
        assert m.outside_dia == pytest.approx(
            m.pitch_dia + 2.0 * m.addendum * math.cos(m.pitch_angle), rel=1e-12
        )


# --- virtual (back cone) spur gear -----------------------------------------


def test_virtual_pitch_radius_two_ways(geo):
    """r_p = (d/2)/cos(delta) must equal m*z_v/2."""
    for m in (geo.pinion, geo.gear):
        assert m.virtual_pitch_r == pytest.approx(
            ANCHOR.module * m.virtual_teeth / 2.0, rel=1e-12
        )


def test_base_radius_from_pressure_angle(geo):
    for m in (geo.pinion, geo.gear):
        assert m.virtual_base_r == pytest.approx(
            m.virtual_pitch_r * math.cos(ANCHOR.alpha), rel=1e-12
        )


def test_involute_starts_on_the_base_circle_at_zero_angle():
    """At roll angle 0 the involute touches the base circle, with inv(0) == 0."""
    assert inv(0.0) == pytest.approx(0.0, abs=1e-15)
    r_b = 10.0
    t = 0.0
    x = r_b * (math.sin(t) - t * math.cos(t))
    y = r_b * (math.cos(t) + t * math.sin(t))
    assert math.hypot(x, y) == pytest.approx(r_b, rel=1e-12)
    assert math.atan2(x, y) == pytest.approx(0.0, abs=1e-15)


def test_inv_of_pressure_angle():
    assert inv(math.radians(20.0)) == pytest.approx(0.0149044, abs=1e-7)


# --- the cone mapping (plan section 4) -------------------------------------


@pytest.mark.parametrize("delta_deg", [5.0, 21.5658, 45.0, 68.4342, 85.0])
@pytest.mark.parametrize("r,phi", [(10.0, 0.05), (25.0, 0.2), (40.0, -0.13)])
def test_cone_mapping_preserves_arc_length(delta_deg, r, phi):
    """R * theta == r * phi. The single best guard against a lost cos(delta)."""
    delta = math.radians(delta_deg)
    x, y, z = to_cone_3d(r * math.cos(phi), r * math.sin(phi), delta, 0.0)
    radius = math.hypot(x, y)
    theta = math.atan2(y, x)
    assert radius == pytest.approx(r * math.cos(delta), rel=1e-12)
    assert radius * theta == pytest.approx(r * phi, rel=1e-12)


def test_cone_mapping_turns_virtual_teeth_into_real_teeth(geo):
    """One virtual angular pitch must map to exactly one real angular pitch."""
    for m in (geo.pinion, geo.gear):
        virtual_pitch = 2.0 * math.pi / m.virtual_teeth
        r = m.virtual_pitch_r
        x, y, _ = to_cone_3d(
            r * math.cos(virtual_pitch), r * math.sin(virtual_pitch), m.pitch_angle, 0.0
        )
        assert math.atan2(y, x) == pytest.approx(m.angular_pitch, rel=1e-12)


def test_pitch_point_lands_where_it_should(geo):
    """The virtual pitch radius must map onto the real outer pitch point."""
    for m in (geo.pinion, geo.gear):
        apex_z = geo.outer_cone_dist / math.cos(m.pitch_angle)
        x, y, z = to_cone_3d(m.virtual_pitch_r, 0.0, m.pitch_angle, apex_z)
        assert math.hypot(x, y) == pytest.approx(m.pitch_dia / 2.0, rel=1e-12)
        assert z == pytest.approx(
            geo.outer_cone_dist * math.cos(m.pitch_angle), rel=1e-12
        )


# --- the inner section -----------------------------------------------------


def test_root_radius_scales_uniformly(geo):
    """Root cone apexes at the origin, so root radii are a plain k scaling."""
    outer = tooth_space_section(geo, "pinion", "outer")
    inner = tooth_space_section(geo, "pinion", "inner")
    assert inner.r_root == pytest.approx(geo.section_scale * outer.r_root, rel=1e-12)


def test_inner_tip_radius_is_not_a_uniform_scaling(geo):
    """The Gleason face cone apex is offset, so the tip must not scale by k."""
    for m in (geo.pinion, geo.gear):
        naive = geo.section_scale * m.virtual_tip_r
        assert m.virtual_tip_r_inner != pytest.approx(naive, rel=1e-6)
        assert m.virtual_tip_r_inner < m.virtual_tip_r


def test_inner_tip_radius_does_scale_when_the_face_cone_apexes_at_origin(geo):
    """Sanity check on the solver: feed it the standard (untilted) face angle.

    With theta_a = atan(a/Ao) the face cone passes through the pitch apex, and
    the solved inner tip radius must collapse to exactly k * outer.
    """
    m = geo.pinion
    delta = m.pitch_angle
    cos_d, sin_d = math.cos(delta), math.sin(delta)
    tip_R = geo.outer_cone_dist * sin_d + m.addendum * cos_d
    tip_z = geo.outer_cone_dist * cos_d - m.addendum * sin_d
    standard_face_angle = delta + math.atan(m.addendum / geo.outer_cone_dist)

    solved = tip_radius_at_cone_distance(
        geo.inner_cone_dist, delta, standard_face_angle, tip_R, tip_z
    )
    assert solved == pytest.approx(geo.section_scale * m.virtual_tip_r, rel=1e-10)


@pytest.mark.parametrize("member", ["pinion", "gear"])
def test_tip_solver_reproduces_the_outer_tip_at_the_outer_cone_distance(geo, member):
    """The same solver serves both ends: at Ao it must give exactly r_p + a."""
    m = geo.member(member)
    solved = tip_radius_at_cone_distance(
        geo.outer_cone_dist,
        m.pitch_angle,
        m.face_angle,
        m.outside_dia / 2.0,
        m.crown_to_apex,
    )
    assert solved == pytest.approx(m.virtual_tip_r, rel=1e-12)


@pytest.mark.parametrize("member", ["pinion", "gear"])
def test_overshoot_extends_sections_past_the_face_width(geo, member):
    """The loft sections must finish clear of the blank, not tangent to it.

    A section sitting exactly on the back cone gives SOLIDWORKS a cut that ends
    on an existing face, which it rejects as zero-thickness geometry.
    """
    over = end_overshoot(geo, member)
    assert over > 0.0

    plain_outer = tooth_space_section(geo, member, "outer")
    past_outer = tooth_space_section(geo, member, "outer", overshoot=over)
    plain_inner = tooth_space_section(geo, member, "inner")
    past_inner = tooth_space_section(geo, member, "inner", overshoot=over)

    # Outer moves away from the apex, inner moves toward it.
    assert past_outer.cone_apex_z > plain_outer.cone_apex_z
    assert past_inner.cone_apex_z < plain_inner.cone_apex_z

    # Every point of a section at cone distance A sits exactly (A - Ao) away
    # from the back cone, measured along the pitch cone. So the whole extended
    # outer section clears the back cone by the overshoot, uniformly - not just
    # its extreme point.
    m = geo.member(member)
    delta = m.pitch_angle

    def distance_from_cone(pt, cone_dist):
        x, y, z = pt
        radius = math.hypot(x, y)
        return (radius - cone_dist * math.sin(delta)) * math.sin(delta) + (
            z - cone_dist * math.cos(delta)
        ) * math.cos(delta)

    for pt in past_outer.loop_3d():
        assert distance_from_cone(pt, geo.outer_cone_dist) == pytest.approx(
            over, abs=1e-9
        )
    for pt in past_inner.loop_3d():
        assert distance_from_cone(pt, geo.inner_cone_dist) == pytest.approx(
            -over, abs=1e-9
        )


def test_section_scale_matches_cone_distances(geo):
    assert geo.section_scale == pytest.approx(
        geo.inner_cone_dist / geo.outer_cone_dist, rel=1e-12
    )
    assert geo.inner_cone_dist == pytest.approx(
        geo.outer_cone_dist - ANCHOR.face_width, rel=1e-12
    )


# --- tooth space profile ---------------------------------------------------


@pytest.mark.parametrize("member", ["pinion", "gear"])
@pytest.mark.parametrize("end", ["outer", "inner"])
def test_section_is_a_closed_non_degenerate_loop(geo, member, end):
    s = tooth_space_section(geo, member, end)
    assert len(s.loop_2d) > 40
    # No duplicate consecutive points, and the loop is not self-closing twice.
    for a, b in zip(s.loop_2d, s.loop_2d[1:]):
        assert math.dist(a, b) > 1e-9


@pytest.mark.parametrize("member", ["pinion", "gear"])
def test_section_is_symmetric_about_the_x_axis(geo, member):
    """The space is centred on angle 0, so the loop must mirror across y=0."""
    s = tooth_space_section(geo, member, "outer")
    ys = sorted(y for _, y in s.loop_2d)
    for lo, hi in zip(ys, reversed(ys)):
        assert lo == pytest.approx(-hi, abs=1e-9)


@pytest.mark.parametrize("member", ["pinion", "gear"])
def test_section_spans_root_to_beyond_tip(geo, member):
    s = tooth_space_section(geo, member, "outer")
    radii = [math.hypot(x, y) for x, y in s.loop_2d]
    assert min(radii) == pytest.approx(s.r_root, abs=1e-6)
    assert max(radii) == pytest.approx(s.r_cap, abs=1e-6)
    assert s.r_cap > s.r_tip  # the cut must clear the blank


def _angle_where_radius_crosses(points, r_target):
    """Interpolate along a polyline to the angle at which it crosses r_target."""
    for a, b in zip(points, points[1:]):
        ra, rb = math.hypot(*a), math.hypot(*b)
        if (ra - r_target) * (rb - r_target) <= 0.0 and ra != rb:
            f = (r_target - ra) / (rb - ra)
            return math.atan2(
                a[1] + f * (b[1] - a[1]), a[0] + f * (b[0] - a[0])
            )
    raise AssertionError(f"polyline never reaches r = {r_target}")


@pytest.mark.parametrize("member", ["pinion", "gear"])
def test_space_and_tooth_thickness_sum_to_the_circular_pitch(geo, member):
    """At the pitch radius, space width + tooth thickness == circular pitch."""
    m = geo.member(member)
    s = tooth_space_section(geo, member, "outer")

    half_space = _angle_where_radius_crosses(
        s.segments["flank_pos"], m.virtual_pitch_r
    )
    space_width = 2.0 * half_space * m.virtual_pitch_r
    tooth_thickness = math.pi * ANCHOR.module / 2.0
    assert space_width + tooth_thickness == pytest.approx(geo.circular_pitch, abs=0.02)


@pytest.mark.parametrize("member", ["pinion", "gear"])
def test_teeth_close_around_the_full_circle(geo, member):
    """z copies of the real angular pitch must be exactly one revolution."""
    m = geo.member(member)
    assert m.z * m.angular_pitch == pytest.approx(2.0 * math.pi, rel=1e-12)

    # And the space must be narrower than one real angular pitch, or the
    # circular pattern would overlap itself.
    s = tooth_space_section(geo, member, "outer")
    thetas = [math.atan2(y, x) for x, y, _ in s.loop_3d()]
    assert max(thetas) - min(thetas) < m.angular_pitch


def _segments_cross(a, b, c, d) -> bool:
    def side(p, q, r):
        return (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])

    d1, d2 = side(c, d, a), side(c, d, b)
    d3, d4 = side(a, b, c), side(a, b, d)
    return ((d1 > 0) != (d2 > 0)) and ((d3 > 0) != (d4 > 0))


@pytest.mark.parametrize("member", ["pinion", "gear"])
@pytest.mark.parametrize("end", ["outer", "inner"])
def test_tip_is_clamped_so_teeth_never_go_pointed(geo, member, end):
    """Extending a section along the cone must not grow the tip past pointed.

    Without the clamp, the anchor pinion's top land goes negative at the
    overshoot used for the loft, so neighbouring tooth spaces overlap.
    """
    m = geo.member(member)
    over = end_overshoot(geo, member)
    s = tooth_space_section(geo, member, end, overshoot=over)

    k = s.r_root / m.virtual_root_r
    psi0 = (math.pi * ANCHOR.module / 2.0) / (2.0 * m.virtual_pitch_r) + inv(
        ANCHOR.alpha
    )
    land = top_land(s.r_tip, k * m.virtual_base_r, psi0)
    assert land >= MIN_TOP_LAND_FACTOR * ANCHOR.module * k - 1e-9

    # And the space must stay narrower than one angular pitch, or the circular
    # pattern would have neighbouring cuts overlap.
    thetas = [math.atan2(y, x) for x, y, _ in s.loop_3d()]
    assert max(thetas) - min(thetas) < m.angular_pitch


def test_max_tip_radius_hits_the_requested_top_land():
    r_base, psi0 = 17.1779, 0.100833
    for min_land in (0.0, 0.05, 0.2, 0.5):
        r = max_tip_radius(r_base, psi0, min_land)
        assert top_land(r, r_base, psi0) == pytest.approx(min_land, abs=1e-6)


@pytest.mark.parametrize("member", ["pinion", "gear"])
@pytest.mark.parametrize("end", ["outer", "inner"])
@pytest.mark.parametrize("sigma", [45.0, 90.0])
def test_section_is_a_simple_polygon(member, end, sigma):
    """No self-intersections - SOLIDWORKS will reject the loft otherwise.

    This is the cheapest possible stand-in for "will the cut actually build",
    and it catches the profile bugs that are invisible in a list of numbers.
    """
    g = compute_set(BevelSetParams.with_defaults(2.0, 17, 43, shaft_angle=sigma))
    loop = tooth_space_section(
        g, member, end, overshoot=end_overshoot(g, member)
    ).loop_2d
    n = len(loop)
    for i in range(n):
        a, b = loop[i], loop[(i + 1) % n]
        for j in range(i + 2, n):
            if i == 0 and j == n - 1:
                continue  # adjacent across the seam
            c, d = loop[j], loop[(j + 1) % n]
            assert not _segments_cross(a, b, c, d), (
                f"segments {i} and {j} intersect"
            )


@pytest.mark.parametrize("member", ["pinion", "gear"])
def test_section_winds_counter_clockwise(geo, member):
    """The loft cut profile must have a consistent, positive orientation."""
    loop = tooth_space_section(geo, member, "outer").loop_2d
    area = 0.5 * sum(
        loop[i][0] * loop[(i + 1) % len(loop)][1]
        - loop[(i + 1) % len(loop)][0] * loop[i][1]
        for i in range(len(loop))
    )
    assert area > 0.0


def test_root_fillet_is_fitted_for_the_anchor_case(geo):
    for member in ("pinion", "gear"):
        for end in ("outer", "inner"):
            assert tooth_space_section(geo, member, end).filleted


def test_fillet_is_skipped_rather_than_crashing_when_it_cannot_fit():
    """A absurd fillet factor must degrade to a sharp corner, not raise."""
    p = BevelSetParams.with_defaults(2.0, 17, 43, fillet_factor=5.0)
    s = tooth_space_section(compute_set(p), "pinion", "outer")
    assert not s.filleted
    assert len(s.loop_2d) > 40


def test_inner_section_sits_axially_inboard_of_the_outer_one(geo):
    """Smaller cone distance means closer to the apex, i.e. lower z."""
    outer = tooth_space_section(geo, "pinion", "outer")
    inner = tooth_space_section(geo, "pinion", "inner")
    assert max(z for _, _, z in inner.loop_3d()) < min(
        z for _, _, z in outer.loop_3d()
    )


# --- blank -----------------------------------------------------------------


@pytest.mark.parametrize("member", ["pinion", "gear"])
def test_blank_outline_is_sane(geo, member):
    outline = blank_outline(geo, member)
    assert len(outline) >= 5
    r_bore = ANCHOR.bore / 2.0
    for R, z in outline:
        assert R >= r_bore - 1e-9
        assert z > 0.0  # entirely on one side of the pitch apex

    # The widest point of the blank is the crown, and it must match d_a.
    m = geo.member(member)
    assert 2.0 * max(R for R, _ in outline) == pytest.approx(m.outside_dia, rel=1e-9)


@pytest.mark.parametrize("member", ["pinion", "gear"])
@pytest.mark.parametrize("sigma", [45.0, 60.0, 90.0])
@pytest.mark.parametrize("z1,z2", [(17, 43), (20, 20), (12, 60)])
def test_loft_sections_clear_the_blank(member, sigma, z1, z2):
    """Both loft sections must end clear of the blank, not touching it.

    This is the property that makes the cut buildable. A section that ends on
    a face of the blank - or worse, parallel and adjacent to one - is rejected
    by SOLIDWORKS as zero-thickness geometry.
    """
    g = compute_set(BevelSetParams.with_defaults(2.0, z1, z2, shaft_angle=sigma))
    m = g.member(member)
    over = end_overshoot(g, member)

    outer = tooth_space_section(g, member, "outer", overshoot=over)
    inner = tooth_space_section(g, member, "inner", overshoot=over)

    # The back is the root rim, which stands proud of the back cone; the flat
    # front is at the inner tip point. Clearing the cone is not enough - a
    # section between the cone and the rim would leave the rim uncut, bridging
    # the tooth spaces at the heel.
    assert min(
        beyond_back_cone(g, m, pt) for pt in outer.loop_3d()
    ) > blank_reach_past_back_cone(g, member)
    assert max(z for _, _, z in inner.loop_3d()) < front_face_z(g, m)

    # Nothing on the outer section overhangs the flat rim behind the back cone
    # either: the whole section sits radially outside the outer root radius.
    assert min(
        math.hypot(x, y) for x, y, _ in outer.loop_3d()
    ) > m.outer_root_radius


@pytest.mark.parametrize("member", ["pinion", "gear"])
def test_blank_back_face_follows_the_back_cone(geo, member):
    """The large end of the teeth must end on the back cone, not on a plane.

    That is what makes a meshed pair flush - see `test_tooth_tips_land_on_the_
    mates_back_cone`.
    """
    outline = blank_outline(geo, member)
    m = geo.member(member)

    # The crown is the widest point; the segment leaving it is the back cone,
    # running inward in R and *backward* in z.
    crown_i = max(range(len(outline)), key=lambda i: outline[i][0])
    crown = outline[crown_i]
    back = outline[crown_i + 1]
    assert crown == pytest.approx((m.outside_dia / 2.0, m.crown_to_apex), rel=1e-12)
    assert back == pytest.approx((m.outer_root_radius, m.root_to_apex), rel=1e-12)
    assert back[0] < crown[0]
    assert back[1] > crown[1]

    # It is perpendicular to the pitch cone, which is the defining property.
    edge = (back[0] - crown[0], back[1] - crown[1])
    pitch_dir = (math.sin(m.pitch_angle), math.cos(m.pitch_angle))
    assert edge[0] * pitch_dir[0] + edge[1] * pitch_dir[1] == pytest.approx(
        0.0, abs=1e-12
    )

    # Its length is the whole depth: crown to outer root point.
    assert math.hypot(*edge) == pytest.approx(m.addendum + m.dedendum, rel=1e-12)

    # Behind it comes the root rim - a cylinder at the same radius - and only
    # then does the blank go flat. The front face is flat too.
    rim = outline[crown_i + 2]
    assert rim[0] == pytest.approx(back[0], abs=1e-12)
    assert rim[1] - back[1] == pytest.approx(ANCHOR.min_root_thickness, abs=1e-12)
    assert outline[crown_i + 3][1] == pytest.approx(rim[1], abs=1e-12)
    assert outline[0][1] == pytest.approx(outline[1][1], abs=1e-12)


@pytest.mark.parametrize("member", ["pinion", "gear"])
@pytest.mark.parametrize("z1,z2", [(17, 43), (20, 20), (12, 60)])
def test_root_rim_keeps_material_under_the_whole_tooth(member, z1, z2):
    """The point of the rim: no radius under the teeth is thinner than it.

    Without one the flat back runs through the outer root point, so the rim
    thins linearly to zero at the heel - on the anchor gear it is under 1 mm for
    the last 2.1 mm of radius. The check walks the root cone over the face width
    and measures what is left between it and the flat back.
    """
    t = 0.8
    p = BevelSetParams.with_defaults(2.0, z1, z2, min_root_thickness=t)
    g = compute_set(p)
    m = g.member(member)
    outline = blank_outline(g, member)

    z_back = outline[4][1]
    assert outline[4][0] == pytest.approx(m.outer_root_radius, rel=1e-12)
    assert z_back - m.root_to_apex == pytest.approx(t, abs=1e-12)

    # Sample the root cone from toe to heel. The thinnest place is the heel, and
    # there it is exactly the rim.
    gaps = []
    for i in range(41):
        A = g.outer_cone_dist - p.face_width * i / 40
        r_root = (A / g.outer_cone_dist) * m.virtual_root_r
        R = r_root * math.cos(m.pitch_angle)
        z = A / math.cos(m.pitch_angle) - r_root * math.sin(m.pitch_angle)
        assert R <= m.outer_root_radius + 1e-9
        gaps.append(z_back - z)
    assert min(gaps) == pytest.approx(t, abs=1e-9)


@pytest.mark.parametrize("member", ["pinion", "gear"])
def test_zero_root_rim_restores_the_plain_flat_back(member):
    """Setting it to zero has to give back the old outline exactly, not a
    degenerate zero-length segment - which SOLIDWORKS would reject as a line."""
    p = BevelSetParams.with_defaults(2.0, 17, 43, min_root_thickness=0.0)
    g = compute_set(p)
    m = g.member(member)
    outline = blank_outline(g, member)

    assert outline[3] == pytest.approx((m.outer_root_radius, m.root_to_apex), rel=1e-12)
    assert outline[4][1] == pytest.approx(m.root_to_apex, abs=1e-12)
    assert blank_reach_past_back_cone(g, member) == pytest.approx(0.0, abs=1e-12)
    assert len({(round(R, 12), round(z, 12)) for R, z in outline}) == len(outline)


@pytest.mark.parametrize("member", ["pinion", "gear"])
@pytest.mark.parametrize("z1,z2", [(17, 43), (20, 20), (12, 60)])
def test_root_rim_reaches_past_the_back_cone_by_t_cos_delta(member, z1, z2):
    """How far the rim stands proud of the back cone, which is what the loft
    overshoot has to beat. The near corner is on the cone, so it is the far one
    that reaches, at t * cos(delta)."""
    t = 1.25
    g = compute_set(BevelSetParams.with_defaults(2.0, z1, z2, min_root_thickness=t))
    m = g.member(member)
    assert blank_reach_past_back_cone(g, member) == pytest.approx(
        t * math.cos(m.pitch_angle), rel=1e-9
    )


@pytest.mark.parametrize("sigma", [45.0, 60.0, 90.0, 120.0])
@pytest.mark.parametrize("z1,z2", [(17, 43), (20, 20), (12, 60)])
def test_tooth_tips_land_on_the_mates_back_cone(sigma, z1, z2):
    """The whole point of the back cone: a meshed pair ends flush.

    Both members' back cones are perpendicular to the *same* pitch generator at
    Ao, so in the meridian plane they are one and the same line. Walking that
    line from the outer pitch point, each member's crown and outer root point
    are at signed distances +a and -b_f, with the sign flipped between members
    because their radial directions oppose. The mate's crown must therefore fall
    strictly inside this member's back cone span - that is "flush, not
    overhanging".
    """
    g = compute_set(BevelSetParams.with_defaults(2.0, z1, z2, shaft_angle=sigma))
    d1 = g.pinion.pitch_angle
    sig = math.radians(sigma)

    # Shared meridian frame (x, h): the pinion keeps its own frame, so pinion
    # (R, z) -> (R, z). The gear is turned to the shaft angle and its meshing
    # side faces the pinion, which flips its radial direction - see mesh.py.
    def to_shared(member, R, z):
        if member == "pinion":
            return R, z
        return (
            z * math.sin(sig) - R * math.cos(sig),
            R * math.sin(sig) + z * math.cos(sig),
        )

    # Everything below is measured along the shared back cone generator: the
    # line through the outer pitch point, perpendicular to the pitch cone.
    p_out = (g.outer_cone_dist * math.sin(d1), g.outer_cone_dist * math.cos(d1))
    u = (math.cos(d1), -math.sin(d1))   # outward for the pinion along that line

    def t_of(member, R, z):
        x, h = to_shared(member, R, z)
        off = (x - p_out[0], h - p_out[1])
        # Must lie *on* the line, not merely near it.
        assert off[0] * u[1] - off[1] * u[0] == pytest.approx(0.0, abs=1e-9)
        return off[0] * u[0] + off[1] * u[1]

    spans = {}
    for name in ("pinion", "gear"):
        m = g.member(name)
        t_crown = t_of(name, m.outside_dia / 2.0, m.crown_to_apex)
        t_root = t_of(name, m.outer_root_radius, m.root_to_apex)
        # Each member's back cone spans its own whole depth, and the sign of
        # the crown says which way its radial direction points.
        assert abs(t_crown - t_root) == pytest.approx(
            m.addendum + m.dedendum, rel=1e-9
        )
        spans[name] = (min(t_crown, t_root), max(t_crown, t_root), t_crown)

    # The payoff: each member's crown sits strictly inside the *mate's* back
    # cone span, so its tooth tips end flush against the mate instead of
    # hanging past it.
    for name, mate in (("pinion", "gear"), ("gear", "pinion")):
        lo, hi, _ = spans[mate]
        assert lo < spans[name][2] < hi


@pytest.mark.parametrize("member", ["pinion", "gear"])
def test_section_cap_clears_the_blank(geo, member):
    """The cut profile must reach past the face cone, or it would leave a web."""
    outline = blank_outline(geo, member)
    max_blank_R = max(R for R, _ in outline)
    for end in ("outer", "inner"):
        s = tooth_space_section(geo, member, end)
        cap_R = max(
            math.hypot(x, y) for x, y, _ in
            [to_cone_3d(px, py, s.pitch_angle, s.cone_apex_z) for px, py in s.segments["cap"]]
        )
        assert cap_R > s.r_tip * math.cos(s.pitch_angle)
        if end == "outer":
            assert cap_R > max_blank_R
