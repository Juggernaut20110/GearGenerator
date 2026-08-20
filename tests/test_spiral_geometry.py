"""Closed-form checks on the spiral bevel tooth trace. No SOLIDWORKS involved.

The anchor spiral set is the straight anchor with a 35 degree mean spiral added:
m=2, 17 x 43 teeth, 20 deg pressure angle, 90 deg shafts. Its face width comes
out narrower than the straight set's - 13.87 mm against 15.41 - because a curved
tooth takes the tighter Gleason limit of 0.30*Ao.

The property this file exists for is `test_both_members_traces_coincide...`:
that is the meshing condition, and it is the one thing about the construction
that would be expensive to discover in SOLIDWORKS and costs nothing to assert
here.
"""

from __future__ import annotations

import math

import pytest

from gears.bevel.geometry import (
    MAX_SECTION_SAGITTA_MM,
    CrownTrace,
    compute_set,
    guide_spiral,
    phase_at_cone_distance,
    section_cone_distances,
    section_count,
    section_span,
    to_cone_3d,
    tooth_space_section,
)
from gears.bevel import mesh
from gears.bevel.params import BevelSetParams
from gears.bevel.validate import validate
from gears.placement import (
    apply,
    gear_clocking,
    pinion_placement,
)

ANCHOR_STRAIGHT = BevelSetParams.with_defaults(2.0, 17, 43)
ANCHOR_SPIRAL = BevelSetParams.with_defaults(2.0, 17, 43, spiral_angle=35.0)

# Zerol: the same set at zero mean spiral with a real cutter, which is a curved
# tooth crossing the mean cone distance radially rather than a straight one.
ANCHOR_ZEROL = BevelSetParams.with_defaults(2.0, 17, 43, cutter_radius=39.3035)

MEMBERS = ("pinion", "gear")


@pytest.fixture
def straight():
    return compute_set(ANCHOR_STRAIGHT)


@pytest.fixture
def spiral():
    return compute_set(ANCHOR_SPIRAL)


@pytest.fixture
def zerol():
    return compute_set(ANCHOR_ZEROL)


# --- the arc itself --------------------------------------------------------


@pytest.mark.parametrize("psi_deg", [10.0, 20.0, 35.0, 40.0])
@pytest.mark.parametrize("cutter_factor", [0.6, 1.0, 1.5, 3.0])
def test_the_arc_delivers_the_mean_spiral_angle_at_the_mean_cone_distance(
    psi_deg, cutter_factor
):
    """Whatever cutter you pick, the arc is placed to hit psi_m at Am.

    That is the one thing `CrownTrace.for_set` solves for, and it has to hold
    for every cutter radius - the cutter changes how the angle varies either
    side of the mean, not what it is at the mean.
    """
    am = 39.3035
    trace = CrownTrace.for_set(math.radians(psi_deg), cutter_factor * am, am)
    assert trace.spiral_angle_at(am) == pytest.approx(math.radians(psi_deg))


def test_the_trace_angle_is_zero_at_the_mean_cone_distance(spiral):
    """Am is the clocking reference, the way z = 0 is for a spur gear."""
    assert spiral.trace.theta_at(spiral.mean_cone_dist) == pytest.approx(0.0)
    for member in MEMBERS:
        assert phase_at_cone_distance(
            spiral, member, spiral.mean_cone_dist
        ) == pytest.approx(0.0)


def test_the_spiral_angle_grows_from_toe_to_heel(spiral):
    """A circular arc does not hold a constant spiral angle, and should not.

    Measured on the anchor spiral set: 30.074 at the toe, 35 at the mean,
    40.599 at the heel.
    """
    t = spiral.trace
    toe = math.degrees(t.spiral_angle_at(spiral.inner_cone_dist))
    mean = math.degrees(t.spiral_angle_at(spiral.mean_cone_dist))
    heel = math.degrees(t.spiral_angle_at(spiral.outer_cone_dist))

    assert toe == pytest.approx(30.074, abs=1e-3)
    assert mean == pytest.approx(35.0)
    assert heel == pytest.approx(40.599, abs=1e-3)
    assert toe < mean < heel


def test_a_larger_cutter_flattens_the_spiral_angle_across_the_face():
    """The limit of an infinite cutter is a straight tooth laid at an angle.

    So the swing from toe to heel has to shrink monotonically as the cutter
    grows - which is the whole reason cutter radius is an input rather than a
    constant.
    """
    swings = []
    for factor in (0.8, 1.0, 2.0, 5.0, 20.0):
        p = BevelSetParams.with_defaults(
            2.0, 17, 43, spiral_angle=35.0, cutter_radius=factor * 39.3035
        )
        geo = compute_set(p)
        t = geo.trace
        swings.append(
            abs(t.spiral_angle_at(geo.outer_cone_dist))
            - abs(t.spiral_angle_at(geo.inner_cone_dist))
        )
    assert swings == sorted(swings, reverse=True)


def test_the_arc_must_actually_span_the_face():
    """A circle only has points between |rho - r_c| and rho + r_c."""
    p = BevelSetParams.with_defaults(
        2.0, 17, 43, spiral_angle=35.0, cutter_radius=8.0
    )
    geo = compute_set(p)
    assert not geo.trace.reaches(geo.inner_cone_dist, geo.outer_cone_dist)

    result = validate(p)
    assert not result.ok
    assert any(issue.field == "cutter_radius" for issue in result.errors)


# --- the meshing property --------------------------------------------------


def placed_trace_point(geo, member, cone_dist, flip_gear=False):
    """Where a member's tooth trace sits **in the assembly**, at `cone_dist`.

    The pitch-cone point at the trace's own phase, put through the same
    placement transform `sw/bevel_assembly.py` writes. `flip_gear` undoes the
    negation `phase_at_cone_distance` applies to the gear, so a test can show
    what the geometry looked like before it was corrected.
    """
    m = geo.member(member)
    delta = m.pitch_angle
    phase = phase_at_cone_distance(geo, member, cone_dist)
    if member == "gear":
        if flip_gear:
            phase = -phase
        # The contact line lies at pi around the gear's axis, not 0 - see the
        # module docstring of `gears/bevel/mesh.py`.
        phase += math.pi + gear_clocking(m.z)
        matrix = mesh.gear_placement(geo.params.sigma, 0.0)
    else:
        matrix = pinion_placement()
    return apply(matrix, (
        cone_dist * math.sin(delta) * math.cos(phase),
        cone_dist * math.sin(delta) * math.sin(phase),
        cone_dist * math.cos(delta),
    ))


def worst_trace_gap(geo, flip_gear=False, samples=101):
    """How far apart the two placed traces get across the face. mm."""
    worst = 0.0
    for i in range(samples):
        A = geo.inner_cone_dist + (
            geo.outer_cone_dist - geo.inner_cone_dist
        ) * i / (samples - 1)
        worst = max(worst, math.dist(
            placed_trace_point(geo, "pinion", A),
            placed_trace_point(geo, "gear", A, flip_gear),
        ))
    return worst


def test_both_members_traces_coincide_along_the_common_pitch_generator(spiral):
    """The meshing condition, asserted where it actually has to hold: in space.

    This test used to compare the two arc lengths - `R * phase = A * theta_c` for
    either member - and assert they were **equal**. They are, and it proved
    nothing: an arc length is a magnitude, and two displacements of equal size
    can point opposite ways. They did. The pair was built with both traces
    curving the same way about their own axes, which at the shared pitch
    generator puts them on opposite sides of it, and no test in this file could
    see it because they all compared each member against itself.

    So place both traces and measure the distance between them. What the pinion
    and the gear have to agree about is a **point**, not a length.
    """
    assert worst_trace_gap(spiral) < 1.5


def test_the_traces_would_diverge_if_the_gear_were_not_negated(spiral):
    """The paired failure, because a check that cannot fail proves nothing.

    Undo the negation and the two traces pull apart to 11.53 mm on the anchor
    set - agreeing only at the mean cone distance, where the phase is zero by
    construction and every section is the clocking reference. That is exactly
    why this went unnoticed: the one place the old tests looked was the one
    place that was right.

    Built in SOLIDWORKS the same difference reads as 103.507 mm3 of interference
    in 17 regions, against 0.063 mm3 in 5 once negated.
    """
    assert worst_trace_gap(spiral, flip_gear=True) == pytest.approx(11.53, abs=0.05)
    assert worst_trace_gap(spiral) < worst_trace_gap(spiral, flip_gear=True) / 10.0

    # Agreeing at the mean either way is the blind spot itself, stated outright.
    for flip in (False, True):
        assert math.dist(
            placed_trace_point(spiral, "pinion", spiral.mean_cone_dist),
            placed_trace_point(spiral, "gear", spiral.mean_cone_dist, flip),
        ) == pytest.approx(0.0, abs=1e-9)


def test_a_zerol_pair_meshes_too(zerol):
    """The same property on the set whose phases are smallest.

    A Zerol trace barely turns - 0.898 degrees of sweep against the 35 degree
    set's 38.790 - so its divergence is small in absolute terms and was the
    easiest to dismiss. 1.2353 mm unnegated, 0.0172 mm negated; in SOLIDWORKS,
    27.461 mm3 against 0.034.
    """
    assert worst_trace_gap(zerol) < 0.05
    assert worst_trace_gap(zerol, flip_gear=True) == pytest.approx(1.235, abs=0.01)


def test_a_straight_pair_has_no_trace_to_get_wrong(straight):
    """Zero phase everywhere, so the two traces are the pitch generator itself."""
    assert worst_trace_gap(straight) == pytest.approx(0.0, abs=1e-9)


def test_the_arc_length_travelled_is_the_crown_trace_angle_times_the_cone_distance(
    spiral,
):
    """The identity behind the test above, spelled out on its own.

    Worth having separately: the meshing test would still pass if both members
    were wrong in the same way, and this one says what the right answer is.

    Stated as a **magnitude**, and the qualifier is the point. Both members
    travel the same arc length `A * theta_c` off the one crown arc, which is
    what makes the two traces the same curve - but they travel it in opposite
    senses about their own axes, because they roll on the crown in opposite
    senses. Asserting this identity *with* its sign is precisely the mistake
    that hid a 103.507 mm3 interference: it is true of a magnitude and says
    nothing about a direction.
    """
    for A in (spiral.inner_cone_dist, spiral.mean_cone_dist, spiral.outer_cone_dist):
        expected = abs(A * spiral.trace.theta_at(A))
        for member in MEMBERS:
            m = spiral.member(member)
            R = A * math.sin(m.pitch_angle)
            assert abs(R * phase_at_cone_distance(spiral, member, A)) == (
                pytest.approx(expected)
            )

    # And the senses really do oppose, which is the half the magnitude drops.
    A = spiral.outer_cone_dist
    signs = [
        math.copysign(1.0, phase_at_cone_distance(spiral, m, A)) for m in MEMBERS
    ]
    assert signs[0] == -signs[1]


def test_the_two_members_sweep_different_amounts(spiral):
    """Same arc, different pitch angles - so the sweeps are not equal.

    The pinion's smaller pitch angle divides by a smaller sine and sweeps
    further. Measured: 38.790 deg on the 17-tooth pinion, 15.336 on the 43-tooth
    gear. This is the bevel echo of the spur pair's unequal twists.
    """
    sweeps = {
        member: math.degrees(
            phase_at_cone_distance(spiral, member, spiral.outer_cone_dist)
            - phase_at_cone_distance(spiral, member, spiral.inner_cone_dist)
        )
        for member in MEMBERS
    }
    assert abs(sweeps["pinion"]) == pytest.approx(38.790, abs=1e-3)
    assert abs(sweeps["gear"]) == pytest.approx(15.336, abs=1e-3)


def test_the_hand_flips_every_phase_and_nothing_else():
    """Left hand is right hand mirrored, with the profile untouched."""
    right = compute_set(
        BevelSetParams.with_defaults(2.0, 17, 43, spiral_angle=35.0, hand="right")
    )
    left = compute_set(
        BevelSetParams.with_defaults(2.0, 17, 43, spiral_angle=35.0, hand="left")
    )

    for member in MEMBERS:
        for A in (left.inner_cone_dist, left.mean_cone_dist, left.outer_cone_dist):
            assert phase_at_cone_distance(left, member, A) == pytest.approx(
                -phase_at_cone_distance(right, member, A)
            )
        # The section itself is the same shape either way.
        a = tooth_space_section(right, member, cone_dist=right.mean_cone_dist)
        b = tooth_space_section(left, member, cone_dist=left.mean_cone_dist)
        assert a.loop_2d == pytest.approx(b.loop_2d)


# --- straight bevel is untouched -------------------------------------------


def test_a_straight_set_has_no_trace_and_no_phase(straight):
    """The regression guard on everything this feature did not mean to change."""
    assert straight.trace is None
    assert straight.face_contact_ratio == 0.0
    for member in MEMBERS:
        for end in ("inner", "outer"):
            section = tooth_space_section(straight, member, end)
            assert section.phase == 0.0


def test_a_straight_set_still_lofts_from_exactly_two_sections(straight):
    for member in MEMBERS:
        assert section_count(straight, member) == 2
        assert len(section_cone_distances(straight, member)) == 2


def test_a_zero_phase_section_maps_exactly_as_it_did_before(straight):
    """`to_cone_3d`'s new phase argument must be inert at its default.

    Asserted against the mapping written out longhand rather than against the
    function itself, so this catches the phase being applied in the wrong place
    as well as being applied at all.
    """
    delta, apex_z = 0.3765, 21.4
    for x, y in ((5.0, 0.0), (12.5, 3.25), (-4.0, 9.0), (0.0, -7.5)):
        r = math.hypot(x, y)
        phi = math.atan2(y, x)
        R = r * math.cos(delta)
        theta = phi / math.cos(delta)
        expected = (
            R * math.cos(theta),
            R * math.sin(theta),
            apex_z - r * math.sin(delta),
        )
        assert to_cone_3d(x, y, delta, apex_z) == pytest.approx(expected)


# --- loft sections ---------------------------------------------------------


def worst_chord_deviation(geo, member, distances, samples=200):
    """How far the loft's chords leave the swept surface, solved longhand. mm.

    Deliberately not `geometry._chord_deviation`: this samples far more finely
    and builds the tip point from scratch, so it is an independent statement of
    the property rather than the implementation checked against itself.

    Note what it does *not* assume - that the phase runs monotonically between
    two sections. Reading the deviation off the two endpoint phases as
    `r * (1 - cos(delta / 2))` is exactly the shortcut that let a Zerol tooth
    through at 1.3833 mm; see `test_a_zerol_loft_follows_the_bow_it_turns_at_the_mean`.
    """
    r = geo.member(member).outside_dia / 2.0

    def tip(A):
        a = phase_at_cone_distance(geo, member, A)
        return (r * math.cos(a), r * math.sin(a), A)

    worst = 0.0
    for a, b in zip(distances, distances[1:]):
        if max(a, b) < geo.inner_cone_dist or min(a, b) > geo.outer_cone_dist:
            continue        # outside the blank, so it cuts nothing
        pa, pb = tip(a), tip(b)
        for i in range(1, samples):
            f = i / samples
            chord = tuple(pa[k] + f * (pb[k] - pa[k]) for k in range(3))
            worst = max(worst, math.dist(chord, tip(a + (b - a) * f)))
    return worst


def test_the_section_count_keeps_the_chord_inside_tolerance(spiral):
    """The reason there are more than two sections, checked at the tip.

    Each loft interval carries the profile along a straight chord while the true
    trace curves away from it. Checked on the sections `section_cone_distances`
    actually returns, over every interval that reaches the blank - which is the
    guarantee it makes.

    This test is why the count is measured rather than solved. The closed form
    sized it over the face width while the loft runs over the face *plus* two
    overshoots, and the trace turns fastest past the toe, so the interval down
    there came out at 0.0293 mm against a 0.02 mm tolerance.
    """
    for member in MEMBERS:
        distances = section_cone_distances(spiral, member)
        checked = [
            (a, b) for a, b in zip(distances, distances[1:])
            if not (max(a, b) < spiral.inner_cone_dist
                    or min(a, b) > spiral.outer_cone_dist)
        ]
        assert len(checked) >= 2, "the face should span several loft intervals"
        assert worst_chord_deviation(spiral, member, distances) <= MAX_SECTION_SAGITTA_MM


def test_the_spiral_anchor_takes_the_section_counts_that_were_computed(spiral):
    """Recorded so a change in the tolerance or the trace shows up as a number.

    The two land one apart, and the near-miss is a coincidence of two effects
    nearly cancelling: the pinion sweeps 2.5 times as far, and the gear's tip
    stands 2.3 times further from the axis, and the deviation goes with each.
    """
    assert section_count(spiral, "pinion") == 11
    assert section_count(spiral, "gear") == 12


def test_a_straight_set_still_takes_exactly_two_sections():
    """The measure changed; this number must not. A straight tooth is a prism."""
    geo = compute_set(ANCHOR_STRAIGHT)
    assert [section_count(geo, m) for m in MEMBERS] == [2, 2]


def test_the_sections_run_from_beyond_the_heel_to_inside_the_toe(spiral):
    """Both ends pushed past the blank, so no cut finishes tangent to a face."""
    for member in MEMBERS:
        distances = section_cone_distances(spiral, member)
        assert distances[0] > spiral.outer_cone_dist
        assert distances[-1] < spiral.inner_cone_dist
        # Monotonic, heel first - the order the loft profiles are selected in.
        assert distances == sorted(distances, reverse=True)


def test_a_harder_spiral_needs_more_sections():
    """Above the mid range, where sweep is the only thing growing.

    Restricted to 25 degrees and up on purpose - see
    `test_the_section_count_is_worst_at_both_ends_of_the_spiral_range` for why
    the whole range is not monotonic and why asserting that it was hid a bug.
    """
    counts = []
    for psi in (25.0, 35.0, 44.0):
        geo = compute_set(
            BevelSetParams.with_defaults(2.0, 17, 43, spiral_angle=psi)
        )
        counts.append(section_count(geo, "pinion"))
    assert counts == sorted(counts)
    assert counts[0] < counts[-1]


def test_the_section_count_is_worst_at_both_ends_of_the_spiral_range():
    """Two effects, pulling opposite ways, with a minimum between them.

    This test replaces one that asserted the count rose monotonically with the
    spiral angle, which was not a property of the geometry but an artifact of
    the measure: the old one read each interval's error off its two endpoint
    phases, so it could not see a trace that turned around *inside* the
    interval, and a gentle spiral is exactly where that happens.

    A trace's spiral angle passes through zero at cone distance
    `sqrt(rho^2 - r_c^2)`, and with the nominal cutter that lands inside the
    face for every mean angle below about 9.3 degrees on this set - so a 5
    degree spiral has a bow to chord across just as a Zerol one does, on top of
    which the hard spirals need chords for their sweep. Measured on the anchor
    17x43 pinion at 0.02 mm:

        psi     0     5     9    10    15    25    35    44
        count  12    12    11    10     9     9    11    15

    The minimum sits in the middle, which is the useful thing to know: neither
    end of the range is the cheap one to build.
    """
    counts = {
        psi: section_count(
            compute_set(BevelSetParams.with_defaults(2.0, 17, 43, spiral_angle=psi)),
            "pinion",
        )
        for psi in (5.0, 15.0, 25.0, 44.0)
    }
    assert counts[5.0] > counts[15.0]      # the turning trace, not the sweep
    assert counts[44.0] > counts[25.0]     # the sweep, not the turning trace
    assert min(counts.values()) not in (counts[5.0], counts[44.0])


def test_the_spiral_angle_crosses_zero_inside_a_gentle_spirals_face():
    """The reason a gentle spiral costs more sections than a moderate one.

    Stated on the trace rather than on the count, so it says *why* rather than
    restating the measurement. The crossing leaves the face at about 9.3 degrees
    mean on this set, which is the shoulder in the table above.
    """
    for psi, crosses in ((5.0, True), (9.0, True), (10.0, False), (35.0, False)):
        geo = compute_set(
            BevelSetParams.with_defaults(2.0, 17, 43, spiral_angle=psi)
        )
        inside = [
            geo.trace.spiral_angle_at(
                geo.inner_cone_dist
                + (geo.outer_cone_dist - geo.inner_cone_dist) * i / 200.0
            )
            for i in range(201)
        ]
        assert (min(inside) < 0.0 < max(inside)) is crosses


# --- the guide curve -------------------------------------------------------


def test_the_guide_spiral_touches_the_cap_vertex_of_every_section(spiral):
    """A guided loft fails when its guide only passes *near* a profile.

    The cap is split at its centreline so a real vertex exists there in every
    section; the guide is the path of that vertex. Both are built from the same
    `r_cap` and the same phase, so they must agree to the last bit - and this is
    much cheaper to assert here than to discover as a failed feature in
    SOLIDWORKS.
    """
    for member in MEMBERS:
        distances = section_cone_distances(spiral, member)
        guide = guide_spiral(spiral, member, distances[0], distances[-1], points=61)

        for A in distances:
            section = tooth_space_section(
                spiral, member, cone_dist=A, split_cap=True
            )
            segments = section.segments_3d()
            # The vertex shared by the two cap halves.
            vertex = segments["cap_neg"][-1]
            assert vertex == pytest.approx(segments["cap_pos"][0])

            expected = to_cone_3d(
                section.r_cap, 0.0, section.pitch_angle,
                section.cone_apex_z, section.phase,
            )
            assert vertex == pytest.approx(expected)

        # The guide spans the sections rather than stopping short of them.
        assert guide[0] == pytest.approx(
            to_cone_3d(
                tooth_space_section(spiral, member, cone_dist=distances[0]).r_cap,
                0.0,
                spiral.member(member).pitch_angle,
                distances[0] / math.cos(spiral.member(member).pitch_angle),
                phase_at_cone_distance(spiral, member, distances[0]),
            )
        )


def test_the_guide_spiral_radius_shrinks_toward_the_toe(spiral):
    """Unlike a helix, this guide is not at constant radius.

    The cap radius scales with the cone distance, so the guide is a conical
    spiral rather than a cylindrical one - which is why it is sampled and
    splined rather than handed to any helix feature.
    """
    for member in MEMBERS:
        distances = section_cone_distances(spiral, member)
        guide = guide_spiral(spiral, member, distances[0], distances[-1], points=31)
        radii = [math.hypot(x, y) for x, y, _ in guide]
        assert radii == sorted(radii, reverse=True)


# --- the face contact ratio ------------------------------------------------


def test_the_face_contact_ratio_is_the_face_advance_over_the_mean_pitch(spiral):
    p_m = spiral.circular_pitch * spiral.mean_cone_dist / spiral.outer_cone_dist
    expected = (
        spiral.params.face_width * math.tan(math.radians(35.0)) / p_m
    )
    assert spiral.face_contact_ratio == pytest.approx(expected)
    assert spiral.face_contact_ratio == pytest.approx(1.818, abs=1e-3)


def test_a_harder_spiral_buys_more_face_overlap():
    ratios = []
    for psi in (0.0, 10.0, 20.0, 30.0, 40.0):
        geo = compute_set(
            BevelSetParams.with_defaults(2.0, 17, 43, spiral_angle=psi)
        )
        ratios.append(geo.face_contact_ratio)
    assert ratios[0] == 0.0
    assert ratios == sorted(ratios)


# --- Zerol -----------------------------------------------------------------


def test_a_zerol_set_has_a_curved_tooth_at_zero_mean_spiral_angle():
    """Zero spiral angle, finite cutter: curved, and radial only at the mean.

    The distinction `is_curved` exists to make. A Zerol gear is not a straight
    gear, and sizing or building it as one would throw away the curvature that
    is the whole point of it.
    """
    p = BevelSetParams.with_defaults(2.0, 17, 43, cutter_radius=39.3035)
    assert p.spiral_angle == 0.0
    assert p.is_curved

    geo = compute_set(p)
    assert geo.trace is not None
    assert geo.trace.spiral_angle_at(geo.mean_cone_dist) == pytest.approx(0.0)
    # Radial at the mean, curving away either side of it.
    assert phase_at_cone_distance(geo, "pinion", geo.mean_cone_dist) == pytest.approx(
        0.0
    )
    assert abs(phase_at_cone_distance(geo, "pinion", geo.outer_cone_dist)) > 1e-6
    assert abs(phase_at_cone_distance(geo, "pinion", geo.inner_cone_dist)) > 1e-6


def test_a_plain_straight_set_is_not_treated_as_curved():
    assert not ANCHOR_STRAIGHT.is_curved
    assert ANCHOR_STRAIGHT.cutter_radius is None


def test_a_zerol_loft_follows_the_bow_it_turns_at_the_mean():
    """The regression for a Zerol tooth that lofted almost straight.

    The section count used to read each interval's error off its two *endpoint*
    phases, as `r * (1 - cos(delta / 2))`. That is the sagitta of a chord against
    its arc and it is right only while the phase runs monotonically across the
    interval - and a Zerol trace turns around at the mean cone distance, which is
    the definition of Zerol rather than an edge case. So both ends of the face
    sat on the same side, their difference was 0.898 degrees, the measure saw
    nothing and stopped at two sections, and the chord those two sections build
    stood 1.3833 mm off the trace against a 0.02 mm tolerance.
    """
    geo = compute_set(ANCHOR_ZEROL)
    for member in MEMBERS:
        distances = section_cone_distances(geo, member)
        assert worst_chord_deviation(geo, member, distances) <= MAX_SECTION_SAGITTA_MM


def test_two_sections_are_nowhere_near_enough_for_a_zerol_tooth():
    """The paired failure, because a check that cannot fail proves nothing.

    Same measure, same set, the section count the old code chose - and it has to
    come out far outside tolerance, or the test above is passing for free.
    """
    geo = compute_set(ANCHOR_ZEROL)
    a_hi, a_lo = section_span(geo, "pinion")
    deviation = worst_chord_deviation(geo, "pinion", [a_hi, a_lo])
    assert deviation == pytest.approx(1.3833, abs=1e-3)
    assert deviation > 50.0 * MAX_SECTION_SAGITTA_MM


def test_the_zerol_anchor_takes_the_section_counts_that_were_computed():
    """Recorded, and the ordering in them is the surprising part.

    The Zerol set sweeps least of the three anchors - 0.898 degrees on the pinion
    against the 35 degree set's 38.790 - and needs the most sections. Sweep sizes
    the error of a chord cutting a corner; it says nothing about a chord laid
    straight across a bow, which is the whole of a Zerol trace's error.
    """
    geo = compute_set(ANCHOR_ZEROL)
    assert section_count(geo, "pinion") == 12
    assert section_count(geo, "gear") == 20


def test_a_zerol_tooth_takes_its_bow_from_the_hand():
    """`hand` is the only input left saying anything at a zero spiral angle.

    It used to say nothing: the trace's sign was read off `psi_m`, which is 0.0
    for both hands - and `-0.0 < 0.0` is False - so every Zerol tooth bowed the
    same way whatever was asked for.
    """
    right = compute_set(
        BevelSetParams.with_defaults(2.0, 17, 43, cutter_radius=39.3035, hand="right")
    )
    left = compute_set(
        BevelSetParams.with_defaults(2.0, 17, 43, cutter_radius=39.3035, hand="left")
    )
    assert right.trace.sign == -left.trace.sign

    for member in MEMBERS:
        for A in (right.inner_cone_dist, right.mean_cone_dist, right.outer_cone_dist):
            assert phase_at_cone_distance(right, member, A) == pytest.approx(
                -phase_at_cone_distance(left, member, A), abs=1e-12
            )
    # The bow is real in both, not merely mirrored nothing.
    assert abs(phase_at_cone_distance(right, "pinion", right.inner_cone_dist)) > 1e-3


def test_both_hands_of_a_zerol_set_still_mesh():
    """Flipping the hand moves both members together, so the mesh survives it.

    Asserted on the placed traces rather than on an arc length, for the reason
    `test_both_members_traces_coincide_along_the_common_pitch_generator` gives:
    a length cannot tell you which way it points, and which way it points was
    the bug.
    """
    for hand in ("right", "left"):
        geo = compute_set(
            BevelSetParams.with_defaults(
                2.0, 17, 43, cutter_radius=39.3035, hand=hand
            )
        )
        assert worst_trace_gap(geo) < 0.05


def test_a_zerol_set_has_no_face_contact_ratio():
    """Zero, and correct rather than a bug worth fixing.

    A Zerol tooth has no lengthwise overlap - the contact does not travel along
    the face the way a spiral's does - which is exactly the property that makes
    it a straight-bevel substitute that throws no axial thrust at the bearings.
    """
    geo = compute_set(ANCHOR_ZEROL)
    assert geo.face_contact_ratio == pytest.approx(0.0)
    assert compute_set(ANCHOR_SPIRAL).face_contact_ratio > 1.5
