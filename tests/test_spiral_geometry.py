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
    to_cone_3d,
    tooth_space_section,
)
from gears.bevel.params import BevelSetParams
from gears.bevel.validate import validate

ANCHOR_STRAIGHT = BevelSetParams.with_defaults(2.0, 17, 43)
ANCHOR_SPIRAL = BevelSetParams.with_defaults(2.0, 17, 43, spiral_angle=35.0)

MEMBERS = ("pinion", "gear")


@pytest.fixture
def straight():
    return compute_set(ANCHOR_STRAIGHT)


@pytest.fixture
def spiral():
    return compute_set(ANCHOR_SPIRAL)


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


def test_both_members_traces_coincide_along_the_common_pitch_generator(spiral):
    """The meshing condition, asserted directly.

    Both members map the *same* crown arc, so the arc length their traces travel
    along the pitch circle must agree at every cone distance:

        R * phase = (A sin delta) * (theta_c / sin delta) = A * theta_c

    Every pitch-angle term cancels. If this ever fails, the two traces have
    stopped being the same curve on the shared pitch plane and the pair will not
    mesh however well each member is built.
    """
    for i in range(21):
        A = spiral.inner_cone_dist + (
            spiral.outer_cone_dist - spiral.inner_cone_dist
        ) * i / 20.0
        travel = []
        for member in MEMBERS:
            m = spiral.member(member)
            R = A * math.sin(m.pitch_angle)
            travel.append(R * phase_at_cone_distance(spiral, member, A))
        assert travel[0] == pytest.approx(travel[1], abs=1e-12)


def test_the_arc_length_travelled_is_the_crown_trace_angle_times_the_cone_distance(
    spiral,
):
    """The identity behind the test above, spelled out on its own.

    Worth having separately: the meshing test would still pass if both members
    were wrong in the same way, and this one says what the right answer is.
    """
    for A in (spiral.inner_cone_dist, spiral.mean_cone_dist, spiral.outer_cone_dist):
        expected = A * spiral.trace.sign * spiral.trace.theta_at(A)
        for member in MEMBERS:
            m = spiral.member(member)
            R = A * math.sin(m.pitch_angle)
            assert R * phase_at_cone_distance(spiral, member, A) == pytest.approx(
                expected
            )


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


def test_the_section_count_keeps_the_chord_sagitta_inside_tolerance(spiral):
    """The reason there are more than two sections, checked at the tip.

    Each loft interval chords the profile through its own rotation, and the
    chord falls r * (1 - cos(delta / 2)) inside the true swept surface. Checked
    on the sections `section_cone_distances` actually returns, over every
    interval that reaches the blank - which is the guarantee it makes.

    This test is why the count is measured rather than solved. The closed form
    sized it over the face width while the loft runs over the face *plus* two
    overshoots, and the trace turns fastest past the toe, so the interval down
    there came out at 0.0293 mm against a 0.02 mm tolerance.
    """
    for member in MEMBERS:
        m = spiral.member(member)
        r_tip_axis = m.outside_dia / 2.0
        distances = section_cone_distances(spiral, member)
        phases = [phase_at_cone_distance(spiral, member, A) for A in distances]

        checked = 0
        for (a, b), (lo, hi) in zip(
            zip(distances, distances[1:]), zip(phases, phases[1:])
        ):
            if max(a, b) < spiral.inner_cone_dist or min(a, b) > spiral.outer_cone_dist:
                continue        # outside the blank, so it cuts nothing
            checked += 1
            sagitta = r_tip_axis * (1.0 - math.cos(abs(hi - lo) / 2.0))
            assert sagitta <= MAX_SECTION_SAGITTA_MM
        assert checked >= 2, "the face should span several loft intervals"


def test_the_spiral_anchor_takes_the_section_counts_that_were_computed(spiral):
    """Recorded so a change in the tolerance or the trace shows up as a number.

    Both members land on 11, which is a coincidence worth pinning: the pinion
    sweeps 2.5 times as far, and the gear's tip stands 2.3 times further from
    the axis, and the sagitta is proportional to each.
    """
    assert section_count(spiral, "pinion") == 11
    assert section_count(spiral, "gear") == 11


def test_the_sections_run_from_beyond_the_heel_to_inside_the_toe(spiral):
    """Both ends pushed past the blank, so no cut finishes tangent to a face."""
    for member in MEMBERS:
        distances = section_cone_distances(spiral, member)
        assert distances[0] > spiral.outer_cone_dist
        assert distances[-1] < spiral.inner_cone_dist
        # Monotonic, heel first - the order the loft profiles are selected in.
        assert distances == sorted(distances, reverse=True)


def test_a_harder_spiral_needs_more_sections():
    counts = []
    for psi in (5.0, 15.0, 25.0, 35.0, 44.0):
        geo = compute_set(
            BevelSetParams.with_defaults(2.0, 17, 43, spiral_angle=psi)
        )
        counts.append(section_count(geo, "pinion"))
    assert counts == sorted(counts)
    assert counts[0] < counts[-1]


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
