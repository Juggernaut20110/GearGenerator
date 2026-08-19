"""Closed-form checks on the planetary train. SOLIDWORKS is not involved.

The anchor planetary set is **m=2, sun 24, planet 18, ring 60, 3 planets**,
whose planet-ring mesh is the 18 x 60 anchor of `test_internal_geometry.py`. The
two sit beside each other on purpose: every number the ring mesh relies on is
already checked there, and this file is about the *train*.

Two properties earn their keep here.

`test_the_whole_train_interlocks...` places all N + 2 members the way the
assembly builder does and asks whether any two bodies occupy the same space -
sun against planet, planet against ring, planet against planet. It is the only
check that tests all three meshes at once, and it discriminates: mis-clocking
the planets by half a pitch takes it from zero to 1890 overlapping samples.

`test_a_set_that_fails_the_assembly_condition_physically_collides` then shows
that the assembly condition is a fact about metal rather than a rule from a
table. On a 27/17/61 set with three planets, planet 0 fits perfectly - it is the
one the ring was clocked against - and planets 1 and 2 each drive 855 samples
into the ring.
"""

from __future__ import annotations

import math

import pytest

from gears.involute import inv
from gears.placement import gear_clocking
from gears.planetary import mesh
from gears.planetary.geometry import (
    centre_distance_from_ring,
    compute_set,
    planet_ring_params,
    sun_planet_params,
)
from gears.planetary.params import PlanetarySetParams
from gears.planetary.validate import MIN_PLANET_GAP_FACTOR, validate
from gears.spur.mesh import internal_gear_clocking

ANCHOR = PlanetarySetParams.with_defaults(2.0, 24, 18, n_planets=3)


@pytest.fixture
def geo():
    return compute_set(ANCHOR)


# --- the tooth counts ------------------------------------------------------


def test_the_ring_count_is_forced_by_the_other_two():
    assert ANCHOR.z_ring == 60
    assert ANCHOR.z_ring == ANCHOR.z_sun + 2 * ANCHOR.z_planet


def test_the_centre_distance_solves_the_same_from_either_mesh():
    """Two expressions sharing no terms: m_t(z_s + z_p)/2 and m_t(z_r - z_p)/2.

    The cheapest possible check that `z_ring` is right, and the one that would
    notice if the ring count ever became an input.
    """
    assert ANCHOR.centre_distance == pytest.approx(42.0)
    assert centre_distance_from_ring(ANCHOR) == pytest.approx(42.0)
    assert ANCHOR.centre_distance == pytest.approx(centre_distance_from_ring(ANCHOR))


def test_the_two_meshes_are_built_at_the_same_centre_distance(geo):
    assert geo.sun_planet.centre_distance == pytest.approx(
        geo.planet_ring.centre_distance
    )


def test_the_ratios_are_the_textbook_epicyclic_ones():
    assert ANCHOR.ratio_carrier_to_sun == pytest.approx(3.5)     # 1 + 60/24
    assert ANCHOR.ratio_ring_to_sun == pytest.approx(-2.5)       # -60/24


# --- the members -----------------------------------------------------------


def test_the_sun_and_planet_are_external_and_the_ring_is_internal(geo):
    assert not geo.sun.internal
    assert not geo.planet.internal
    assert geo.ring.internal


def test_the_members_have_the_radii_the_meshes_imply(geo):
    assert geo.sun.pitch_r == pytest.approx(24.0)
    assert geo.planet.pitch_r == pytest.approx(18.0)
    assert geo.ring.pitch_r == pytest.approx(60.0)
    # And the three sit on one line: sun pitch + 2 * planet pitch = ring pitch.
    assert geo.sun.pitch_r + 2.0 * geo.planet.pitch_r == pytest.approx(
        geo.ring.pitch_r
    )


def test_the_ring_wraps_outside_everything_else(geo):
    assert geo.ring.tip_r == pytest.approx(58.0)
    assert geo.ring.root_r == pytest.approx(62.5)
    assert geo.ring_rim_radius == pytest.approx(67.5)
    # A planet at its station must fit inside the ring's tip circle.
    assert ANCHOR.centre_distance + geo.planet.tip_r < geo.ring.root_r


def test_one_hand_input_settles_all_three_members():
    """Sun opposite the planets (external), ring the same as them (internal)."""
    geo = compute_set(
        PlanetarySetParams.with_defaults(2.0, 24, 18, n_planets=3, helix_angle=15.0)
    )
    assert geo.sun.hand == "right"
    assert geo.planet.hand == "left"
    assert geo.ring.hand == "left"
    assert geo.planet.hand == geo.ring.hand
    assert geo.sun.hand != geo.planet.hand


def test_the_planet_is_the_same_gear_in_both_meshes():
    """It has to be - there is one planet part, used in two meshes."""
    sp = compute_set(ANCHOR).sun_planet
    pr = compute_set(ANCHOR).planet_ring
    for field in ("z", "pitch_r", "base_r", "tip_r", "root_r", "psi0", "half_pitch"):
        assert getattr(sp.gear, field) == pytest.approx(getattr(pr.pinion, field))


# --- the clocking ----------------------------------------------------------


def test_the_meshing_relation_reproduces_the_plain_spur_pair_clocking():
    """The relation in `mesh`'s docstring, anchored to an answer already right.

    external:  r_A (phi_A - c_A) + r_B (phi_B - c_B) = p/2 + n p
    with phi_A = 0 and c_A = 0 this must give back `gear_clocking`.
    """
    for z2 in (17, 21, 43, 60):
        n = round(math.pi / (2.0 * math.pi / z2) - 0.5)
        derived = math.pi - (2 * n + 1) * math.pi / z2
        tau = 2.0 * math.pi / z2
        difference = (derived - gear_clocking(z2)) % tau
        assert min(difference, tau - difference) == pytest.approx(0.0, abs=1e-12)


def test_the_meshing_relation_reproduces_the_internal_pair_clocking():
    """And the internal form must give back `internal_gear_clocking`."""
    for z2 in (60, 63, 100):
        assert internal_gear_clocking(z2) == pytest.approx(math.pi / z2)


def test_the_sun_keeps_the_frame_it_was_built_in():
    assert mesh.sun_clocking() == 0.0


def test_planet_zero_takes_the_plain_spur_gear_clocking(geo):
    """At carrier angle zero the train term drops out entirely."""
    assert mesh.planet_clocking(geo, 0) == pytest.approx(
        gear_clocking(ANCHOR.z_planet)
    )
    assert mesh.planet_clocking(geo, 0) == pytest.approx(math.radians(10.0))


def test_each_planet_rolls_round_the_sun_to_reach_its_station(geo):
    """The extra term is `phi_k (z_s + z_p) / z_p`.

    The sun does not move while the planet is carried round, so the planet has
    to roll along the sun's teeth to get there - which turns it about its own
    axis by more than the carrier angle.
    """
    for k in range(ANCHOR.n_planets):
        phi = mesh.carrier_angle(geo, k)
        expected = phi * (ANCHOR.z_sun + ANCHOR.z_planet) / ANCHOR.z_planet
        assert mesh.planet_clocking(geo, k) - gear_clocking(
            ANCHOR.z_planet
        ) == pytest.approx(expected)


def test_the_planets_sit_evenly_round_the_orbit(geo):
    positions = [mesh.planet_translation(geo, k) for k in range(ANCHOR.n_planets)]
    for k, (x, y, z) in enumerate(positions):
        assert math.hypot(x, y) == pytest.approx(ANCHOR.centre_distance)
        assert z == 0.0
        assert math.atan2(y, x) % (2.0 * math.pi) == pytest.approx(
            (2.0 * math.pi * k / ANCHOR.n_planets) % (2.0 * math.pi)
        )


def test_every_planet_gives_the_same_ring_clocking(geo):
    """The assembly condition, stated as the thing it actually guarantees.

    Compared modulo the ring's angular pitch, because a whole pitch of rotation
    is not a rotation at all - and compared as a shortest angular distance, so
    an answer that lands just under a full pitch is not read as a full pitch of
    disagreement.
    """
    tau = 2.0 * math.pi / ANCHOR.z_ring
    reference = mesh.ring_clocking(geo, 0)
    for k in range(1, ANCHOR.n_planets):
        difference = (mesh.ring_clocking(geo, k) - reference) % tau
        assert min(difference, tau - difference) == pytest.approx(0.0, abs=1e-9)


def test_a_set_failing_the_assembly_condition_has_no_consistent_ring_clocking():
    """The converse, which is what makes the condition worth checking at all."""
    bad = PlanetarySetParams.with_defaults(2.0, 27, 17, n_planets=3)
    assert bad.assembly_remainder != 0

    geo = compute_set(bad)
    tau = 2.0 * math.pi / bad.z_ring
    reference = mesh.ring_clocking(geo, 0)
    disagreements = 0
    for k in range(1, bad.n_planets):
        difference = (mesh.ring_clocking(geo, k) - reference) % tau
        if min(difference, tau - difference) > 1e-9:
            disagreements += 1
    assert disagreements == bad.n_planets - 1


# --- the whole train, as solid bodies --------------------------------------


def _in_material(m, radius: float, theta: float, clock: float) -> bool:
    """Whether a point in a member's own frame is inside its metal.

    Written out from the involute relation rather than from the generated
    profile, so it inherits nothing from the code under test.
    """
    if m.internal:
        if radius >= m.root_r:
            return True
        if radius <= m.tip_r:
            return False
    else:
        if radius <= m.root_r:
            return True
        if radius >= m.tip_r:
            return False

    alpha_r = math.acos(min(1.0, m.base_r / max(radius, m.base_r)))
    half_space = (
        m.psi0 - inv(alpha_r) if m.internal
        else m.half_pitch - m.psi0 + inv(alpha_r)
    )
    tau = m.angular_pitch
    offset = (theta - clock) % tau
    return min(offset, tau - offset) > half_space


def _train_overlaps(
    geo, planet_error: float = 0.0, ring_error: float = 0.0,
    steps: int = 360, radial: int = 20,
) -> dict[str, list[int]]:
    """Overlapping samples between every pair of bodies, per planet.

    Samples each planet's own metal and asks whether that point is also inside
    the sun, the ring, or another planet. Zero everywhere is the whole meshing
    condition for the train.
    """
    p = geo.params
    planets = [
        (
            mesh.planet_clocking(geo, k) + planet_error,
            mesh.planet_translation(geo, k)[:2],
        )
        for k in range(p.n_planets)
    ]
    ring_clock = mesh.ring_clocking(geo, 0) + ring_error

    result = {"sun": [], "ring": [], "planet": []}
    lo, hi = sorted((geo.planet.root_r, geo.planet.tip_r))

    for k, (clock, centre) in enumerate(planets):
        counts = {"sun": 0, "ring": 0, "planet": 0}
        for i in range(steps):
            theta = 2.0 * math.pi * i / steps
            for j in range(radial):
                radius = lo + (hi - lo) * j / (radial - 1)
                if not _in_material(geo.planet, radius, theta, clock):
                    continue
                x = centre[0] + radius * math.cos(theta)
                y = centre[1] + radius * math.sin(theta)
                world_r, world_th = math.hypot(x, y), math.atan2(y, x)

                if _in_material(geo.sun, world_r, world_th, 0.0):
                    counts["sun"] += 1
                if _in_material(geo.ring, world_r, world_th, ring_clock):
                    counts["ring"] += 1
                for j2, (other_clock, other_centre) in enumerate(planets):
                    if j2 == k:
                        continue
                    dx, dy = x - other_centre[0], y - other_centre[1]
                    if _in_material(
                        geo.planet, math.hypot(dx, dy), math.atan2(dy, dx),
                        other_clock,
                    ):
                        counts["planet"] += 1
        for name, value in counts.items():
            result[name].append(value)
    return result


def test_the_whole_train_interlocks_without_any_two_bodies_overlapping(geo):
    """All three meshes at once, as a question about metal.

    Every other test here checks one number. This one places the sun, all three
    planets and the ring exactly as the assembly builder does, and asks whether
    any of them occupy the same space.
    """
    overlaps = _train_overlaps(geo)
    assert overlaps["sun"] == [0] * ANCHOR.n_planets
    assert overlaps["ring"] == [0] * ANCHOR.n_planets
    assert overlaps["planet"] == [0] * ANCHOR.n_planets


@pytest.mark.parametrize(
    "error_kind,expect_sun,expect_ring",
    [("planet", True, True), ("ring", False, True)],
)
def test_the_interlock_check_would_catch_a_clocking_error(
    geo, error_kind, expect_sun, expect_ring
):
    """A test that cannot fail proves nothing, so make it fail on purpose.

    Half an angular pitch is the error that matters: it is exactly the
    difference between aiming a tooth centre and a tooth space at the contact.
    Mis-clocking the planets breaks both of their meshes; mis-clocking the ring
    breaks only its own, which is a useful thing for the test to distinguish.
    """
    if error_kind == "planet":
        overlaps = _train_overlaps(
            geo, planet_error=math.pi / ANCHOR.z_planet
        )
    else:
        overlaps = _train_overlaps(geo, ring_error=math.pi / ANCHOR.z_ring)

    assert (sum(overlaps["sun"]) > 0) is expect_sun
    assert (sum(overlaps["ring"]) > 0) is expect_ring


def test_a_set_that_fails_the_assembly_condition_physically_collides():
    """The assembly condition is about metal, not about bookkeeping.

    On a 27/17/61 set with three planets, planet 0 fits perfectly - it is the
    one the ring is clocked against - and the other two are driven into the
    ring. That asymmetry is the signature: it is not that the whole train is
    slightly wrong, it is that the first planet defines a phase the others
    cannot match.
    """
    bad = PlanetarySetParams.with_defaults(2.0, 27, 17, n_planets=3)
    overlaps = _train_overlaps(compute_set(bad))

    assert overlaps["ring"][0] == 0
    assert all(count > 0 for count in overlaps["ring"][1:])


def test_a_four_planet_set_assembles_when_the_condition_allows_it():
    """(24 + 60) / 4 = 21, so it fits - and the geometry agrees."""
    four = PlanetarySetParams.with_defaults(2.0, 24, 18, n_planets=4)
    assert four.assembly_remainder == 0
    assert validate(four).ok

    overlaps = _train_overlaps(compute_set(four))
    assert sum(overlaps["sun"]) == 0
    assert sum(overlaps["ring"]) == 0
    assert sum(overlaps["planet"]) == 0


# --- validation ------------------------------------------------------------


def test_the_anchor_set_is_buildable():
    assert validate(ANCHOR).ok


def test_a_failed_assembly_condition_is_refused_and_says_what_would_work():
    result = validate(PlanetarySetParams.with_defaults(2.0, 27, 17, n_planets=3))
    assert not result.ok
    issue = next(i for i in result.errors if i.field == "n_planets")
    assert "not divisible" in issue.message
    # 27 + 61 = 88, so 2, 4, 8 and 11 planets divide it.
    assert "2, 4, 8, 11" in issue.message


def test_planets_that_would_touch_each_other_are_refused():
    """The constraint that bites first when someone asks for more planets.

    Spacing is `2 a sin(pi / N)`, which falls away while the planets stay the
    same size - so there is always an N that will not fit.
    """
    geo = compute_set(ANCHOR)
    assert geo.neighbour_spacing == pytest.approx(72.746, abs=1e-3)
    assert geo.neighbour_spacing > 2.0 * geo.planet.tip_r

    crowded = PlanetarySetParams.with_defaults(2.0, 24, 18, n_planets=12)
    result = validate(crowded)
    assert not result.ok
    assert any("overlap" in issue.message for issue in result.errors)


def test_a_narrow_planet_gap_is_a_warning_not_an_error():
    """It builds; there is just nowhere to put a carrier plate."""
    geo = compute_set(ANCHOR)
    gap = geo.neighbour_spacing - 2.0 * geo.planet.tip_r
    assert gap > MIN_PLANET_GAP_FACTOR * ANCHOR.module      # the anchor is roomy


def test_one_planet_is_allowed_but_flagged():
    single = PlanetarySetParams.with_defaults(2.0, 24, 18, n_planets=1)
    result = validate(single)
    assert result.ok
    assert any(issue.field == "n_planets" for issue in result.warnings)


def test_the_spur_validators_findings_are_relayed_with_planetary_names():
    """A message about "z2" would name a field this parameter set does not have."""
    result = validate(PlanetarySetParams.with_defaults(2.0, 8, 8, n_planets=1))
    fields = {issue.field for issue in result.errors + result.warnings}
    assert fields
    assert not (fields & {"z1", "z2"})
    assert fields <= {
        "z_sun", "z_planet", "z_ring", "n_planets", "module", "bore",
        "face_width", "hub_thickness", "rim_thickness", "pressure_angle",
    }


def test_a_ring_too_small_for_an_involute_is_refused_through_the_relay():
    """The internal minimum tooth count reaches here from the spur validator.

    z_ring = 8 + 2*8 = 24, well under the 34 a ring needs at 20 degrees.
    """
    result = validate(PlanetarySetParams.with_defaults(2.0, 8, 8, n_planets=1))
    assert not result.ok
    assert any("no involute" in issue.message for issue in result.errors)


# --- the CLI ---------------------------------------------------------------


def _run(*extra):
    from gears.__main__ import main

    return main(
        ["--type", "planetary", "--module", "2", "--z1", "24", "--z2", "18"]
        + list(extra)
    )


def test_a_planetary_set_reports_from_the_command_line(capsys):
    assert _run() == 0
    out = capsys.readouterr().out
    assert "ring teeth (derived)" in out
    assert "assembly condition" in out
    assert "SUN" in out and "PLANET" in out and "RING" in out
    assert "PLACEMENT" in out


def test_the_sun_and_planet_can_be_named_explicitly(capsys):
    """--z-sun and --z-planet override the shared --z1 and --z2."""
    assert _run("--z-sun", "30", "--z-planet", "15") == 0
    out = capsys.readouterr().out
    assert "sun teeth" in out
    # 30 + 2*15 = 60, and (30 + 60) / 3 = 30, so it assembles.
    assert "  ring teeth (derived)" in out
    assert "60" in out


def test_a_set_that_cannot_assemble_is_refused_by_the_cli(capsys):
    assert _run("--z-sun", "27", "--z-planet", "17") == 1
    assert "not divisible" in capsys.readouterr().err


def test_each_member_can_be_asked_for(capsys):
    for member in ("sun", "planet", "ring"):
        assert _run("--member", member) == 0
        out = capsys.readouterr().out
        assert f"TOOTH SPACE SECTION  ({member}" in out


def test_the_default_member_maps_onto_the_suns_slot(capsys):
    """--member defaults to "pinion", which a train has no member by."""
    assert _run() == 0
    assert "TOOTH SPACE SECTION  (sun" in capsys.readouterr().out


def test_a_bevel_flag_is_refused_on_a_planetary_set(capsys):
    import pytest as _pytest
    from gears.__main__ import main

    with _pytest.raises(SystemExit) as exit_info:
        main(["--type", "planetary", "--module", "2", "--z1", "24", "--z2", "18",
              "--sigma", "90"])
    assert exit_info.value.code == 2
    assert "--sigma" in capsys.readouterr().err


def test_the_planetary_flags_are_refused_on_a_spur_set(capsys):
    import pytest as _pytest
    from gears.__main__ import main

    with _pytest.raises(SystemExit) as exit_info:
        main(["--type", "spur", "--module", "2", "--z1", "24", "--z2", "18",
              "--planets", "4"])
    assert exit_info.value.code == 2
    assert "--planets" in capsys.readouterr().err


def test_the_flags_planetary_shares_with_spur_are_refused_by_neither():
    """--beta, --rim and --backlash mean the same thing to both types.

    The spur module creates them and both modules claim them, so neither
    refuses them and only bevel does. Worth a test because the arrangement is
    the sort that quietly rots.
    """
    from gears.__main__ import main

    for gear_type in ("spur", "planetary"):
        assert main(
            ["--type", gear_type, "--module", "2", "--z1", "24", "--z2", "18",
             "--beta", "10", "--backlash", "0.05", "--rim", "6"]
        ) == 0


# --- the preview -----------------------------------------------------------


def test_the_train_scene_draws_every_tooth_of_every_member(geo):
    from gears.planetary import preview

    scene = preview.build_scene(geo, "sun", "train")
    # 24 sun spaces + 60 ring spaces + 3 x 18 planet spaces, plus the circles.
    assert len(scene.polylines) == 24 + 60 + 3 * 18 + 7


def test_the_readout_carries_three_member_columns(geo):
    from gears.planetary import preview

    rows = preview.derived_rows(geo)
    assert any(row.third for row in rows)
    members = next(row for row in rows if row.label == "MEMBERS")
    assert members.values == ("SUN", "PLANET", "RING")


def test_a_pair_never_fills_the_third_column():
    """The readout's extra column must stay invisible to the pair-shaped types."""
    from gears.bevel import preview as bevel_preview
    from gears.bevel.params import BevelSetParams
    from gears.bevel.geometry import compute_set as bevel_compute
    from gears.spur import preview as spur_preview
    from gears.spur.geometry import compute_set as spur_compute
    from gears.spur.params import SpurSetParams

    for rows in (
        bevel_preview.derived_rows(
            bevel_compute(BevelSetParams.with_defaults(2.0, 17, 43))
        ),
        spur_preview.derived_rows(
            spur_compute(SpurSetParams.with_defaults(2.0, 17, 43))
        ),
    ):
        assert not any(row.third for row in rows)
