"""Accuracy study for the pure-Python hypoid tooth-surface approximation.

This is deliberately a study tool, not a production geometry path.  The
repository does not contain enough cutter and machine-setting data to rebuild
an ISO/Gleason generated envelope.  It therefore compares the current
back-cone/Tredgold section loft against two explicit trace-transport comparison
models and reports the error categories separately:

``raw-crown``
    The circular ``CrownTrace`` angle used directly as the longitudinal phase,
    without the pitch-cone development factor.  This is the intentionally
    unscaled comparison model used by the original hypoid path.

``developed-cone``
    The ``CrownTrace`` documentation's cone-development mapping, in which the
    crown-plane angle is divided by ``sin(pitch_angle)`` before it is used as
    real cone phase.  For a non-zero-offset pinion, the production path also
    applies the Method 1 wheel-to-pinion offset transport, so this remains an
    explicit circular-trace comparison model rather than an exact generated
    target.

Neither reference is a true generated hypoid flank.  A true envelope also
needs the inside/outside blade geometry, cutter-head motion and machine
settings.  The tool reports that limitation instead of presenting a fitted
trace model as generated-surface accuracy.

Run from the repository root with the project interpreter, for example::

    .venv\\Scripts\\python.exe tools\\hypoid_accuracy_study.py --json
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Iterable

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from gears.hypoid.geometry import (
    HypoidSetGeometry,
    _cutter_trace,
    compute_set,
    section_cone_distances,
    tooth_space_section,
)
from gears.hypoid.mesh import (
    contact_point,
    gear_contact_point,
    gear_mate_ratio,
    gear_translation,
    skew_axis_distance,
)
from gears.hypoid.params import HypoidSetParams
from gears.bevel.geometry import to_cone_3d


ANCHOR = HypoidSetParams.with_defaults(
    170.0 / 42.0,
    13,
    42,
    offset=15.0,
    face_width=30.0,
    spiral_angle=50.0,
    cutter_radius=63.5,
    backlash=0.2,
)

STUDY_FLANK_POINTS = 48
_SECTION_CACHE = {}


@dataclass(frozen=True)
class SurfaceSample:
    position: tuple[float, float, float]
    normal: tuple[float, float, float]


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def _sub(a, b):
    return tuple(a[i] - b[i] for i in range(len(a)))


def _add(a, b):
    return tuple(a[i] + b[i] for i in range(len(a)))


def _scale(a, factor):
    return tuple(value * factor for value in a)


def _dot(a, b):
    return sum(a[i] * b[i] for i in range(len(a)))


def _cross(a, b):
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _norm(a):
    return math.sqrt(_dot(a, a))


def _unit(a):
    length = _norm(a)
    if length <= 1e-14:
        raise ValueError("zero-length surface differential")
    return _scale(a, 1.0 / length)


def _distance(a, b) -> float:
    return _norm(_sub(a, b))


def _lerp(a, b, t):
    return _add(a, _scale(_sub(b, a), t))


def _polyline_point(points, t: float):
    """Return a point at normalized index, preserving root-to-tip order."""
    if len(points) < 2:
        raise ValueError("a flank needs at least two points")
    value = _clamp(t, 0.0, 1.0) * (len(points) - 1)
    index = min(len(points) - 2, int(math.floor(value)))
    return _lerp(points[index], points[index + 1], value - index)


def _trace_reference_phase(
    geo: HypoidSetGeometry,
    member: str,
    cone_dist: float,
    mode: str,
) -> float:
    m = geo.member(member)
    trace = _cutter_trace(m, geo)
    if trace is None:
        return 0.0
    theta = trace.theta_at(cone_dist)
    if mode == "developed-cone":
        theta /= max(abs(math.sin(m.pitch_angle)), 1e-12)
    elif mode != "raw-crown":
        raise ValueError(f"unknown trace reference {mode!r}")
    hand = 1.0 if geo.params.hand == "right" else -1.0
    # The two members traverse the same cutter trace in opposite senses.
    return (-hand if member == "pinion" else hand) * theta


def _section_flank_point(
    geo: HypoidSetGeometry,
    member: str,
    side: str,
    cone_dist: float,
    profile: float,
    phase_mode: str | None,
):
    key = (id(geo), member, round(cone_dist, 10))
    section = _SECTION_CACHE.get(key)
    if section is None:
        section = tooth_space_section(
            geo, member, cone_dist, n_flank=STUDY_FLANK_POINTS
        )
        _SECTION_CACHE[key] = section
    flank = section.drive_flank if side == "drive" else section.coast_flank
    x, y = _polyline_point(flank, profile)
    phase = (
        section.phase
        if phase_mode is None
        else _trace_reference_phase(geo, member, cone_dist, phase_mode)
    )
    return to_cone_3d(
        x,
        y,
        section.pitch_angle,
        section.cone_apex_z,
        phase,
    )


def _surface_normal(
    geo: HypoidSetGeometry,
    member: str,
    side: str,
    cone_dist: float,
    profile: float,
    phase_mode: str | None,
):
    m = geo.member(member)
    lo, hi = m.tooth_face_inner_cone_distance, m.tooth_face_outer_cone_distance
    h_a = max(1e-4, min(0.02, (hi - lo) * 1e-3))
    a0, a1 = cone_dist - h_a, cone_dist + h_a
    if a0 < lo:
        a0, a1 = cone_dist, cone_dist + h_a
    if a1 > hi:
        a0, a1 = cone_dist - h_a, cone_dist
    h_t = 1e-4
    t0, t1 = max(0.01, profile - h_t), min(0.99, profile + h_t)
    tangent_a = _sub(
        _section_flank_point(geo, member, side, a1, profile, phase_mode),
        _section_flank_point(geo, member, side, a0, profile, phase_mode),
    )
    tangent_t = _sub(
        _section_flank_point(geo, member, side, cone_dist, t1, phase_mode),
        _section_flank_point(geo, member, side, cone_dist, t0, phase_mode),
    )
    return _unit(_cross(tangent_a, tangent_t))


def _surface_sample(
    geo: HypoidSetGeometry,
    member: str,
    side: str,
    cone_dist: float,
    profile: float,
    phase_mode: str | None,
) -> SurfaceSample:
    return SurfaceSample(
        _section_flank_point(
            geo, member, side, cone_dist, profile, phase_mode
        ),
        _surface_normal(
            geo, member, side, cone_dist, profile, phase_mode
        ),
    )


def _angle_between(a, b) -> float:
    return math.degrees(math.acos(_clamp(abs(_dot(a, b)), -1.0, 1.0)))


def _summary(values: Iterable[float]) -> dict[str, float]:
    values = list(values)
    if not values:
        return {"max": 0.0, "rms": 0.0, "mean": 0.0}
    return {
        "max": max(values),
        "rms": math.sqrt(sum(value * value for value in values) / len(values)),
        "mean": sum(values) / len(values),
    }


def _production_pitch_trace_spiral_angle(
    geo: HypoidSetGeometry, member: str, cone_dist: float
) -> float:
    """Differentiate the actual section/``to_cone_3d`` production path.

    The point follows the developed pitch generator through the same phase
    stored on ``tooth_space_section``.  At the local radial azimuth ``q`` its
    cone generator and circumferential directions are

        g = (sin(delta) cos(q), sin(delta) sin(q), cos(delta))
        c = (-sin(q), cos(q), 0).

    Thus ``atan2(v·c, v·g)`` is the physical longitudinal trace angle.  This
    is a diagnostic of the production Tredgold trace transport, not a claim
    that the flank is a generated or conjugate hypoid surface.
    """
    m = geo.member(member)
    h = max(1e-5, min(1e-3, 1e-4 * m.cone_distance))

    def point(value: float):
        section = tooth_space_section(geo, member, value)
        developed_pitch_radius = m.virtual_pitch_r * value / m.cone_distance
        return to_cone_3d(
            developed_pitch_radius,
            0.0,
            m.pitch_angle,
            section.cone_apex_z,
            section.phase,
        )

    before = point(cone_dist - h)
    after = point(cone_dist + h)
    tangent = _scale(_sub(after, before), 1.0 / (2.0 * h))
    current = point(cone_dist)
    azimuth = math.atan2(current[1], current[0])
    circumferential = (-math.sin(azimuth), math.cos(azimuth), 0.0)
    generator = (
        math.sin(m.pitch_angle) * math.cos(azimuth),
        math.sin(m.pitch_angle) * math.sin(azimuth),
        math.cos(m.pitch_angle),
    )
    return math.atan2(_dot(tangent, circumferential), _dot(tangent, generator))


def _production_trace_report(
    geo: HypoidSetGeometry, member: str
) -> dict:
    """Report actual-vs-Method-1 spiral angles at all physical boundaries."""
    m = geo.member(member)
    expected_sense = 1.0 if member == "pinion" else -1.0
    stations = {
        "inner": (m.tooth_face_inner_cone_distance, m.inner_spiral_angle),
        "mean": (m.cone_distance, m.mean_spiral_angle),
        "outer": (m.tooth_face_outer_cone_distance, m.outer_spiral_angle),
    }
    values = {}
    for name, (cone_dist, expected) in stations.items():
        actual = _production_pitch_trace_spiral_angle(geo, member, cone_dist)
        expected_local = expected_sense * expected
        values[name] = {
            "cone_dist_mm": cone_dist,
            "actual_signed_deg": math.degrees(actual),
            "method1_signed_deg": math.degrees(expected_local),
            "error_deg": math.degrees(actual - expected_local),
        }
    errors = [abs(value["error_deg"]) for value in values.values()]
    return {
        "stations": values,
        "max_abs_error_deg": max(errors),
        "rms_error_deg": math.sqrt(sum(value * value for value in errors) / len(errors)),
    }


def _compare_approximation_models(
    geo: HypoidSetGeometry, member: str, side: str
) -> dict:
    """Compare two explicit trace transports, not a generated reference."""
    m = geo.member(member)
    longitudinal = [
        m.tooth_face_inner_cone_distance
        + (m.tooth_face_outer_cone_distance - m.tooth_face_inner_cone_distance)
        * fraction
        for fraction in (0.0, 0.25, 0.5, 0.75, 1.0)
    ]
    profiles = (0.2, 0.5, 0.8)
    position_errors = []
    normal_errors = []
    samples = []
    for cone_dist in longitudinal:
        for profile in profiles:
            raw = _surface_sample(
                geo, member, side, cone_dist, profile, "raw-crown"
            )
            developed = _surface_sample(
                geo, member, side, cone_dist, profile, "developed-cone"
            )
            position_errors.append(_distance(raw.position, developed.position))
            normal_errors.append(_angle_between(raw.normal, developed.normal))
            samples.append(
                {
                    "cone_dist_mm": cone_dist,
                    "profile": profile,
                    "position_deviation_mm": position_errors[-1],
                    "normal_deviation_deg": normal_errors[-1],
                }
            )
    return {
        "position_deviation_mm": _summary(position_errors),
        "normal_deviation_deg": _summary(normal_errors),
        "samples": samples,
    }


def _compare_surface(
    geo: HypoidSetGeometry,
    member: str,
    side: str,
    reference: str,
) -> dict:
    """Compare production sections with one named phase-transport model.

    The returned positional values are longitudinal deviations of the
    production Tredgold surface from that explicit comparison model.  They
    are not macro-geometry errors and neither comparison model is a generated
    hypoid envelope.
    """
    m = geo.member(member)
    longitudinal = [
        m.tooth_face_inner_cone_distance
        + (m.tooth_face_outer_cone_distance - m.tooth_face_inner_cone_distance)
        * fraction
        for fraction in (0.0, 0.25, 0.5, 0.75, 1.0)
    ]
    profiles = (0.2, 0.5, 0.8)
    position_errors = []
    normal_errors = []
    phase_errors = []
    pitch_tangential_errors = []
    samples = []
    for cone_dist in longitudinal:
        current_section = _SECTION_CACHE.get(
            (id(geo), member, round(cone_dist, 10))
        )
        if current_section is None:
            current_section = tooth_space_section(
                geo, member, cone_dist, n_flank=STUDY_FLANK_POINTS
            )
            _SECTION_CACHE[(id(geo), member, round(cone_dist, 10))] = current_section
        reference_phase = _trace_reference_phase(
            geo, member, cone_dist, reference
        )
        phase_errors.append(abs(current_section.phase - reference_phase))
        pitch_tangential_errors.append(
            m.pitch_radius * abs(current_section.phase - reference_phase)
        )
        for profile in profiles:
            current = _surface_sample(
                geo, member, side, cone_dist, profile, None
            )
            target = _surface_sample(
                geo, member, side, cone_dist, profile, reference
            )
            position_errors.append(_distance(current.position, target.position))
            normal_errors.append(_angle_between(current.normal, target.normal))
            samples.append(
                {
                    "cone_dist": cone_dist,
                    "profile": profile,
                    "position_error_mm": position_errors[-1],
                    "normal_error_deg": normal_errors[-1],
                }
            )
    return {
        "position_error_mm": _summary(position_errors),
        "longitudinal_position_deviation_mm": _summary(position_errors),
        "normal_error_deg": _summary(normal_errors),
        "surface_normal_deviation_deg": _summary(normal_errors),
        "phase_error_rad": _summary(phase_errors),
        "phase_error_deg": _summary(math.degrees(value) for value in phase_errors),
        "pitch_tangential_error_mm": _summary(pitch_tangential_errors),
        "samples": samples,
    }


def _loft_chord_deviation(
    geo: HypoidSetGeometry, member: str, side: str
) -> dict[str, float]:
    """Measure section-to-section chord error, not a CAD spline error.

    SOLIDWORKS interpolates the profiles with a spline.  This intentionally
    reports the simpler straight-chord deviation as a reproducible diagnostic;
    it is a bound/indicator for station density, not a claim about the exact
    B-spline loft implementation.
    """
    m = geo.member(member)
    stations = section_cone_distances(geo, member)
    lo, hi = m.tooth_face_inner_cone_distance, m.tooth_face_outer_cone_distance
    values = []
    for a0, a1 in zip(stations, stations[1:]):
        if a1 <= lo or a0 >= hi:
            continue
        start = max(a0, lo)
        stop = min(a1, hi)
        for fraction in (0.25, 0.5, 0.75):
            a = start + (stop - start) * fraction
            for profile in (0.2, 0.5, 0.8):
                p0 = _section_flank_point(geo, member, side, start, profile, None)
                p1 = _section_flank_point(geo, member, side, stop, profile, None)
                actual = _section_flank_point(geo, member, side, a, profile, None)
                chord = _lerp(p0, p1, (a - start) / max(stop - start, 1e-12))
                values.append(_distance(actual, chord))
    return _summary(values)


def _contact_summary(geo: HypoidSetGeometry) -> dict[str, float]:
    p = contact_point(geo)
    g = gear_contact_point(geo)
    translation = gear_translation(geo)
    axis1 = (0.0, 0.0, 1.0)
    axis2 = (math.sin(geo.params.sigma), 0.0, math.cos(geo.params.sigma))
    axis_distance = skew_axis_distance(
        (0.0, 0.0, 0.0), axis1, translation, axis2
    )
    ratio = gear_mate_ratio(geo.pinion.z, geo.gear.z)
    return {
        "mean_contact_position_error_mm": _distance(p, g),
        "shaft_axis_distance_mm": axis_distance,
        "shaft_axis_offset_error_mm": axis_distance - abs(geo.params.offset),
        "velocity_ratio": -ratio[1] / ratio[0],
        "velocity_ratio_error": -ratio[1] / ratio[0] + geo.params.ratio,
        "pinion_mean_phase_deg": math.degrees(
            _section_phase(geo, "pinion", geo.pinion.cone_distance)
        ),
        "gear_mean_phase_deg": math.degrees(
            _section_phase(geo, "gear", geo.gear.cone_distance)
        ),
    }


def _section_phase(geo, member, cone_dist):
    return tooth_space_section(geo, member, cone_dist).phase


def _case_report(params: HypoidSetParams) -> dict:
    try:
        geo = compute_set(params)
        member_reports = {}
        for member in ("pinion", "gear"):
            raw_crown = {
                side: _compare_surface(geo, member, side, "raw-crown")
                for side in ("drive", "coast")
            }
            developed_cone = {
                side: _compare_surface(geo, member, side, "developed-cone")
                for side in ("drive", "coast")
            }
            raw_vs_developed = {
                side: _compare_approximation_models(geo, member, side)
                for side in ("drive", "coast")
            }
            member_reports[member] = {
                "tooth_face_inner_mm": geo.member(member).tooth_face_inner_cone_distance,
                "tooth_face_outer_mm": geo.member(member).tooth_face_outer_cone_distance,
                "method1_face_width_mm": geo.member(member).face_width,
                "pitch_cone_face_width_mm": geo.member(member).face_width_along_pitch_cone,
                "tooth_face_width_mm": geo.member(member).tooth_face_width,
                "longitudinal_trace_transport": {
                    "production_method1_trace": _production_trace_report(geo, member),
                    "production_vs_raw_crown_model": raw_crown,
                    "production_vs_developed_cone_model": developed_cone,
                    "raw_crown_vs_developed_cone_model": raw_vs_developed,
                },
                "tredgold_section_profile": {
                    "production_model": "Tredgold/back-cone involute section",
                    "generated_envelope_reference_available": False,
                    "profile_error": None,
                    "limitation": (
                        "No true generated-envelope inputs are available, so "
                        "profile approximation error is not isolated."
                    ),
                },
                "loft_station_interpolation": {
                    side: _loft_chord_deviation(geo, member, side)
                    for side in ("drive", "coast")
                },
            }
            # Keep the old top-level names readable for downstream scripts,
            # while the structured names above make the diagnostic category
            # explicit.  These are comparison models, never generated flanks.
            member_reports[member]["raw_crown"] = raw_crown
            member_reports[member]["developed_cone"] = developed_cone
            member_reports[member]["loft_chord"] = member_reports[member][
                "loft_station_interpolation"
            ]
        contact = _contact_summary(geo)
        return {
            "valid": True,
            "inputs": {
                "module_mm": params.module,
                "z1": params.z1,
                "z2": params.z2,
                "offset_mm": params.offset,
                "face_width_mm": params.face_width,
                "spiral_angle_deg": params.spiral_angle,
                "cutter_radius_mm": params.cutter_radius,
            },
            "macro": {
                "pinion_pitch_angle_deg": geo.pinion.pitch_angle_deg,
                "gear_pitch_angle_deg": geo.gear.pitch_angle_deg,
                "offset_angle_deg": geo.offset_angle_deg,
                "pitch_plane_offset_mm": geo.pitch_plane_offset,
                "mean_normal_module_mm": geo.mean_normal_module,
                "pinion_mean_spiral_deg": geo.pinion.mean_spiral_angle_deg,
                "gear_mean_spiral_deg": geo.gear.mean_spiral_angle_deg,
                "pinion_face_width_mm": geo.pinion.face_width,
                "gear_face_width_mm": geo.gear.face_width,
                "pinion_pitch_cone_face_width_mm": geo.pinion.face_width_along_pitch_cone,
                "gear_pitch_cone_face_width_mm": geo.gear.face_width_along_pitch_cone,
            },
            "method1_macro_geometry": contact,
            "contact": contact,
            "members": member_reports,
            "true_generated_reference": {
                "available": False,
                "missing_inputs": [
                    "inside and outside cutter blade edge profiles",
                    "cutter-head blade spacing and indexing",
                    "rake, hook, relief and blade pressure-angle data",
                    "machine settings and relative cutter/work motions",
                ],
            },
        }
    except (ArithmeticError, ValueError, ZeroDivisionError) as exc:
        return {
            "valid": False,
            "inputs": {
                "module_mm": params.module,
                "z1": params.z1,
                "z2": params.z2,
                "offset_mm": params.offset,
                "face_width_mm": params.face_width,
                "spiral_angle_deg": params.spiral_angle,
                "cutter_radius_mm": params.cutter_radius,
            },
            "error": f"{type(exc).__name__}: {exc}",
        }


def _cases() -> list[tuple[str, HypoidSetParams]]:
    # The cases are deliberately not all anchor perturbations.  They exercise
    # independent ratio, offset, face-width and cutter-radius directions.
    cases = [("anchor", ANCHOR)]
    cases.extend(
        (f"offset_{offset:g}", replace(ANCHOR, offset=offset))
        for offset in (0.0, 5.0, 25.0, -15.0)
    )
    cases.extend(
        (f"face_{width:g}", replace(ANCHOR, face_width=width))
        for width in (20.0, 40.0)
    )
    cases.extend(
        (f"cutter_{radius:g}", replace(ANCHOR, cutter_radius=radius))
        for radius in (40.0, 100.0)
    )
    cases.append(("invalid_cutter_1", replace(ANCHOR, cutter_radius=1.0)))
    cases.extend(
        (f"ratio_{z1}_{z2}", replace(ANCHOR, z1=z1, z2=z2))
        for z1, z2 in ((15, 42), (17, 50))
    )
    cases.append(("left_hand_anchor", replace(ANCHOR, hand="left")))
    return cases


def _markdown(report: dict) -> str:
    lines = [
        "# Hypoid accuracy study",
        "",
        "This report separates Method 1 macro geometry, longitudinal trace "
        "transport, Tredgold section-profile comparison, and loft station "
        "interpolation. The named raw-crown and developed-cone models are "
        "explicit comparison models, not a true generated-envelope reference.",
        "",
        "## Comparison-model positional deviations",
        "",
        "| case | valid | pinion raw mm | gear raw mm | pinion developed mm | gear developed mm |",
        "|---|---:|---:|---:|---:|---:|",
    ]
    for name, result in report["cases"].items():
        if not result["valid"]:
            lines.append(f"| {name} | no | n/a | n/a | n/a | n/a |")
            continue
        p = result["members"]["pinion"]
        g = result["members"]["gear"]
        p_raw = max(p["raw_crown"][s]["position_error_mm"]["max"] for s in ("drive", "coast"))
        g_raw = max(g["raw_crown"][s]["position_error_mm"]["max"] for s in ("drive", "coast"))
        p_dev = max(p["developed_cone"][s]["position_error_mm"]["max"] for s in ("drive", "coast"))
        g_dev = max(g["developed_cone"][s]["position_error_mm"]["max"] for s in ("drive", "coast"))
        lines.append(
            f"| {name} | yes | {p_raw:.6g} | {g_raw:.6g} | {p_dev:.6g} | {g_dev:.6g} |"
        )
    lines.extend(
        [
            "",
            "## Production-path trace and macro checks",
            "",
            "| case | member | max spiral error deg | inner error deg | mean error deg | outer error deg | loft chord max mm |",
            "|---|---|---:|---:|---:|---:|---:|",
        ]
    )
    for name, result in report["cases"].items():
        if not result["valid"]:
            continue
        for member in ("pinion", "gear"):
            trace = result["members"][member]["longitudinal_trace_transport"][
                "production_method1_trace"
            ]
            stations = trace["stations"]
            loft = result["members"][member]["loft_station_interpolation"]
            chord_max = max(loft[side]["max"] for side in ("drive", "coast"))
            lines.append(
                f"| {name} | {member} | {trace['max_abs_error_deg']:.6g} | "
                f"{abs(stations['inner']['error_deg']):.6g} | "
                f"{abs(stations['mean']['error_deg']):.6g} | "
                f"{abs(stations['outer']['error_deg']):.6g} | {chord_max:.6g} |"
            )
    lines.extend(
        [
            "",
            "The JSON output additionally contains macro contact/axis/ratio "
            "residuals, production-vs-model longitudinal position and surface-"
            "normal deviations, direct comparison between the two explicit "
            "phase models, face limits, and section-chord diagnostics.",
        ]
    )
    return "\n".join(lines)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--json", action="store_true", help="emit machine-readable JSON")
    args = parser.parse_args()
    report = {"cases": {name: _case_report(params) for name, params in _cases()}}
    if args.json:
        print(json.dumps(report, indent=2, sort_keys=True))
    else:
        print(_markdown(report))


if __name__ == "__main__":
    main()
