"""Validation for Method 1 hypoid inputs."""

from __future__ import annotations

import math

from ..validate import (
    MAX_PRESSURE_ANGLE,
    MIN_PRESSURE_ANGLE,
    MIN_TEETH,
    ValidationResult,
)
from .geometry import _cutter_trace, compute_set, tooth_space_section
from .params import HypoidSetParams

MAX_METHOD1_OFFSET_FRACTION = 0.25
MAX_SPIRAL_ANGLE = 60.0


def validate(p: HypoidSetParams) -> ValidationResult:
    result = ValidationResult()
    if p.module <= 0:
        result.error("module", "must be greater than zero")
    if p.z1 < MIN_TEETH:
        result.error("z1", f"pinion needs at least {MIN_TEETH} teeth")
    if p.z2 < MIN_TEETH:
        result.error("z2", f"gear needs at least {MIN_TEETH} teeth")
    if not MIN_PRESSURE_ANGLE <= p.pressure_angle <= MAX_PRESSURE_ANGLE:
        result.error("pressure_angle", f"must be between {MIN_PRESSURE_ANGLE} and {MAX_PRESSURE_ANGLE} degrees")
    if not 0.0 < p.shaft_angle < 180.0:
        result.error("shaft_angle", "must be between 0 and 180 degrees")
    if p.face_width <= 0:
        result.error("face_width", "must be greater than zero")
    if p.face_width >= p.wheel_outer_radius:
        result.error("face_width", "must be less than the wheel outer radius")
    if p.bore < 0:
        result.error("bore", "cannot be negative")
    if p.hub_thickness < 0 or p.min_root_thickness < 0:
        result.error("hub_thickness", "backing dimensions cannot be negative")
    if not math.isfinite(p.backlash) or p.backlash < 0:
        result.error("backlash", "must be finite and non-negative")
    if not -0.95 < p.thickness_factor < 0.95:
        result.error("thickness_factor", "must be between -0.95 and 0.95")
    if p.hand not in ("right", "left"):
        result.error("hand", "must be 'right' or 'left'")
    if not -MAX_SPIRAL_ANGLE <= p.spiral_angle <= MAX_SPIRAL_ANGLE:
        result.error("spiral_angle", f"must be between -{MAX_SPIRAL_ANGLE} and {MAX_SPIRAL_ANGLE} degrees")
    if p.cutter_radius is not None and p.cutter_radius <= 0:
        result.error("cutter_radius", "must be greater than zero")
    if abs(p.offset) > 0.0 and p.cutter_radius is None:
        result.error(
            "cutter_radius",
            "is required for non-zero-offset Method 1 curvature closure",
        )
    if abs(p.offset) > MAX_METHOD1_OFFSET_FRACTION * p.wheel_outer_diameter:
        result.error(
            "offset",
            "Method 1 offset magnitude must not exceed "
            f"{MAX_METHOD1_OFFSET_FRACTION:.0%} of the wheel outer diameter",
        )
    if result.errors:
        return result

    try:
        geo = compute_set(p)
    except (ValueError, ArithmeticError) as exc:
        message = str(exc)
        if p.cutter_radius is not None and "Method 1 curvature" in message:
            field = "cutter_radius"
        elif "tooth thickness" in message:
            field = "backlash"
        else:
            field = "geometry"
        result.error(field, f"Method 1 geometry failed: {exc}")
        return result

    for member in (geo.pinion, geo.gear):
        if member.pitch_angle <= 0 or member.pitch_angle >= math.pi / 2:
            result.error(member.name, "pitch angle is outside the external hypoid range")
        if member.virtual_tip_r <= member.virtual_base_r:
            result.error(member.name, "tip circle does not reach the involute")
        if member.virtual_root_r <= 0:
            result.error(member.name, "root radius is not positive")
        if member.mean_normal_tooth_thickness <= 0.0:
            result.error(
                "backlash",
                f"{member.name} mean normal tooth thickness is not positive",
            )
        try:
            tooth_space_section(geo, member.name)
        except ValueError as exc:
            result.error(
                member.name,
                "Tredgold tooth-space approximation failed: " + str(exc),
            )

    if p.cutter_radius is not None:
        for member in (geo.pinion, geo.gear):
            trace = _cutter_trace(member, geo)
            inner = member.tooth_face_inner_cone_distance
            outer = member.tooth_face_outer_cone_distance
            if not trace.reaches(inner, outer):
                lo = abs(trace.centre_distance - trace.cutter_radius)
                hi = trace.centre_distance + trace.cutter_radius
                result.error(
                    "cutter_radius",
                    f"a {trace.cutter_radius:.2f} mm cutter does not cover "
                    f"the {member.name} Method 1 tooth face from "
                    f"{inner:.2f} to {outer:.2f} mm; "
                    f"its arc spans {lo:.2f} to {hi:.2f} mm",
                )

    if p.cutter_radius is not None and p.cutter_radius < 0.25 * p.module:
        result.warn("cutter_radius", "a very small cutter produces a sharply varying spiral")
    if abs(p.offset) > 0.15 * p.wheel_outer_diameter:
        result.warn("offset", "offset exceeds the usual 15% design range")
    if abs(p.spiral_angle) > 45:
        result.warn("spiral_angle", "high spiral angle increases axial thrust")
    if p.bore / 2.0 + p.module >= geo.pinion.inner_root_radius:
        result.error(
            "bore",
            "pinion bore leaves insufficient material below the Method 1 "
            f"inner root radius ({geo.pinion.inner_root_radius:.3f} mm)",
        )
    return result


__all__ = ["ValidationResult", "validate"]
