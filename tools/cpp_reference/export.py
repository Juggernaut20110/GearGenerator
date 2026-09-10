"""Export compact Python reference snapshots for the native port.

This is deliberately a development tool, not a runtime dependency of the C++
application.  The Python implementation remains the behavioral authority while
the native port is being built.  Only stable scalar results are exported at
this stage; full tooth-space point arrays belong in later geometry fixtures.
"""

from __future__ import annotations

import argparse
import dataclasses
import json
import math
import sys
from pathlib import Path

REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
if str(REPOSITORY_ROOT) not in sys.path:
    sys.path.insert(0, str(REPOSITORY_ROOT))

from gears.bevel.geometry import compute_set as compute_bevel
from gears.bevel.params import BevelSetParams
from gears.bevel.validate import validate as validate_bevel
from gears.hypoid.geometry import compute_set as compute_hypoid
from gears.hypoid.params import HypoidSetParams
from gears.hypoid.validate import validate as validate_hypoid
from gears.planetary.geometry import compute_set as compute_planetary
from gears.planetary.params import PlanetarySetParams
from gears.planetary.validate import validate as validate_planetary
from gears.spur.geometry import compute_set as compute_spur
from gears.spur.params import SpurSetParams
from gears.spur.validate import validate as validate_spur


def finite_or_none(value):
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def clean(value):
    if dataclasses.is_dataclass(value):
        return clean(dataclasses.asdict(value))
    if isinstance(value, dict):
        return {str(key): clean(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [clean(item) for item in value]
    if isinstance(value, float):
        return finite_or_none(value)
    return value


def validation_snapshot(result):
    return {
        "ok": result.ok,
        "errors": [dataclasses.asdict(issue) for issue in result.errors],
        "warnings": [dataclasses.asdict(issue) for issue in result.warnings],
        "advisories": [],
    }


def member_snapshot(member):
    return {
        "z": member.z,
        "beta": member.beta,
        "reference_r": member.reference_r,
        "base_r": member.base_r,
        "working_r": member.working_r,
        "tip_r": member.tip_r,
        "root_r": member.root_r,
        "twist": member.twist,
        "internal": member.internal,
    }


def spur_snapshot(params: SpurSetParams):
    geometry = compute_spur(params)
    return {
        "transverse_module": params.transverse_module,
        "alpha_n": params.alpha_n,
        "alpha_t": params.alpha_t,
        "beta": params.beta,
        "ratio": params.ratio,
        "reference_centre_distance": params.reference_centre_distance,
        "working_centre_distance": geometry.working_centre_distance,
        "working_pressure_angle": geometry.working_pressure_angle,
        "circular_pitch": geometry.circular_pitch,
        "whole_depth": geometry.whole_depth,
        "pinion": member_snapshot(geometry.pinion),
        "gear": member_snapshot(geometry.gear),
    }


def bevel_snapshot(params: BevelSetParams):
    geometry = compute_bevel(params)
    return {
        "alpha": params.alpha,
        "sigma": params.sigma,
        "psi_m": params.psi_m,
        "ratio": params.ratio,
        "trace_kind": params.trace_kind,
        "is_curved": params.is_curved,
        "outer_cone_dist": geometry.outer_cone_dist,
        "mean_cone_dist": geometry.mean_cone_dist,
        "inner_cone_dist": geometry.inner_cone_dist,
        "working_depth": geometry.working_depth,
        "whole_depth": geometry.whole_depth,
        "clearance": geometry.clearance,
        "circular_pitch": geometry.circular_pitch,
        "pinion_pitch_angle": geometry.pinion.pitch_angle,
        "gear_pitch_angle": geometry.gear.pitch_angle,
        "pinion_virtual_teeth": geometry.pinion.virtual_teeth,
        "gear_virtual_teeth": geometry.gear.virtual_teeth,
    }


def hypoid_snapshot(params: HypoidSetParams):
    geometry = compute_hypoid(params)
    return {
        "alpha": params.alpha,
        "sigma": params.sigma,
        "psi1": params.psi1,
        "ratio": params.ratio,
        "wheel_outer_diameter": params.wheel_outer_diameter,
        "effective_root_fillet_radius": params.effective_root_fillet_radius,
        # The first native foundation compares the parameter-derived scalar
        # contract here.  The converged non-zero-offset Method 1 fields are
        # deliberately deferred until that solver is ported as one unit.
    }


def planetary_snapshot(params: PlanetarySetParams):
    geometry = compute_planetary(params)
    return {
        "z_ring": params.z_ring,
        "alpha_n": params.alpha_n,
        "alpha_t": params.alpha_t,
        "beta": params.beta,
        "transverse_module": params.transverse_module,
        "centre_distance": params.centre_distance,
        "assembly_remainder": params.assembly_remainder,
        "ratio_carrier_to_sun": params.ratio_carrier_to_sun,
        "ratio_ring_to_sun": params.ratio_ring_to_sun,
        "circular_pitch": geometry.circular_pitch,
        "whole_depth": geometry.whole_depth,
        "sun": member_snapshot(geometry.sun),
        "planet": member_snapshot(geometry.planet),
        "ring": member_snapshot(geometry.ring),
    }


def snapshot(name, kind, params, validator, calculator):
    result = validator(params)
    # Geometry is intentionally computed only for valid fixtures. Invalid
    # inputs still provide their complete validation contract without asking a
    # calculator to divide by a bad module or tooth count.
    derived = calculator(params) if result.ok else {}
    return clean(
        {
            "name": name,
            "type": kind,
            "inputs": dataclasses.asdict(params),
            "validation": validation_snapshot(result),
            "derived": derived,
            "geometry": {},
        }
    )


def fixtures():
    spur_invalid = SpurSetParams.with_defaults(2.0, 20, 40)
    spur_invalid = dataclasses.replace(
        spur_invalid, module=-1.0, z1=3, pressure_angle=30.0
    )
    return [
        snapshot(
            "spur_external",
            "spur",
            SpurSetParams.with_defaults(2.0, 20, 40),
            validate_spur,
            spur_snapshot,
        ),
        snapshot(
            "spur_helical",
            "spur",
            SpurSetParams.with_defaults(2.0, 17, 43, helix_angle=15.0),
            validate_spur,
            spur_snapshot,
        ),
        snapshot(
            "spur_internal_profile_shift",
            "spur",
            SpurSetParams.with_defaults(
                2.0, 18, 60, internal=True, profile_shift_1=0.3
            ),
            validate_spur,
            spur_snapshot,
        ),
        snapshot("spur_invalid", "spur", spur_invalid, validate_spur, spur_snapshot),
        snapshot(
            "bevel_straight",
            "bevel",
            BevelSetParams.with_defaults(2.0, 25, 40),
            validate_bevel,
            bevel_snapshot,
        ),
        snapshot(
            "bevel_spiral",
            "bevel",
            BevelSetParams.with_defaults(2.0, 25, 40, spiral_angle=35.0),
            validate_bevel,
            bevel_snapshot,
        ),
        snapshot(
            "bevel_zerol",
            "bevel",
            BevelSetParams.with_defaults(2.0, 25, 40, zerol=True),
            validate_bevel,
            bevel_snapshot,
        ),
        snapshot(
            "hypoid_offset_sample",
            "hypoid",
            HypoidSetParams.with_defaults(
                170.0 / 42.0,
                13,
                42,
                offset=15.0,
                face_width=30.0,
                spiral_angle=50.0,
                cutter_radius=63.5,
            ),
            validate_hypoid,
            hypoid_snapshot,
        ),
        snapshot(
            "planetary_anchor",
            "planetary",
            PlanetarySetParams.with_defaults(2.0, 24, 18, n_planets=3),
            validate_planetary,
            planetary_snapshot,
        ),
        snapshot(
            "planetary_valid_five",
            "planetary",
            PlanetarySetParams.with_defaults(2.0, 30, 20, n_planets=5),
            validate_planetary,
            planetary_snapshot,
        ),
    ]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-o",
        "--output",
        type=Path,
        default=REPOSITORY_ROOT / "cpp" / "tests" / "fixtures" / "python_reference.json",
    )
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps({"schema": 1, "fixtures": fixtures()}, indent=2, sort_keys=True)
        + "\n",
        encoding="utf-8",
    )
    print(f"wrote {len(fixtures())} fixtures to {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
