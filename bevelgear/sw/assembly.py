"""Build the full bevel set: both parts, then an assembly with them meshing.

The two components are positioned by writing their transforms directly rather
than by adding mates. A bevel pair needs the pitch apexes coincident, the axes
at the shaft angle, *and* the teeth clocked to interleave - three conditions
that are awkward to express as mates against generated geometry, and that the
mate solver can satisfy in the wrong way. The placement is exact arithmetic
(see `bevelgear.mesh`), so writing it straight in is both simpler and more
reliable. Mates can be layered on afterwards if the assembly needs to articulate.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path

from .. import mesh
from ..geometry import SetGeometry
from .part import BuildResult, build_gear
from .session import (
    SW_ADD_COMPONENT_CURRENT_CONFIG,
    SwError,
    doubles,
    flag_methods,
    require,
)


@dataclass
class SetResult:
    """Both parts plus the assembly, and the checks made on the result."""

    pinion: BuildResult
    gear: BuildResult
    assembly_title: str
    assembly_path: str | None
    shaft_angle_deg: float
    clocking_deg: float
    measured_shaft_angle_deg: float
    interference_count: int = -1
    interference_volume_mm3: float = 0.0
    model: object = field(default=None, repr=False)

    @property
    def shaft_angle_error_deg(self) -> float:
        return self.measured_shaft_angle_deg - self.shaft_angle_deg


def part_filename(geo: SetGeometry, member: str) -> str:
    m = geo.member(member)
    module = f"{geo.params.module:g}".replace(".", "p")
    return f"{member}_m{module}_z{m.z}.sldprt"


def _add_component(model, path: str):
    comp = model.AddComponent5(
        str(path),
        SW_ADD_COMPONENT_CURRENT_CONFIG,
        "",       # new config name, unused
        False,    # use config for part references
        "",       # existing config name
        0.0, 0.0, 0.0,
    )
    if comp is None:
        raise SwError(f"could not insert {path} into the assembly")
    return comp


def _place(session, comp, matrix, what: str) -> None:
    """Write a component's placement transform directly."""
    xform = require(
        session.math_utility().CreateTransform(doubles(mesh.to_array_data(matrix))),
        f"build the transform for the {what}",
    )
    comp.Transform2 = xform


def _measure_axis(comp) -> tuple[float, float, float]:
    """Read back where a component's own +Z axis actually points."""
    data = list(comp.Transform2.ArrayData)
    # Column-major: entries 6..8 are the image of the Z axis.
    return data[6], data[7], data[8]


def check_interference(model) -> tuple[int, float]:
    """Run SOLIDWORKS interference detection. Returns (count, total volume mm3).

    `TreatCoincidenceAsInterference` must be off: with zero backlash the tooth
    flanks touch exactly at the pitch point, and coincident faces are the
    correct answer there, not a fault.

    Some genuine interference is expected and worth reporting rather than
    hiding. Tredgold's back-cone construction is an approximation, so the
    flanks are not perfectly conjugate; the volume says how much that costs.
    Returns (-1, 0.0) if detection is unavailable.
    """
    try:
        idm = model.InterferenceDetectionManager
        if idm is None:
            return -1, 0.0
        idm.TreatCoincidenceAsInterference = False
        idm.TreatSubAssembliesAsComponents = False
        count = flag_methods(
            idm, "GetInterferenceCount", "GetInterferences", "Done"
        ).GetInterferenceCount()
        volume = 0.0
        if count:
            for itf in list(idm.GetInterferences() or []):
                try:
                    volume += float(itf.Volume) * 1e9  # m3 -> mm3
                except Exception:
                    pass
        try:
            idm.Done()
        except Exception:
            pass
        return int(count), volume
    except Exception:
        return -1, 0.0


def build_set(
    session,
    geo: SetGeometry,
    out_dir: str | Path,
    save_assembly: bool = True,
) -> SetResult:
    """Build both members and an assembly with their teeth meshing."""
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    pinion_path = out_dir / part_filename(geo, "pinion")
    gear_path = out_dir / part_filename(geo, "gear")

    pinion = build_gear(session, geo, "pinion", str(pinion_path))
    gear = build_gear(session, geo, "gear", str(gear_path))

    model = session.new_assembly()
    flag_methods(model, "EditRebuild3", "ViewZoomtofit2")

    pinion_comp = _add_component(model, pinion_path)
    gear_comp = _add_component(model, gear_path)

    clocking = mesh.gear_clocking(geo.gear.z)
    pinion_matrix = mesh.pinion_placement()
    gear_matrix = mesh.gear_placement(geo.params.sigma, clocking)

    _place(session, pinion_comp, pinion_matrix, "pinion")
    _place(session, gear_comp, gear_matrix, "gear")

    model.EditRebuild3()
    try:
        model.ViewZoomtofit2()
    except Exception:
        pass

    measured = math.degrees(
        mesh.angle_between(_measure_axis(pinion_comp), _measure_axis(gear_comp))
    )
    interference_count, interference_volume = check_interference(model)

    path = None
    if save_assembly:
        module = f"{geo.params.module:g}".replace(".", "p")
        path = str(
            out_dir / f"set_m{module}_z{geo.pinion.z}x{geo.gear.z}.sldasm"
        )
        session.save(model, path)

    return SetResult(
        pinion=pinion,
        gear=gear,
        assembly_title=model.GetTitle,
        assembly_path=path,
        shaft_angle_deg=geo.params.shaft_angle,
        clocking_deg=math.degrees(clocking),
        measured_shaft_angle_deg=measured,
        interference_count=interference_count,
        interference_volume_mm3=interference_volume,
        model=model,
    )
