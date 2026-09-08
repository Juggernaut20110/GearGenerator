"""Build and constrain a skew-axis hypoid pair."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path

from ..hypoid import mesh
from ..hypoid.geometry import HypoidSetGeometry
from .assembly_common import (
    EXPECTED_COMPONENT_STATUS,
    add_component,
    add_mate,
    check_interference,
    component_axis,
    component_status,
    feature_pick,
    feature_of_type,
    measure_axis,
    measure_origin,
    origin_point,
    part_filename,
    place,
    plane_pick,
    point_pick,
    unfix,
)
from .common import BuildResult
from .hypoid_part import build_hypoid_gear
from .session import (
    FRONT_PLANE_NAMES,
    RIGHT_PLANE_NAMES,
    SW_MATE_ANGLE,
    SW_MATE_COINCIDENT,
    SW_MATE_DISTANCE,
    SW_MATE_GEAR,
    TOP_PLANE_NAMES,
    SwError,
    flag_methods,
    value_of,
)

MATE_GROUP_TYPE = "MateGroup"
GEAR_MATE_TYPE = "MateGearDim"


@dataclass
class HypoidSetResult:
    pinion: BuildResult
    gear: BuildResult
    assembly_title: str
    assembly_path: str | None
    shaft_angle_deg: float
    offset_mm: float
    clocking_deg: float
    measured_shaft_angle_deg: float
    measured_offset_mm: float
    interference_count: int = -1
    interference_volume_mm3: float = 0.0
    mates: tuple[str, ...] = ()
    gear_ratio: tuple[float, float] | None = None
    gear_mate_name: str | None = None
    gear_mate_reversed: bool = False
    pinion_status: str = "not checked"
    gear_status: str = "not checked"
    model: object = field(default=None, repr=False)

    @property
    def shaft_angle_error_deg(self):
        return self.measured_shaft_angle_deg - self.shaft_angle_deg

    @property
    def offset_error_mm(self):
        return self.measured_offset_mm - abs(self.offset_mm)

    @property
    def parts(self):
        return self.pinion, self.gear

    @property
    def gear_ratios(self):
        return (self.gear_ratio,) if self.gear_ratio else ()

    @property
    def articulates(self):
        return bool(self.gear_ratio) and self.pinion_status == self.gear_status == EXPECTED_COMPONENT_STATUS


def _persisted_gear_mate(model, name: str):
    """Find, name, and re-read the gear mate in the assembly feature tree.

    ``AddMate5`` returning an object is not enough: SOLIDWORKS can return a
    transient mate object while refusing to retain the feature.  Walking the
    MateGroup after a rebuild proves the mate that will be written to disk is
    present, and the deterministic name makes it obvious in the feature tree.
    """
    flag_methods(model, "EditRebuild3").EditRebuild3()
    group = feature_of_type(model, MATE_GROUP_TYPE, "assembly MateGroup")
    feature = flag_methods(group, "GetFirstSubFeature").GetFirstSubFeature()
    gear_mates = []
    while feature is not None:
        if str(value_of(feature, "GetTypeName2")) == GEAR_MATE_TYPE:
            gear_mates.append(feature)
        feature = flag_methods(feature, "GetNextSubFeature").GetNextSubFeature()
    if not gear_mates:
        raise SwError("gear mate was accepted but is absent from the assembly MateGroup")
    gear_mate = gear_mates[-1]
    gear_mate.Name = name
    if str(value_of(gear_mate, "Name")) != name:
        raise SwError(f"could not name the persisted gear mate {name!r}")
    return gear_mate


def add_mates(model, pinion_comp, gear_comp, geo: HypoidSetGeometry, reverse=False):
    unfix(model, pinion_comp, "pinion")
    unfix(model, gear_comp, "gear")
    origin = point_pick(origin_point(model, "the assembly origin"), "the assembly origin")
    top = plane_pick(TOP_PLANE_NAMES, "the assembly Top plane")
    right = plane_pick(RIGHT_PLANE_NAMES, "the assembly Right plane")
    front = plane_pick(FRONT_PLANE_NAMES, "the assembly Front plane")
    pinion_axis = feature_pick(component_axis(pinion_comp, "pinion"), "the pinion axis")
    gear_axis = feature_pick(component_axis(gear_comp, "gear"), "the gear axis")
    pinion_origin = point_pick(origin_point(pinion_comp, "the pinion origin"), "the pinion origin")
    gear_origin = point_pick(origin_point(gear_comp, "the gear origin"), "the gear origin")
    gear_position = mesh.gear_translation(geo)
    added = []

    def mate(picks, kind, what, **kwargs):
        add_mate(model, picks, kind, what, **kwargs)
        added.append(what)

    mate((pinion_origin, origin), SW_MATE_COINCIDENT, "pinion origin - assembly origin")
    mate((pinion_axis, top), SW_MATE_COINCIDENT, "pinion axis - Top plane")
    mate((pinion_axis, right), SW_MATE_COINCIDENT, "pinion axis - Right plane")
    # The gear axis is parallel to the Top plane and offset from it along the
    # pair's common normal.  Making it coincident first and then adding an
    # axis-to-axis distance is contradictory for every non-zero hypoid offset
    # and SOLIDWORKS correctly reports an over-defined assembly.  The initial
    # transform selects the signed side; this distance mate preserves it while
    # leaving rotation about both shafts free.
    if abs(geo.params.offset) < 1e-9:
        mate((gear_axis, top), SW_MATE_COINCIDENT, "gear axis - Top plane")
    else:
        mate(
            (gear_axis, top),
            SW_MATE_DISTANCE,
            f"hypoid offset {abs(geo.params.offset):g} mm",
            distance=abs(geo.params.offset),
            flip=geo.params.offset < 0.0,
        )
    # The axis-to-Top-plane mate fixes the gear's y translation, but an axis
    # can still slide in that plane.  Anchor the solved origin's x and z
    # coordinates as point-to-plane mates.  These remove translations only;
    # the origin point has no rotational constraint, so the shaft spin remains
    # available to the gear mate.
    for coordinate, plane, plane_name, label in (
        (gear_position[0], right, "Right", "x"),
        (gear_position[2], front, "Front", "z"),
    ):
        if abs(coordinate) < 1e-9:
            mate(
                (gear_origin, plane),
                SW_MATE_COINCIDENT,
                f"gear origin - {plane_name} plane",
            )
        else:
            mate(
                (gear_origin, plane),
                SW_MATE_DISTANCE,
                f"gear origin {label} = {coordinate:+g} mm from "
                f"{plane_name} plane",
                distance=abs(coordinate),
                flip=coordinate < 0.0,
            )
    mate((gear_axis, pinion_axis), SW_MATE_ANGLE,
         f"shaft angle {geo.params.shaft_angle:g} deg", angle=float(geo.params.sigma))
    ratio = mesh.gear_mate_ratio(geo.pinion.z, geo.gear.z)
    mate((pinion_axis, gear_axis), SW_MATE_GEAR,
         f"gear mate {geo.pinion.z}:{geo.gear.z}" + (" reversed" if reverse else ""),
         ratio=ratio, flip=bool(reverse))
    gear_mate_name = f"HypoidGearMate_{geo.pinion.z}_{geo.gear.z}"
    _persisted_gear_mate(model, gear_mate_name)
    return tuple(added), ratio, gear_mate_name


def build_hypoid_set(session, geo: HypoidSetGeometry, out_dir: str | Path,
                     save_assembly=True, mate=True, reverse_gear_mate=False):
    out_dir = Path(out_dir).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    pinion_path = out_dir / part_filename(geo, "pinion", prefix="hypoid_")
    gear_path = out_dir / part_filename(geo, "gear", prefix="hypoid_")
    pinion = build_hypoid_gear(session, geo, "pinion", str(pinion_path))
    gear = build_hypoid_gear(session, geo, "gear", str(gear_path))
    model = session.new_assembly()
    flag_methods(model, "EditRebuild3", "ViewZoomtofit2")
    pinion_comp = add_component(model, pinion_path)
    gear_comp = add_component(model, gear_path)
    clocking = mesh.gear_clocking(geo)
    place(session, pinion_comp, mesh.pinion_placement(geo), "pinion")
    place(session, gear_comp, mesh.gear_placement(geo.params.sigma, clocking), "gear",
          translation=mesh.gear_translation(geo))
    mates, ratio, gear_mate_name = (
        add_mates(model, pinion_comp, gear_comp, geo, reverse_gear_mate)
        if mate else ((), None, None)
    )
    model.EditRebuild3()
    model.ViewZoomtofit2()
    pinion_axis = measure_axis(pinion_comp)
    gear_axis = measure_axis(gear_comp)
    pinion_origin = measure_origin(pinion_comp)
    gear_origin = measure_origin(gear_comp)
    measured_axis = math.degrees(mesh.angle_between(pinion_axis, gear_axis))
    measured_offset = mesh.skew_axis_distance(
        pinion_origin, pinion_axis, gear_origin, gear_axis
    )
    count, volume = check_interference(model)
    path = None
    if save_assembly:
        module = f"{geo.params.module:g}".replace(".", "p")
        path = str(out_dir / f"hypoid_m{module}_z{geo.pinion.z}x{geo.gear.z}.sldasm")
        session.save(model, path)
    return HypoidSetResult(
        pinion=pinion, gear=gear, assembly_title=model.GetTitle,
        assembly_path=path, shaft_angle_deg=geo.params.shaft_angle,
        offset_mm=geo.params.offset, clocking_deg=math.degrees(clocking),
        measured_shaft_angle_deg=measured_axis, measured_offset_mm=measured_offset,
        interference_count=count, interference_volume_mm3=volume, mates=mates,
        gear_ratio=ratio, gear_mate_reversed=bool(reverse_gear_mate),
        gear_mate_name=gear_mate_name,
        pinion_status=component_status(pinion_comp) if mate else "not mated",
        gear_status=component_status(gear_comp) if mate else "not mated", model=model,
    )


__all__ = ["HypoidSetResult", "build_hypoid_set"]
