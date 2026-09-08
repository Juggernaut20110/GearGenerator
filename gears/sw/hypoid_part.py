"""SOLIDWORKS part builder for the hypoid geometry package."""

from __future__ import annotations

from ..hypoid.geometry import (
    HypoidSetGeometry,
    section_cone_bounds,
    section_cone_distances,
    tooth_space_section,
)
from ..bevel.geometry import to_cone_3d
from .bevel_part import _dimension_blank, _section_curves
from .common import (
    AXIS_FEATURE_NAME,
    BuildResult,
    close_and_revolve_blank,
    constrain_blank,
    create_axis,
    draw_curves_3d,
    drop_offcut,
    loft_cut,
    measure_part,
    pattern_teeth,
    sketch_blank_outline,
)
from .session import MARK_LOFT_GUIDE


def _blank(app, model, geo: HypoidSetGeometry, member: str):
    from ..hypoid.geometry import blank_outline

    outline = blank_outline(geo, member)
    mgr, axis, lines = sketch_blank_outline(model, outline)
    constrain_blank(model, axis, lines, outline)
    dims, variables = _dimension_blank(app, model, geo, member, axis, lines, outline)
    return close_and_revolve_blank(model, mgr, dims, variables)


def _section(model, geo: HypoidSetGeometry, member: str, cone_dist: float, bounds=None):
    section = tooth_space_section(geo, member, cone_dist, split_cap=True)
    if bounds is None:
        bounds = section_cone_bounds(geo, member)
    kind = "loft-only extension" if bounds.is_loft_extension(cone_dist) else "physical tooth face"
    return draw_curves_3d(
        model,
        _section_curves(section),
        f"hypoid Tredgold section A={cone_dist:.4f} ({kind})",
    )


def _guide_points(geo: HypoidSetGeometry, member: str, distances: list[float]):
    """Trace the split-cap vertex through every loft profile.

    A hypoid tooth is spiral on both members.  Multiple rotated profiles bound
    the approximation, while this guide gives SOLIDWORKS an unambiguous common
    vertex and connector order.  Every section distance is included exactly;
    intermediate samples merely smooth the guide between them.
    """
    points = []
    samples_per_interval = 8
    for interval, (a, b) in enumerate(zip(distances, distances[1:])):
        for i in range(samples_per_interval + 1):
            if interval and i == 0:
                continue
            cone_dist = a + (b - a) * i / samples_per_interval
            section = tooth_space_section(
                geo, member, cone_dist, split_cap=True
            )
            points.append(
                to_cone_3d(
                    section.r_cap,
                    0.0,
                    section.pitch_angle,
                    section.cone_apex_z,
                    section.phase,
                )
            )
    return points


def build_hypoid_gear(session, geo: HypoidSetGeometry, member: str,
                      save_path: str | None = None) -> BuildResult:
    """Build one hypoid member as a revolved blank plus patterned loft cut."""
    m = geo.member(member)
    model = session.new_part()
    axis = create_axis(model)
    _blank(session.app, model, geo, member)
    bounds = section_cone_bounds(geo, member)
    distances = section_cone_distances(geo, member)
    sections = [
        _section(model, geo, member, distance, bounds)
        for distance in distances
    ]
    guide = draw_curves_3d(
        model,
        [("hypoid cap guide", _guide_points(geo, member, distances))],
        "hypoid loft guide",
    )
    cut = loft_cut(
        model, sections, guides=(guide,), guide_mark=MARK_LOFT_GUIDE
    )
    drop_offcut(model)
    pattern_teeth(model, cut, axis, m.z)
    return measure_part(session, model, member, m.z, m.outside_dia / 2.0, save_path)


__all__ = ["AXIS_FEATURE_NAME", "BuildResult", "build_hypoid_gear"]
