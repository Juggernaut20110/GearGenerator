"""Lightweight 3D assembly preview primitives and gear scene generation.

This module deliberately has no Tkinter dependency.  The geometry builders
consume the same tooth sections and placement functions used by the existing
preview and SOLIDWORKS layers; only :func:`draw_scene3d` knows about a canvas.

Coordinate convention
---------------------
Parts are generated in their native frame with the gear axis along ``+Z``.
The camera's front view looks down ``+Z`` and shows the XY plane.  Positive
camera yaw turns the view toward ``+X`` (the projected horizontal coordinate
becomes world Z), and positive pitch turns it toward ``+Y`` (the projected
vertical coordinate becomes world -Z).  This makes the named Front, Top, and
Right views useful for the cylindrical and conical gear families alike.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from .placement import Matrix3, angular_velocity_ratio, apply, matmul, rot_z
from .preview import Style
from .bevel import mesh as bevel_mesh
from .bevel.geometry import SetGeometry as BevelSetGeometry
from .bevel.geometry import tooth_space_section as bevel_tooth_space_section
from .hypoid import mesh as hypoid_mesh
from .hypoid.geometry import HypoidSetGeometry
from .hypoid.geometry import tooth_space_section as hypoid_tooth_space_section
from .hypoid.geometry import section_cone_bounds as hypoid_section_cone_bounds
from .planetary import mesh as planetary_mesh
from .planetary.geometry import PlanetarySetGeometry
from .spur import mesh as spur_mesh
from .spur.geometry import SpurSetGeometry
from .spur.geometry import tooth_space_section as spur_tooth_space_section
from .spur.preview import tooth_loop as spur_tooth_loop
from .preview import tooth_boundary

Point3 = tuple[float, float, float]
Point2 = tuple[float, float]


@dataclass(frozen=True)
class Polyline3D:
    """One drawable 3D polyline, in millimetres."""

    points: list[Point3]
    style: str = "outer"
    closed: bool = False
    # The member label is metadata used by the mesh-position control.  It is
    # optional so callers can still construct the small suggested data object.
    member: str = ""

    def drawn_points(self) -> list[Point3]:
        if self.closed and len(self.points) > 2:
            return [*self.points, self.points[0]]
        return list(self.points)


@dataclass(frozen=True)
class Scene3D:
    """A complete assembly wireframe in world coordinates."""

    polylines: list[Polyline3D] = field(default_factory=list)
    title: str = ""
    key: str = "assembly"
    legend: list[tuple[str, str]] = field(default_factory=list)

    def bounds(self) -> tuple[float, float, float, float, float, float]:
        """Return ``(xmin, ymin, zmin, xmax, ymax, zmax)`` over all points."""
        points = [point for line in self.polylines for point in line.points]
        if not points:
            return (-1.0, -1.0, -1.0, 1.0, 1.0, 1.0)
        xs, ys, zs = zip(*points)
        return min(xs), min(ys), min(zs), max(xs), max(ys), max(zs)


@dataclass
class Camera:
    """A small orthographic orbit camera.

    ``zoom`` is a multiplier over the scale selected by :meth:`fit`.
    ``pan_x`` and ``pan_y`` are canvas pixels, so camera motion does not need
    to regenerate or mutate a scene.
    """

    target: Point3 = (0.0, 0.0, 0.0)
    yaw: float = 0.0
    pitch: float = 0.0
    zoom: float = 1.0
    pan_x: float = 0.0
    pan_y: float = 0.0
    _fit_scale: float = field(default=1.0, init=False, repr=False)

    MIN_ZOOM = 0.05
    MAX_ZOOM = 100.0
    MAX_PITCH = math.pi / 2.0 - 0.02

    def _camera_coordinates(self, point: Point3) -> tuple[float, float, float]:
        """Transform world coordinates into right/up/depth camera axes."""
        x = point[0] - self.target[0]
        y = point[1] - self.target[1]
        z = point[2] - self.target[2]

        # Orbit around world Y.  At yaw=0 the depth axis is world +Z.
        cy, sy = math.cos(self.yaw), math.sin(self.yaw)
        right = cy * x + sy * z
        depth = -sy * x + cy * z

        # Orbit around the yawed camera's horizontal axis.  Positive pitch
        # moves the camera toward +Y and exposes the gear's axial Z direction.
        cp, sp = math.cos(self.pitch), math.sin(self.pitch)
        up = cp * y - sp * depth
        depth = sp * y + cp * depth
        return right, up, depth

    def project(self, point: Point3, width: float = 0.0, height: float = 0.0) -> Point2:
        """Project a world point orthographically into canvas coordinates."""
        right, up, _depth = self._camera_coordinates(point)
        scale = self._fit_scale * self.zoom
        return (
            width * 0.5 + self.pan_x + right * scale,
            height * 0.5 + self.pan_y - up * scale,
        )

    project_point = project

    def orbit(self, dx: float, dy: float, sensitivity: float = 0.01) -> None:
        """Orbit by screen-pixel deltas."""
        self.yaw += dx * sensitivity
        self.pitch = max(
            -self.MAX_PITCH,
            min(self.MAX_PITCH, self.pitch - dy * sensitivity),
        )

    def pan(self, dx: float, dy: float) -> None:
        self.pan_x += dx
        self.pan_y += dy

    def zoom_by(self, factor: float) -> None:
        if factor <= 0.0 or not math.isfinite(factor):
            return
        self.zoom = max(self.MIN_ZOOM, min(self.MAX_ZOOM, self.zoom * factor))

    def zoom_at(self, factor: float, x: float, y: float, width: float, height: float) -> None:
        """Zoom while keeping the canvas point ``(x, y)`` stationary."""
        old_zoom = self.zoom
        old_pan_x, old_pan_y = self.pan_x, self.pan_y
        self.zoom_by(factor)
        actual = self.zoom / old_zoom if old_zoom else 1.0
        cx, cy = width * 0.5, height * 0.5
        self.pan_x = (x - cx) - actual * (x - cx - old_pan_x)
        self.pan_y = (y - cy) - actual * (y - cy - old_pan_y)

    def set_view(self, name: str) -> None:
        """Set a named engineering view: Front, Top, Right, or Iso."""
        key = name.strip().lower()
        views = {
            "front": (0.0, 0.0),
            "rear": (math.pi, 0.0),
            "top": (0.0, math.pi / 2.0),
            "bottom": (0.0, -math.pi / 2.0),
            "right": (math.pi / 2.0, 0.0),
            "left": (-math.pi / 2.0, 0.0),
            "iso": (math.radians(45.0), math.radians(30.0)),
            "isometric": (math.radians(45.0), math.radians(30.0)),
        }
        try:
            self.yaw, self.pitch = views[key]
        except KeyError:
            raise ValueError(f"unknown camera view {name!r}") from None

    def front(self) -> None:
        self.set_view("front")

    def top(self) -> None:
        self.set_view("top")

    def right(self) -> None:
        self.set_view("right")

    def iso(self) -> None:
        self.set_view("iso")

    def fit(
        self,
        bounds: tuple[float, float, float, float, float, float],
        width: float,
        height: float,
        margin: float = 20.0,
    ) -> "Camera":
        """Fit an axis-aligned scene bound in the current camera orientation."""
        xmin, ymin, zmin, xmax, ymax, zmax = bounds
        self.target = (
            0.5 * (xmin + xmax),
            0.5 * (ymin + ymax),
            0.5 * (zmin + zmax),
        )
        corners = [
            (x, y, z)
            for x in (xmin, xmax)
            for y in (ymin, ymax)
            for z in (zmin, zmax)
        ]
        projected = [self._camera_coordinates(point) for point in corners]
        span_x = max(max(p[0] for p in projected) - min(p[0] for p in projected), 1e-9)
        span_y = max(max(p[1] for p in projected) - min(p[1] for p in projected), 1e-9)
        avail_x = max(width - 2.0 * margin, 1.0)
        avail_y = max(height - 2.0 * margin, 1.0)
        self._fit_scale = min(avail_x / span_x, avail_y / span_y)
        self.zoom = 1.0
        self.pan_x = self.pan_y = 0.0
        return self

    def fit_scene(self, scene: Scene3D, width: float, height: float, margin: float = 20.0) -> "Camera":
        return self.fit(scene.bounds(), width, height, margin)


STYLES3D: dict[str, Style] = {
    "pinion": Style("#1769aa", 1.8),
    "gear": Style("#c45a16", 1.8),
    "sun": Style("#1769aa", 1.8),
    "planet": Style("#6f8391", 1.6),
    "ring": Style("#b53d3d", 1.8),
    "reference": Style("#777777", 1.0, (5, 3)),
    "axis": Style("#555555", 1.0, (8, 3, 2, 3)),
    "pitch": Style("#2f8f4f", 1.0, (7, 3, 2, 3)),
}


def style_for_3d(name: str) -> Style:
    return STYLES3D.get(name, Style("#333333", 1.0))


def draw_scene3d(canvas, scene: Scene3D, camera: Camera, width: float, height: float, legend: bool = True) -> None:
    """Draw a complete 3D scene using only Canvas line primitives."""
    for line in scene.polylines:
        points = line.drawn_points()
        if len(points) < 2 or not all(math.isfinite(v) for point in points for v in point):
            continue
        flat: list[float] = []
        for point in points:
            x, y = camera.project(point, width, height)
            flat.extend((x, y))
        style = style_for_3d(line.style)
        kwargs = {"fill": style.colour, "width": style.width}
        if style.dash:
            kwargs["dash"] = style.dash
        canvas.create_line(*flat, **kwargs)

    if legend:
        for i, (style_name, label) in enumerate(scene.legend):
            style = style_for_3d(style_name)
            y = 18.0 + 14.0 * i
            kwargs = {"fill": style.colour, "width": max(style.width, 1.5)}
            if style.dash:
                kwargs["dash"] = style.dash
            canvas.create_line(12.0, y, 34.0, y, **kwargs)
            canvas.create_text(40.0, y, text=label, anchor="w", fill="#444444", font=("TkDefaultFont", 7))


def _finite_scene(scene: Scene3D) -> Scene3D:
    if not all(math.isfinite(value) for line in scene.polylines for point in line.points for value in point):
        raise ValueError("3D preview contains a non-finite point")
    return scene


def _lerp_positions(start: float, end: float, count: int) -> list[float]:
    count = max(2, count)
    return [start + (end - start) * i / (count - 1) for i in range(count)]


def _decimate(points: list[Point3], maximum: int = 56) -> list[Point3]:
    """Reduce rendering samples while retaining both ends of a profile."""
    if len(points) <= maximum:
        return points
    if maximum < 2:
        return points[:1]
    return [points[round(i * (len(points) - 1) / (maximum - 1))] for i in range(maximum)]


def _transformed(point: Point3, rotation: Matrix3, translation: Point3) -> Point3:
    turned = apply(rotation, point)
    return tuple(turned[i] + translation[i] for i in range(3))


def _circle(radius: float, z: float, samples: int = 73) -> list[Point3]:
    return [
        (radius * math.cos(2.0 * math.pi * i / (samples - 1)),
         radius * math.sin(2.0 * math.pi * i / (samples - 1)), z)
        for i in range(samples)
    ]


def _add_member_wireframe(
    lines: list[Polyline3D],
    sections: list[list[Point3]],
    teeth: int,
    angular_pitch: float,
    clocking: float,
    rotation: Matrix3,
    translation: Point3,
    style: str,
    member: str,
    connector_stride: int = 20,
) -> None:
    """Add tooth loops and sparse longitudinal connectors for one member."""
    for tooth in range(teeth):
        tooth_sections = [
            [_transformed(point, rot_z(tooth * angular_pitch + clocking), (0.0, 0.0, 0.0)) for point in section]
            for section in sections
        ]
        for section in tooth_sections:
            transformed = [_transformed(point, rotation, translation) for point in section]
            lines.append(Polyline3D(transformed, style, True, member))
        for first, second in zip(tooth_sections, tooth_sections[1:]):
            limit = min(len(first), len(second))
            for index in range(0, limit, max(1, connector_stride)):
                lines.append(
                    Polyline3D(
                        [
                            _transformed(first[index], rotation, translation),
                            _transformed(second[index], rotation, translation),
                        ],
                        style,
                        False,
                        member,
                    )
                )


def _add_axis(
    lines: list[Polyline3D], rotation: Matrix3, translation: Point3,
    length: float, member: str,
) -> None:
    lines.append(
        Polyline3D(
            [_transformed((0.0, 0.0, 0.0), rotation, translation),
             _transformed((0.0, 0.0, length), rotation, translation)],
            "axis", False, member,
        )
    )


def _spur_sections(geo: SpurSetGeometry, role: str, count: int = 5) -> list[list[Point3]]:
    member = geo.member(role)
    sections: list[list[Point3]] = []
    for z in _lerp_positions(0.0, geo.params.face_width, count):
        section = spur_tooth_space_section(geo, role, z=z)
        loop = spur_tooth_loop(section, member.angular_pitch)
        sections.append(_decimate([spur_tooth_point(point, section) for point in loop]))
    return sections


def spur_tooth_point(point: Point2, section) -> Point3:
    """Use the spur geometry module's authoritative axial conversion."""
    from .spur.geometry import to_axial_3d

    return to_axial_3d(point[0], point[1], section.phase, section.z)


def _bevel_sections(geo: BevelSetGeometry, role: str, count: int = 5) -> list[list[Point3]]:
    member = geo.member(role)
    positions = _lerp_positions(geo.inner_cone_dist, geo.outer_cone_dist, count)
    result: list[list[Point3]] = []
    for cone_dist in positions:
        section = bevel_tooth_space_section(geo, role, cone_dist=cone_dist)
        loop = tooth_boundary(section.segments, 2.0 * math.pi / member.virtual_teeth)
        if not loop:
            loop = section.loop_2d
        result.append(_decimate([section_point(point, section) for point in loop]))
    return [list(section) for section in result]


def section_point(point: Point2, section) -> Point3:
    from .bevel.geometry import to_cone_3d

    return to_cone_3d(point[0], point[1], section.pitch_angle, section.cone_apex_z, section.phase)


def _hypoid_sections(geo: HypoidSetGeometry, role: str, count: int = 5) -> list[list[Point3]]:
    member = geo.member(role)
    bounds = hypoid_section_cone_bounds(geo, role)
    positions = _lerp_positions(bounds.tooth_face_inner, bounds.tooth_face_outer, count)
    result: list[list[Point3]] = []
    for cone_dist in positions:
        section = hypoid_tooth_space_section(geo, role, cone_dist=cone_dist)
        loop = tooth_boundary(section.segments, 2.0 * math.pi / member.virtual_teeth)
        if not loop:
            loop = section.loop_2d
        result.append(_decimate([hypoid_section_point(point, section) for point in loop]))
    return [list(section) for section in result]


def hypoid_section_point(point: Point2, section) -> Point3:
    from .hypoid.geometry import to_cone_3d

    return to_cone_3d(point[0], point[1], section.pitch_angle, section.cone_apex_z, section.phase)


def _pair_phase(geo, mesh_position: float) -> float:
    """Return the mating member's angle for a pinion angle in radians."""
    return mesh_position / angular_velocity_ratio(
        geo.pinion.z, geo.gear.z, getattr(geo.params, "internal", False)
    )


def _spur_scene(geo: SpurSetGeometry, mesh_position: float) -> Scene3D:
    p = geo.params
    lines: list[Polyline3D] = []
    pinion_rotation = matmul(spur_mesh.pinion_placement(), rot_z(mesh_position))
    gear_rotation = matmul(
        spur_mesh.gear_placement(spur_mesh.clocking_for(geo)),
        rot_z(_pair_phase(geo, mesh_position)),
    )
    pinion_translation = (0.0, 0.0, 0.0)
    gear_translation = spur_mesh.gear_translation(geo)
    for role, rotation, translation, style in (
        ("pinion", pinion_rotation, pinion_translation, "pinion"),
        ("gear", gear_rotation, gear_translation, "gear"),
    ):
        member = geo.member(role)
        sections = _spur_sections(geo, role)
        _add_member_wireframe(
            lines, sections, member.z, member.angular_pitch, 0.0,
            rotation, translation, style, role,
        )
        for z in (0.0, p.face_width):
            lines.append(
                Polyline3D(
                    [_transformed(point, rotation, translation) for point in _circle(member.working_r, z, 49)],
                    "pitch", True, role,
                )
            )
        _add_axis(lines, rotation, translation, p.face_width, role)
    # A centre line makes internal and external placement immediately clear.
    lines.append(Polyline3D([(0.0, 0.0, 0.0), gear_translation], "reference"))
    arrangement = "internal ring" if p.internal else "external pair"
    hand = "straight" if not p.helix_angle else f"{p.helix_angle:g} deg {p.hand}"
    return _finite_scene(Scene3D(
        lines,
        f"3D assembly - {arrangement}, {hand}, {p.z1}:{p.z2}",
        legend=[("pinion", "pinion"), ("gear", "gear/ring"), ("pitch", "working pitch"), ("axis", "axes")],
    ))


def _bevel_scene(geo: BevelSetGeometry, mesh_position: float) -> Scene3D:
    p = geo.params
    clocking = bevel_mesh.gear_clocking(geo.gear.z)
    pinion_rotation = matmul(bevel_mesh.pinion_placement(), rot_z(mesh_position))
    gear_rotation = matmul(
        bevel_mesh.gear_placement(p.sigma, clocking),
        rot_z(_pair_phase(geo, mesh_position)),
    )
    lines: list[Polyline3D] = []
    for role, rotation, style in (
        ("pinion", pinion_rotation, "pinion"),
        ("gear", gear_rotation, "gear"),
    ):
        member = geo.member(role)
        sections = _bevel_sections(geo, role)
        _add_member_wireframe(
            lines, sections, member.z, member.angular_pitch, 0.0,
            rotation, (0.0, 0.0, 0.0), style, role,
        )
        _add_axis(lines, rotation, (0.0, 0.0, 0.0), member.crown_to_apex, role)
    trace = "straight" if geo.trace is None else "spiral/Zerol"
    return _finite_scene(Scene3D(
        lines,
        f"3D assembly - bevel {trace}, {p.z1}:{p.z2}, shaft {p.shaft_angle:g} deg",
        legend=[("pinion", "pinion"), ("gear", "gear"), ("axis", "shaft axes")],
    ))


def _hypoid_scene(geo: HypoidSetGeometry, mesh_position: float) -> Scene3D:
    p = geo.params
    clocking = hypoid_mesh.gear_clocking(geo)
    pinion_rotation = matmul(hypoid_mesh.pinion_placement(geo), rot_z(mesh_position))
    gear_rotation = matmul(
        hypoid_mesh.gear_placement(p.sigma, clocking),
        rot_z(_pair_phase(geo, mesh_position)),
    )
    lines: list[Polyline3D] = []
    for role, rotation, translation, style in (
        ("pinion", pinion_rotation, (0.0, 0.0, 0.0), "pinion"),
        ("gear", gear_rotation, hypoid_mesh.gear_translation(geo), "gear"),
    ):
        member = geo.member(role)
        sections = _hypoid_sections(geo, role)
        _add_member_wireframe(
            lines, sections, member.z, 2.0 * math.pi / member.z, 0.0,
            rotation, translation, style, role,
        )
        _add_axis(lines, rotation, translation, member.outer_tip_z, role)
    return _finite_scene(Scene3D(
        lines,
        f"3D assembly - hypoid {p.z1}:{p.z2}, shaft {p.shaft_angle:g} deg, offset {p.offset:g} mm",
        legend=[("pinion", "pinion"), ("gear", "gear"), ("axis", "skew axes")],
    ))


def _planetary_scene(geo: PlanetarySetGeometry) -> Scene3D:
    p = geo.params
    lines: list[Polyline3D] = []
    planet_sections = _spur_sections(geo.sun_planet, "gear")
    members = (
        ("sun", geo.sun_planet, "pinion", planetary_mesh.sun_clocking(), (0.0, 0.0, 0.0), "sun"),
        ("ring", geo.planet_ring, "gear", planetary_mesh.ring_clocking(geo, 0), (0.0, 0.0, 0.0), "ring"),
    )
    for role_name, pair, pair_role, clocking, translation, style in members:
        member = pair.member(pair_role)
        sections = _spur_sections(pair, pair_role)
        rotation = planetary_mesh.member_placement(clocking)
        _add_member_wireframe(
            lines, sections, member.z, member.angular_pitch, 0.0,
            rotation, translation, style, role_name,
        )
        _add_axis(lines, rotation, translation, p.face_width, role_name)
        if role_name == "ring":
            for z in (0.0, p.face_width):
                lines.append(
                    Polyline3D(
                        [_transformed(point, rotation, translation) for point in _circle(geo.ring_rim_radius, z, 73)],
                        "ring", True, role_name,
                    )
                )
    for index in range(p.n_planets):
        translation = planetary_mesh.planet_translation(geo, index)
        clocking = planetary_mesh.planet_clocking(geo, index)
        member = geo.planet
        _add_member_wireframe(
            lines, planet_sections, member.z,
            member.angular_pitch, 0.0, planetary_mesh.member_placement(clocking),
            translation, "planet", f"planet {index}",
        )
        _add_axis(lines, planetary_mesh.member_placement(clocking), translation, p.face_width, f"planet {index}")
    lines.append(Polyline3D([(0.0, 0.0, 0.0), (geo.centre_distance, 0.0, 0.0)], "reference"))
    return _finite_scene(Scene3D(
        lines,
        f"3D assembly - planetary train, sun {p.z_sun}, {p.n_planets} planets of {p.z_planet}, ring {p.z_ring}",
        legend=[("sun", "sun"), ("planet", "planets"), ("ring", "ring"), ("axis", "axes")],
    ))


def build_scene(geo, mesh_position: float = 0.0) -> Scene3D:
    """Build the complete 3D assembly scene for any supported gear set.

    ``mesh_position`` is a pinion rotation in radians.  Planetary scenes keep
    their static placement because the current generator intentionally models
    a carrier-stationary train; no unsupported carrier kinematics are implied.
    """
    if isinstance(geo, PlanetarySetGeometry):
        return _planetary_scene(geo)
    if isinstance(geo, SpurSetGeometry):
        return _spur_scene(geo, mesh_position)
    if isinstance(geo, BevelSetGeometry):
        return _bevel_scene(geo, mesh_position)
    if isinstance(geo, HypoidSetGeometry):
        return _hypoid_scene(geo, mesh_position)
    raise TypeError(f"unsupported geometry type {type(geo).__name__}")


build_assembly_scene = build_scene
OrthographicCamera = Camera
draw_scene = draw_scene3d


__all__ = [
    "Camera", "Point2", "Point3", "Polyline3D", "Scene3D", "STYLES3D",
    "OrthographicCamera", "build_assembly_scene", "build_scene", "draw_scene",
    "draw_scene3d", "style_for_3d",
]
