"""How the two bevel members sit relative to each other. Pure math - no COM.

Coordinate system, matching `geometry`: each part is built with its own axis
along +Z, its pitch apex at the origin, and a tooth **space** centred on the
part's +X direction.

In the assembly the pinion keeps that frame unchanged. The gear is rotated so
that the two pitch cones share the apex and touch along a common line:

    contact line   u  = (sin d1, 0, cos d1)      in the XZ plane
    gear axis      ag = (sin S,  0, cos S)       S = shaft angle

The angle between them is S - d1 = d2, so the gear's pitch cone is tangent to
the pinion's along u, which is exactly the meshing condition.

Rotating the gear about Y by S puts its axis on `ag`. It also means the gear's
own +X direction maps to (cos S, 0, -sin S), and decomposing the contact line
against that gives

    u = cos(d2) * ag  -  sin(d2) * (cos S, 0, -sin S)

so the contact line lies at angle **pi** around the gear's axis, not 0. That is
what `placement.gear_clocking` is measured against.

Only the arrangement itself lives here. The clocking, the speed ratio and the
matrix packing are shared with the other gear types and sit in
`gears.placement`; they are re-exported so callers can keep saying `mesh.x`.
"""

from __future__ import annotations

from ..placement import (
    Matrix3,
    angle_between,
    angular_velocity_ratio,
    apply,
    axis_of,
    gear_clocking,
    gear_mate_ratio,
    matmul,
    pinion_placement,
    rot_y,
    rot_z,
    to_array_data,
)

__all__ = [
    "Matrix3",
    "angle_between",
    "angular_velocity_ratio",
    "apply",
    "axis_of",
    "gear_clocking",
    "gear_mate_ratio",
    "gear_placement",
    "pinion_placement",
    "to_array_data",
]


def gear_placement(shaft_angle: float, clocking: float) -> Matrix3:
    """Spin the gear about its own axis, then tilt it to the shaft angle."""
    return matmul(rot_y(shaft_angle), rot_z(clocking))
